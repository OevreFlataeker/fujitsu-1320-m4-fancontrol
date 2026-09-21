#!/usr/bin/env python3
"""TX1320 M4 iRMC fan control using the hardware-tested Fujitsu PW protocol.

Writes are restricted to system slot 1 and CPU slot 2. PSU slot 12 is read-only.
Percent means requested PWM, not percent of maximum RPM. Overrides can reduce
cooling below automatic control; they persist after this program exits. Use clear
explicitly. This is a manual override tool, not a temperature-control daemon.
Requires Python 3.9+ and local ipmitool access (normally sudo, interface open).
"""
import argparse
import math
import shlex
import subprocess
import time

IANA = [0x80, 0x28, 0x00]
CPU, SYSTEM = 2, 1
WRITABLE_SLOTS = {CPU, SYSTEM}
LABELS = {
    CPU: 'CPU', SYSTEM: 'System',
    3: 'Performance profile', 5: 'Emergency profile',
    12: 'PSUs (read-only)',
}


def percent_to_raw(percent):
    """Round a 0..100 percent request to the nearest 8-bit PWM value."""
    if not math.isfinite(percent) or not 0 <= percent <= 100:
        raise ValueError('percentage must be finite and in range 0..100')
    return int(percent * 255 / 100 + 0.5)


def write_payload(slot, value=None):
    """PW: omit value to release one override; zero instead forces zero PWM.

    Enforce the slot allowlist here, at the payload boundary. No broadcast or
    arbitrary-slot write facility is provided, including for clear operations.
    """
    if slot not in WRITABLE_SLOTS:
        raise ValueError('writes are limited to CPU slot 2 and system slot 1')
    if value is not None and (not isinstance(value, int) or not 0 <= value <= 255):
        raise ValueError('raw PWM must be an integer in range 0..255')
    return IANA + [0x2d, ord('P'), ord('W'), slot] + ([] if value is None else [value])


def read_payload(indices):
    if not 1 <= len(indices) <= 31 or any(not 0 <= i <= 31 for i in indices):
        raise ValueError('read requires 1..31 indices in range 0..31')
    return IANA + [0x2d, ord('F'), ord('R'), 1, len(indices)] + [v for i in indices for v in (i, 0)]


def run(args, tail):
    cmd = [args.ipmitool, '-I', args.interface] + tail
    if args.dry_run:
        print(shlex.join(cmd))
        return None
    proc = subprocess.run(cmd, text=True, capture_output=True, timeout=30)
    if proc.returncode:
        raise RuntimeError(proc.stderr.strip() or proc.stdout.strip() or f'ipmitool exit {proc.returncode}')
    return proc.stdout


def raw(args, payload):
    out = run(args, ['raw', '0x2e', '0xf5'] + [f'0x{x:02x}' for x in payload])
    if out is None:
        return None
    try:
        result = [int(t, 16) for t in out.split()]
    except ValueError as exc:
        raise RuntimeError(f'Unexpected non-hex response: {out!r}') from exc
    if any(not 0 <= b <= 255 for b in result) or result[:3] != IANA:
        raise RuntimeError(f'Unexpected OEM response: {out!r}')
    return result


def decode_read(result, indices):
    """FR returns IANA/version/count followed by (index+flags, cached byte)."""
    if result[:3] != IANA or len(result) != 5 + 2 * len(indices) or result[3:5] != [1, len(indices)]:
        raise RuntimeError(f'Unexpected FR response length/header: {result}')
    rows = []
    for i, expected in enumerate(indices):
        flags, value = result[5+2*i:7+2*i]
        if flags & 0x3f != expected:
            raise RuntimeError('FR response index mismatch')
        rows.append((expected, bool(flags & 0x40), bool(flags & 0x80), value))
    return rows


def cached_percent(slot, value):
    # Only apply scales established for this board. Inactive slots may be stale.
    if slot in WRITABLE_SLOTS:
        return f'{value * 100 / 255:.1f}% PWM'
    if slot == 12:
        return f'{value}% PSU request' if value <= 100 else 'out of PSU range'
    return 'unknown scale'


def read(args, indices=None):
    if indices is None:
        indices = list(range(32))
    if any(not 0 <= i <= 31 for i in indices):
        raise ValueError('read indices must be in range 0..31')
    print(f'{"Slot":6}{"Control":22}{"Active":8}{"Forced":8}{"Cached raw":12}Cached request')
    for start in range(0, len(indices), 16):
        batch = indices[start:start+16]
        result = raw(args, read_payload(batch))
        if result is not None:
            for slot, active, forced, value in decode_read(result, batch):
                text = cached_percent(slot, value) if active else 'inactive / possibly stale'
                print(f'{slot:02d}    {LABELS.get(slot, "Unmapped"):22}{str(active):8}{str(forced):8}{value:<12} {text}')
    print('Cached requests are software values, not measured duty or RPM. PSU requests use 0..100; CPU/system use 0..255.')


def fans(args):
    out = run(args, ['sdr', 'type', 'fan'])
    if out is not None:
        print(out, end='' if out.endswith('\n') else '\n')


def change(args, targets):
    """Write sequentially; preserve already-applied changes if a later call fails.

    PW acknowledgement contains active/forced bitmaps for slots 0..7. Verify
    only the selected bit. Acknowledgement does not verify physical fan speed.
    """
    for slot, value in targets:
        print(f'{LABELS[slot]} (slot {slot}): ' + ('restore automatic control' if value is None else f'request {value*100/255:.1f}% PWM, raw {value} (0x{value:02x})'), flush=True)
        try:
            result = raw(args, write_payload(slot, value))
            if result is not None:
                if len(result) != 5 or not result[3] & (1 << slot):
                    raise RuntimeError(f'unexpected PW acknowledgement or inactive slot: {result}')
                if bool(result[4] & (1 << slot)) != (value is not None):
                    raise RuntimeError(f'override state did not match request: {result}')
        except (RuntimeError, OSError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError(f'{exc}. Some requests may already have applied; inspect readout and use clear --cpu / --system as needed.') from exc
    if not args.dry_run:
        time.sleep(2)
    read(args, [slot for slot, _ in targets])
    fans(args)


def build_parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--ipmitool', default='ipmitool')
    p.add_argument('-I', '--interface', default='open')
    p.add_argument('--dry-run', action='store_true', help='print commands without executing them')
    subs = p.add_subparsers(dest='command', required=True)
    s = subs.add_parser('set', help='set CPU and/or system PWM percentages; unspecified fans are untouched')
    s.add_argument('--cpu', type=float, metavar='PERCENT')
    s.add_argument('--system', type=float, metavar='PERCENT')
    s.add_argument('--allow-low', action='store_true', help='permit requests below 10%%; this threshold is not a hardware safety guarantee')
    c = subs.add_parser('clear', help='return selected CPU/system controls to automatic')
    c.add_argument('--cpu', action='store_true')
    c.add_argument('--system', action='store_true')
    r = subs.add_parser('read', help='read all slots or selected decimal/hex indices')
    r.add_argument('indices', nargs='*', type=lambda s: int(s, 0))
    subs.add_parser('sdr', help='show fan RPM sensors')
    w = subs.add_parser('watch', help='repeat control readout and fan RPM readings')
    w.add_argument('-n', '--interval', type=float, default=5)
    return p


def main(argv=None):
    p = build_parser()
    a = p.parse_args(argv)
    if a.command == 'set':
        targets = []
        for slot, percent in [(CPU, a.cpu), (SYSTEM, a.system)]:
            if percent is not None:
                try:
                    value = percent_to_raw(percent)
                except ValueError as exc:
                    p.error(str(exc))
                if percent < 10 and not a.allow_low:
                    p.error('requests below 10% require --allow-low')
                targets.append((slot, value))
        if not targets:
            p.error('set requires --cpu and/or --system')
        # Validate every argument before executing the first write.
        change(a, targets)
    elif a.command == 'clear':
        targets = [(slot, None) for slot, selected in [(CPU, a.cpu), (SYSTEM, a.system)] if selected]
        if not targets:
            p.error('clear requires --cpu and/or --system')
        change(a, targets)
    elif a.command == 'read':
        read(a, a.indices or None)
    elif a.command == 'sdr':
        fans(a)
    else:
        if not math.isfinite(a.interval) or a.interval <= 0:
            p.error('interval must be finite and positive')
        while True:
            print(time.strftime('%Y-%m-%d %H:%M:%S'))
            read(a, [SYSTEM, CPU, 12])
            fans(a)
            time.sleep(a.interval)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit('Interrupted. Existing overrides remain active; use clear explicitly.')
    except (ValueError, RuntimeError, OSError, subprocess.TimeoutExpired) as exc:
        raise SystemExit(str(exc))
