#!/usr/bin/env python3
"""Discover ESP32 nodes sending UDP feedback and OTA flash them.

Default behavior:
- optionally listen on UDP port 6666 to discover active device IPs
- let the user choose which devices to flash
- build firmware and LittleFS images once
- run espota.py for firmware, then LittleFS, for each selected device

Example:
  python scripts/ota_flash.py --discover --listen-seconds 5
  python scripts/ota_flash.py --ips 10.106.28.95 --yes
"""

from __future__ import annotations

import argparse
import shlex
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Sequence

ROOT = Path(__file__).resolve().parents[1]
PIO = Path.home() / '.platformio/penv/bin/pio'
PIO_PYTHON = Path.home() / '.platformio/penv/bin/python'
ESPOTA = Path.home() / '.platformio/packages/framework-arduinoespressif32/tools/espota.py'


class CommandError(RuntimeError):
    def __init__(self, message: str, *, output: str = '', returncode: int | None = None):
        super().__init__(message)
        self.output = output
        self.returncode = returncode


def run(cmd: Sequence[str], *, cwd: Path) -> str:
    print(f"\n$ {' '.join(shlex.quote(part) for part in cmd)}")
    completed = subprocess.run(
        list(cmd),
        cwd=str(cwd),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    output = completed.stdout or ''
    if output:
        print(output, end='' if output.endswith('\n') else '\n')
    if completed.returncode != 0:
        raise CommandError(
            f"Command failed with exit code {completed.returncode}: {' '.join(cmd)}",
            output=output,
            returncode=completed.returncode,
        )
    return output


def ensure_tools_exist() -> None:
    missing = [str(path) for path in (PIO, PIO_PYTHON, ESPOTA) if not path.exists()]
    if missing:
        raise FileNotFoundError(f"Missing required tool(s): {', '.join(missing)}")


def build_images(env: str, build_fs: bool) -> tuple[Path, Path | None]:
    run([str(PIO), 'run', '-e', env], cwd=ROOT)
    firmware = ROOT / '.pio' / 'build' / env / 'firmware.bin'
    if not firmware.exists():
        raise FileNotFoundError(f"Firmware image not found: {firmware}")

    littlefs = None
    if build_fs:
        run([str(PIO), 'run', '-e', env, '-t', 'buildfs'], cwd=ROOT)
        littlefs = ROOT / '.pio' / 'build' / env / 'littlefs.bin'
        if not littlefs.exists():
            raise FileNotFoundError(f"LittleFS image not found: {littlefs}")

    return firmware, littlefs


def discover_devices(port: int, listen_seconds: float) -> Dict[str, dict]:
    devices: Dict[str, dict] = {}
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('', port))
    except OSError as exc:
        raise RuntimeError(
            f"Unable to bind UDP {port} for discovery: {exc}. "
            "Use --ips to specify devices manually if another process already owns the port."
        ) from exc

    sock.settimeout(0.5)
    deadline = time.time() + listen_seconds
    print(f"Listening on UDP {port} for {listen_seconds:.1f}s...")
    with sock:
        while time.time() < deadline:
            try:
                payload, (ip, src_port) = sock.recvfrom(65535)
            except socket.timeout:
                continue
            info = devices.setdefault(ip, {'count': 0, 'bytes': 0, 'last_src_port': src_port, 'last_seen': 0.0})
            info['count'] += 1
            info['bytes'] += len(payload)
            info['last_src_port'] = src_port
            info['last_seen'] = time.time()
    return devices


def select_ips(discovered: Dict[str, dict], explicit_ips: List[str], yes: bool) -> List[str]:
    ips: List[str] = []
    seen = set()
    for ip in explicit_ips:
        if ip not in seen:
            ips.append(ip)
            seen.add(ip)
    for ip in discovered:
        if ip not in seen:
            ips.append(ip)
            seen.add(ip)

    if not ips:
        return []
    if yes:
        return ips

    print('\nDiscovered candidate devices:')
    for index, ip in enumerate(ips, start=1):
        info = discovered.get(ip)
        if info:
            print(f"  {index}. {ip}  packets={info['count']} bytes={info['bytes']} src_port={info['last_src_port']}")
        else:
            print(f"  {index}. {ip}  (manual)")

    while True:
        choice = input("\nFlash which devices? [all/none/1,3,...]: ").strip().lower()
        if choice in {'', 'all', 'a'}:
            return ips
        if choice in {'none', 'n'}:
            return []
        try:
            selected = []
            for token in choice.split(','):
                idx = int(token.strip())
                if idx < 1 or idx > len(ips):
                    raise ValueError
                selected.append(ips[idx - 1])
            # preserve order, de-dupe
            ordered = []
            for ip in selected:
                if ip not in ordered:
                    ordered.append(ip)
            return ordered
        except ValueError:
            print('Enter all, none, or a comma-separated list like 1,3')


def infer_host_ip(target_ip: str) -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((target_ip, 1))
        host_ip = sock.getsockname()[0]
    finally:
        sock.close()
    if not host_ip or host_ip == '0.0.0.0':
        raise RuntimeError(f'Could not infer a usable host IP for target {target_ip}')
    return host_ip


def wait_for_host_port(preferred_port: int, wait_seconds: float = 90.0, poll_interval: float = 0.5) -> int:
    deadline = time.time() + wait_seconds
    while time.time() < deadline:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('', preferred_port))
            return preferred_port
        except OSError:
            time.sleep(poll_interval)
        finally:
            sock.close()
    raise RuntimeError(f'Host callback port {preferred_port} did not become free within {wait_seconds:.1f}s')



def run_ota_with_retries(
    *,
    target_ip: str,
    host_ip: str,
    ota_port: int,
    preferred_host_port: int,
    image: Path,
    timeout: int,
    spiffs: bool,
    debug: bool,
    retries: int = 4,
    retry_delay: float = 5.0,
) -> None:
    last_error: CommandError | None = None
    retryable_markers = (
        'Address already in use',
        'No response from device',
        'Broken pipe',
        'timed out',
    )

    for attempt in range(1, retries + 1):
        host_port = wait_for_host_port(preferred_host_port)
        phase = 'data' if spiffs else 'firmware'
        print(f"Using host callback port {host_port} for {phase} OTA (attempt {attempt}/{retries})")
        cmd = ota_cmd(
            target_ip=target_ip,
            host_ip=host_ip,
            ota_port=ota_port,
            host_port=host_port,
            image=image,
            timeout=timeout,
            spiffs=spiffs,
            debug=debug,
        )
        try:
            run(cmd, cwd=ROOT)
            if attempt > 1:
                print(f"Succeeded on retry {attempt}.")
            return
        except CommandError as exc:
            last_error = exc
            output = exc.output or str(exc)
            if not any(marker in output for marker in retryable_markers) or attempt >= retries:
                raise
            print(f"Retrying in {retry_delay:.1f}s after OTA transport failure...")
            time.sleep(retry_delay)

    if last_error is not None:
        raise last_error


def ota_cmd(
    *,
    target_ip: str,
    host_ip: str,
    ota_port: int,
    host_port: int,
    image: Path,
    timeout: int,
    spiffs: bool,
    debug: bool,
) -> List[str]:
    cmd = [
        str(PIO_PYTHON),
        str(ESPOTA),
        '-i',
        target_ip,
        '-I',
        host_ip,
        '-p',
        str(ota_port),
        '-P',
        str(host_port),
    ]
    if spiffs:
        cmd.append('-s')
    cmd.extend([
        '-f',
        str(image),
        f'--timeout={timeout}',
    ])
    if debug:
        cmd.append('-d')
    return cmd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--env', default='jw_esp32_ota_lab', help='PlatformIO environment to build (default: %(default)s)')
    parser.add_argument('--ips', nargs='*', default=[], help='Explicit target IPs to flash')
    parser.add_argument('--discover', action='store_true', help='Listen on UDP 6666 and discover active devices before flashing')
    parser.add_argument('--discover-only', action='store_true', help='Only discover devices and print them')
    parser.add_argument('--listen-port', type=int, default=6666, help='UDP port to listen on for discovery (default: %(default)s)')
    parser.add_argument('--listen-seconds', type=float, default=5.0, help='How long to listen during discovery (default: %(default)s)')
    parser.add_argument('--host-port', type=int, default=3232, help='Fixed host callback port for espota (default: %(default)s)')
    parser.add_argument('--ota-port', type=int, default=3232, help='OTA port on the ESP32 (default: %(default)s)')
    parser.add_argument('--timeout', type=int, default=30, help='espota host-side timeout in seconds (default: %(default)s)')
    parser.add_argument('--yes', action='store_true', help='Skip selection prompt and flash all discovered/manual IPs')
    parser.add_argument('--no-build', action='store_true', help='Reuse existing .pio build artifacts instead of rebuilding')
    parser.add_argument('--skip-firmware', action='store_true', help='Skip firmware OTA and only push LittleFS')
    parser.add_argument('--skip-data', action='store_true', help='Skip LittleFS OTA and only push firmware')
    parser.add_argument('--debug', action='store_true', help='Pass -d to espota.py for extra logging')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    ensure_tools_exist()

    if args.skip_firmware and args.skip_data:
        print('Nothing to do: both firmware and data uploads are disabled.')
        return 1

    discovered: Dict[str, dict] = {}
    if args.discover or args.discover_only:
        discovered = discover_devices(args.listen_port, args.listen_seconds)
        if discovered:
            print('\nDiscovered devices:')
            for ip, info in discovered.items():
                print(f"  {ip}  packets={info['count']} bytes={info['bytes']} src_port={info['last_src_port']}")
        else:
            print('\nNo devices seen during discovery window.')
        if args.discover_only:
            return 0

    targets = select_ips(discovered, args.ips, args.yes)
    if not targets:
        print('No target devices selected.')
        return 0

    firmware_image: Path | None = None
    littlefs_image: Path | None = None
    if args.no_build:
        build_dir = ROOT / '.pio' / 'build' / args.env
        firmware_image = build_dir / 'firmware.bin'
        littlefs_image = build_dir / 'littlefs.bin'
    else:
        firmware_image, littlefs_image = build_images(args.env, build_fs=not args.skip_data)

    if not args.skip_firmware and (firmware_image is None or not firmware_image.exists()):
        raise FileNotFoundError('Firmware image is missing. Build first or remove --no-build.')
    if not args.skip_data and (littlefs_image is None or not littlefs_image.exists()):
        raise FileNotFoundError('LittleFS image is missing. Build first or remove --no-build.')

    for ip in targets:
        host_ip = infer_host_ip(ip)
        print(f"\n=== Target {ip} (host callback IP: {host_ip}) ===")
        if not args.skip_firmware:
            run_ota_with_retries(
                target_ip=ip,
                host_ip=host_ip,
                ota_port=args.ota_port,
                preferred_host_port=args.host_port,
                image=firmware_image,
                timeout=args.timeout,
                spiffs=False,
                debug=args.debug,
            )
            time.sleep(5.0)
        if not args.skip_data:
            run_ota_with_retries(
                target_ip=ip,
                host_ip=host_ip,
                ota_port=args.ota_port,
                preferred_host_port=args.host_port,
                image=littlefs_image,
                timeout=args.timeout,
                spiffs=True,
                debug=args.debug,
            )
            time.sleep(5.0)

    print('\nAll requested OTA updates completed successfully.')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('\nInterrupted by user.')
        raise SystemExit(130)
