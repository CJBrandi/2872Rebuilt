#!/usr/bin/env python3
"""Phoenix-style persistent TalonFX tuning CLI."""

from __future__ import annotations

import argparse
import json
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

DS_SEND_PORT = 1110
SEND_INTERVAL = 0.02
DEFAULT_HTTP_PORT = 5805


class Mode:
    TELEOP = 0x00
    TEST = 0x01
    AUTO = 0x02


class Request:
    NORMAL = 0x00


def control_packet(num: int, mode: int, enabled: bool, request: int = Request.NORMAL, alliance: int = 0) -> bytes:
    return struct.pack(">HBBBB", num & 0xFFFF, 0x01, mode | (0x04 if enabled else 0), request, alliance)


def date_tag() -> bytes:
    now = datetime.now(timezone.utc)
    data = struct.pack(">IBBBBB", now.microsecond, now.second, now.minute, now.hour, now.day, now.month - 1)
    data += struct.pack("B", now.year - 1900)
    return bytes([len(data) + 1, 0x0F]) + data


def timezone_tag() -> bytes:
    tz = time.tzname[0].encode("utf-8")
    return bytes([len(tz) + 1, 0x10]) + tz


def empty_joystick_tag() -> bytes:
    return bytes([4, 0x0C, 0, 0, 0, 0])


def robot_host(team: int) -> str:
    return f"roboRIO-{team}-FRC.local"


def http_json(host: str, port: int, method: str, path: str, body: dict[str, Any] | None = None, timeout: float = 5.0) -> dict[str, Any]:
    url = f"http://{host}:{port}{path}"
    data = None if body is None else json.dumps(body).encode("utf-8")
    request = urllib.request.Request(url, data=data, method=method)
    request.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


@dataclass
class DriverStationEnable:
    host: str
    enabled: bool = False
    mode: int = Mode.TEST

    def __post_init__(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._stop = threading.Event()
        self._packet_num = 0
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self.enabled = True
        self._thread = threading.Thread(target=self._loop, name="talonfx-tuner-ds", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.enabled = False
        self._stop.set()
        for _ in range(8):
            self._send(False)
            time.sleep(0.02)
        if self._thread:
            self._thread.join(timeout=0.5)
        self._sock.close()

    def _loop(self) -> None:
        while not self._stop.is_set():
            self._send(self.enabled)
            time.sleep(SEND_INTERVAL)

    def _send(self, enabled: bool) -> None:
        packet = control_packet(self._packet_num, self.mode, enabled)
        packet += date_tag() + timezone_tag()
        packet += empty_joystick_tag()
        try:
            self._sock.sendto(packet, (self.host, DS_SEND_PORT))
        except OSError:
            pass
        self._packet_num = (self._packet_num + 1) & 0xFFFF


def skill_dir() -> Path:
    return Path(__file__).resolve().parents[1]


def repo_root_from_args(args: argparse.Namespace) -> Path:
    return Path(args.repo).expanduser().resolve()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(value, f, indent=2, sort_keys=True)
        f.write("\n")


def parse_followers(raw: str | None) -> list[dict[str, Any]]:
    if not raw:
        return []
    followers: list[dict[str, Any]] = []
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        if ":" in item:
            id_text, alignment = item.split(":", 1)
        else:
            id_text, alignment = item, "Aligned"
        followers.append({"id": int(id_text), "alignment": alignment.strip() or "Aligned"})
    return followers


def command_deploy_server(args: argparse.Namespace) -> int:
    repo = repo_root_from_args(args)
    gradlew = repo / "gradlew"
    if not gradlew.exists():
        print(f"Missing Gradle wrapper at {gradlew}", file=sys.stderr)
        return 2

    asset = skill_dir() / "assets" / "talonfx-tuner-server"
    work_dir = Path(args.work_dir).expanduser().resolve()
    if work_dir.exists():
        shutil.rmtree(work_dir)
    shutil.copytree(asset, work_dir)
    repo_vendordeps = repo / "vendordeps"
    if repo_vendordeps.exists():
        shutil.copytree(repo_vendordeps, work_dir / "vendordeps")

    config = {
        "leaderId": args.leader_id,
        "canBus": args.can_bus,
        "followers": parse_followers(args.followers),
        "leaderInverted": args.leader_inverted,
        "neutralMode": args.neutral_mode,
        "httpPort": args.port,
    }
    if args.sensor_to_mechanism_ratio is not None:
        config["sensorToMechanismRatio"] = args.sensor_to_mechanism_ratio
    if args.rotor_to_sensor_ratio is not None:
        config["rotorToSensorRatio"] = args.rotor_to_sensor_ratio
    write_json(work_dir / "src/main/deploy/talonfx-tuner-config.json", config)

    print(f"Deploying persistent TalonFX tuner server from {work_dir}")
    print("This replaces normal robot code until you redeploy the normal project.")
    cmd = [str(gradlew), "-p", str(work_dir), "deploy", f"-PteamNumber={args.team}"]
    subprocess.run(cmd, cwd=repo, check=True)
    return command_wait_ready(args)


def command_wait_ready(args: argparse.Namespace) -> int:
    host = args.host or robot_host(args.team)
    deadline = time.monotonic() + args.timeout
    last_error = ""
    while time.monotonic() < deadline:
        try:
            health = http_json(host, args.port, "GET", "/health", timeout=2.0)
            if health.get("ready") and health.get("statusOk", True):
                print("TalonFX tuner server is ready:")
                print(json.dumps(health, indent=2, sort_keys=True))
                return 0
            last_error = json.dumps(health)
        except (OSError, urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            last_error = str(exc)
        time.sleep(0.5)
    print(f"Tuner server was not ready before timeout. Last error: {last_error}", file=sys.stderr)
    return 1


def command_run_test(args: argparse.Namespace) -> int:
    host = args.host or robot_host(args.team)
    plan = load_json(Path(args.plan).expanduser().resolve())
    duration = float(plan.get("durationSec", 10))
    print("About to run this TalonFX test plan:")
    print(json.dumps(plan, indent=2, sort_keys=True))
    confirmation = input("Type yes to enable and run this bounded test: ").strip()
    if confirmation != "yes":
        print("Aborted before enable.")
        return 1

    ds = DriverStationEnable(host)
    result: dict[str, Any] | None = None
    try:
        ds.start()
        time.sleep(args.enable_settle_sec)
        result = http_json(host, args.port, "POST", "/run", plan, timeout=duration + args.http_margin_sec)
        return_code = 0 if result.get("ok") else 3
    except KeyboardInterrupt:
        print("Interrupted; disabling robot and stopping tuner run.", file=sys.stderr)
        return_code = 130
    except Exception as exc:
        print(f"Run failed: {exc}", file=sys.stderr)
        return_code = 2
    finally:
        ds.stop()
        try:
            http_json(host, args.port, "POST", "/stop", {}, timeout=2.0)
        except Exception:
            pass

    if result is not None:
        print(json.dumps(result, indent=2, sort_keys=True))
        if args.output:
            write_json(Path(args.output).expanduser().resolve(), result)
    return return_code


def command_stop(args: argparse.Namespace) -> int:
    host = args.host or robot_host(args.team)
    ds = DriverStationEnable(host)
    ds.stop()
    try:
        result = http_json(host, args.port, "POST", "/stop", {}, timeout=2.0)
        print(json.dumps(result, indent=2, sort_keys=True))
    except Exception as exc:
        print(f"Sent DS disable; HTTP stop failed: {exc}", file=sys.stderr)
        return 1
    return 0


def command_self_test(_: argparse.Namespace) -> int:
    packet = control_packet(7, Mode.TEST, True)
    expected = struct.pack(">HBBBB", 7, 0x01, 0x05, 0x00, 0x00)
    assert packet == expected, packet.hex()
    assert empty_joystick_tag() == bytes([4, 0x0C, 0, 0, 0, 0])
    assert date_tag()[1] == 0x0F
    assert timezone_tag()[1] == 0x10
    print("self-test passed")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.set_defaults(func=lambda args: parser.print_help() or 2)
    parser.add_argument("--repo", default="/Users/cjbrandi/2872Rebuilt", help="Robot repo containing gradlew.")
    parser.add_argument("--team", type=int, default=2872)
    parser.add_argument("--host", help="Robot hostname/IP. Defaults to roboRIO-TEAM-FRC.local.")
    parser.add_argument("--port", type=int, default=DEFAULT_HTTP_PORT)

    sub = parser.add_subparsers(dest="command")

    deploy = sub.add_parser("deploy-server", help="Build and deploy the persistent tuner robot once.")
    deploy.add_argument("--leader-id", type=int, required=True)
    deploy.add_argument("--can-bus", default="rio", help='CAN bus name, e.g. "rio", "*", or CANivore name.')
    deploy.add_argument("--followers", help="Comma list like '2:Aligned,3:Opposed'.")
    deploy.add_argument("--leader-inverted", default="CounterClockwise_Positive", choices=["CounterClockwise_Positive", "Clockwise_Positive"])
    deploy.add_argument("--neutral-mode", default="Brake", choices=["Brake", "Coast"])
    deploy.add_argument("--sensor-to-mechanism-ratio", type=float)
    deploy.add_argument("--rotor-to-sensor-ratio", type=float)
    deploy.add_argument("--work-dir", default="/tmp/talonfx-tuner-server")
    deploy.add_argument("--timeout", type=float, default=45)
    deploy.set_defaults(func=command_deploy_server)

    ready = sub.add_parser("wait-ready", help="Poll /health until the server is ready.")
    ready.add_argument("--timeout", type=float, default=30)
    ready.set_defaults(func=command_wait_ready)

    run = sub.add_parser("run-test", help="Enable, run one confirmed test plan, disable, and print telemetry.")
    run.add_argument("--plan", required=True, help="JSON test plan path.")
    run.add_argument("--output", help="Optional telemetry output JSON path.")
    run.add_argument("--enable-settle-sec", type=float, default=0.5)
    run.add_argument("--http-margin-sec", type=float, default=8.0)
    run.set_defaults(func=command_run_test)

    stop = sub.add_parser("stop", help="Send DS disable and neutral command.")
    stop.set_defaults(func=command_stop)

    self_test = sub.add_parser("self-test", help="Run local packet tests.")
    self_test.set_defaults(func=command_self_test)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return int(args.func(args) or 0)


if __name__ == "__main__":
    raise SystemExit(main())
