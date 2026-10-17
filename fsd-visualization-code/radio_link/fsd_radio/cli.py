import argparse
import json
import logging
from pathlib import Path
import uuid
from .configuration import load_config


def main():
    parser = argparse.ArgumentParser(
        description="FSD low-bandwidth bidirectional E22 / Foxglove link"
    )
    parser.add_argument(
        "--config", default=str(Path(__file__).resolve().parents[1] / "radio.yaml")
    )
    parser.add_argument("--verbose", action="store_true")
    sub = parser.add_subparsers(dest="action", required=True)
    for name in ("linux", "pc", "demo", "inspect"):
        command = sub.add_parser(name)
        command.add_argument("--mode", choices=("native", "bridge"))
        command.add_argument("--port")
        if name == "linux":
            command.add_argument("--use-sim-time", action="store_true")
        if name == "demo":
            command.add_argument("--duration", type=float)
    send = sub.add_parser("send")
    send.add_argument(
        "--message",
        default=None,
        help="JSON object; default sends an identifiable ping",
    )
    send.add_argument("--control-port", type=int, default=8766)
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    try:
        if args.action == "send":
            from .pc_receiver import send_extension

            body = (
                json.loads(args.message)
                if args.message
                else {"type": "ping", "id": uuid.uuid4().hex[:12]}
            )
            reply = send_extension("127.0.0.1", args.control_port, body)
            print(json.dumps(reply, ensure_ascii=False, indent=2))
            if not reply.get("queued"):
                parser.exit(1)
            return
        config = load_config(args.config, args.mode)
        if args.action == "inspect":
            print(
                json.dumps(
                    {
                        "mode": config["display_mode"],
                        "fixed_frame": config["fixed_frame"],
                        **config["inspection"],
                        "link": config["link"],
                    },
                    ensure_ascii=False,
                    indent=2,
                )
            )
        elif args.action == "linux":
            port = args.port or config["serial"]["linux_port"]
            if not port:
                parser.error(
                    "Linux radio port is required: linux --port /dev/serial/by-id/..."
                )
            from .ros_sender import run_linux

            run_linux(config, port, args.use_sim_time)
        else:
            from .pc_receiver import run_pc

            run_pc(
                config,
                args.port or config["serial"]["pc_port"],
                demo=args.action == "demo",
                duration=getattr(args, "duration", None),
            )
    except (ValueError, FileNotFoundError, ImportError, OSError) as exc:
        parser.exit(1, f"fsd-radio: {exc}\n")


if __name__ == "__main__":
    main()
