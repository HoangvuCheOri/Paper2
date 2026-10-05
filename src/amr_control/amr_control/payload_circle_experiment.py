#!/usr/bin/env python3
"""Interactive runner for the seven-case circle payload experiment."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from datetime import datetime
from pathlib import Path


BASE_MASS_KG = 1.55
PAYLOADS_KG = (0.5, 1.0, 1.5)
OFFSET_M = 0.05
CONTROLLERS = {
    "backstepping": "backstepping_circle",
    "bsmc": "bsmc_circle",
    "smc": "smc_circle",
}


def experiment_cases():
    cases = [{
        "case_id": "nominal",
        "placement": "none",
        "payload_mass_kg": 0.0,
        "payload_offset_m": 0.0,
    }]
    for mass in PAYLOADS_KG:
        tag = f"m{mass:.1f}".replace(".", "p")
        cases.append({
            "case_id": f"{tag}_center",
            "placement": "center",
            "payload_mass_kg": mass,
            "payload_offset_m": 0.0,
        })
    for mass in PAYLOADS_KG:
        tag = f"m{mass:.1f}".replace(".", "p")
        cases.append({
            "case_id": f"{tag}_offset_5cm",
            "placement": "offset",
            "payload_mass_kg": mass,
            "payload_offset_m": OFFSET_M,
        })
    return cases


def payload_jz_com(mass, length_m, width_m, explicit_jz):
    """Payload centroidal yaw inertia; rectangular plate if dimensions exist."""
    if explicit_jz is not None:
        return float(explicit_jz), "provided"
    if length_m is not None and width_m is not None:
        return mass * (length_m * length_m + width_m * width_m) / 12.0, "rectangle"
    return 0.0, "point_mass_approximation"


def condition_metadata(case, args):
    payload_jz, model = payload_jz_com(
        case["payload_mass_kg"],
        args.payload_length_m,
        args.payload_width_m,
        args.payload_jz_com,
    )
    delta_jz = payload_jz + (
        case["payload_mass_kg"] * case["payload_offset_m"] ** 2
    )
    total_jz = (
        args.base_jz + delta_jz if args.base_jz is not None else None
    )
    direction_vectors = {
        "forward": (case["payload_offset_m"], 0.0),
        "rear": (-case["payload_offset_m"], 0.0),
        "left": (0.0, case["payload_offset_m"]),
        "right": (0.0, -case["payload_offset_m"]),
    }
    offset_x, offset_y = direction_vectors[args.offset_direction]
    direction = args.offset_direction if case["payload_offset_m"] else "none"
    return {
        **case,
        "payload_offset_direction": direction,
        "payload_offset_x_m": offset_x,
        "payload_offset_y_m": offset_y,
        "base_mass_kg": args.base_mass,
        "total_mass_kg": args.base_mass + case["payload_mass_kg"],
        "base_jz_kg_m2": args.base_jz,
        "payload_jz_com_kg_m2": payload_jz,
        "payload_jz_model": model,
        "payload_delta_jz_kg_m2": delta_jz,
        "total_jz_kg_m2": total_jz,
    }


def _select(values, requested, all_token="all"):
    return list(values) if requested == all_token else [requested]


def _format_number(value):
    return "unknown" if value is None else f"{value:.6g}"


def print_plan(cases, controllers, args):
    print("case                 controller      m_total(kg)  d(m)   delta_Jz(kg.m^2)  Jz_total")
    for case in cases:
        data = condition_metadata(case, args)
        for controller in controllers:
            print(
                f"{case['case_id']:<20} {controller:<15} "
                f"{data['total_mass_kg']:<12.3f} {case['payload_offset_m']:<6.3f} "
                f"{data['payload_delta_jz_kg_m2']:<18.6f} "
                f"{_format_number(data['total_jz_kg_m2'])}"
            )


def controller_command(controller, metadata, output_dir, run_id, args):
    base_jz = metadata["base_jz_kg_m2"]
    command = [
        "ros2", "run", "amr_control", CONTROLLERS[controller], "--ros-args",
        "-p", f"angular_speed:={args.angular_speed}",
        "-p", f"paper_laps:={args.laps}",
        "-p", f"paper_output_dir:={output_dir}",
        "-p", f"paper_run_id:={run_id}",
        "-p", f"experiment_case_id:={metadata['case_id']}",
        "-p", f"load_placement:={metadata['placement']}",
        "-p", f"base_mass_kg:={metadata['base_mass_kg']}",
        "-p", f"payload_mass_kg:={metadata['payload_mass_kg']}",
        "-p", f"payload_offset_m:={metadata['payload_offset_m']}",
        "-p", f"payload_offset_x_m:={metadata['payload_offset_x_m']}",
        "-p", f"payload_offset_y_m:={metadata['payload_offset_y_m']}",
        "-p", f"payload_offset_direction:={metadata['payload_offset_direction']}",
        "-p", f"base_jz_kg_m2:={base_jz if base_jz is not None else -1.0}",
        "-p", f"payload_jz_com_kg_m2:={metadata['payload_jz_com_kg_m2']}",
        "-p", f"payload_jz_model:={metadata['payload_jz_model']}",
    ]
    if controller in {"bsmc", "smc"}:
        command += ["-p", f"ks1:={args.ks1}", "-p", f"ks2:={args.ks2}"]
    return command


def append_manifest(path, session_id, record):
    if path.exists():
        document = json.loads(path.read_text(encoding="utf-8"))
    else:
        document = {"session_id": session_id, "runs": []}
    document["runs"].append(record)
    path.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")


def parse_args(argv=None):
    case_ids = [case["case_id"] for case in experiment_cases()]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=case_ids + ["all"], default="all")
    parser.add_argument(
        "--controller", choices=list(CONTROLLERS) + ["all"], default="all"
    )
    parser.add_argument("--execute", action="store_true", help="run robot; otherwise only print the plan")
    parser.add_argument("--yes", action="store_true", help="skip the per-run Enter prompt")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--laps", type=float, default=3.0)
    parser.add_argument("--angular-speed", type=float, default=0.108)
    parser.add_argument("--base-mass", type=float, default=BASE_MASS_KG)
    parser.add_argument("--base-jz", type=float, default=None)
    parser.add_argument("--payload-jz-com", type=float, default=None)
    parser.add_argument("--payload-length-m", type=float, default=None)
    parser.add_argument("--payload-width-m", type=float, default=None)
    parser.add_argument(
        "--offset-direction",
        choices=("forward", "rear", "left", "right"),
        default="forward",
        help="robot-frame direction of the 5 cm offset (x forward, y left)",
    )
    parser.add_argument("--ks1", type=float, default=0.024)
    parser.add_argument("--ks2", type=float, default=0.050)
    parser.add_argument("--session-id", default=datetime.now().strftime("payload_circle_%Y%m%d"))
    parser.add_argument("--output-root", default="paper_runs/payload_circle")
    args = parser.parse_args(argv)
    if args.repeat < 1 or args.laps <= 0 or args.base_mass <= 0:
        parser.error("repeat, laps, and base-mass must be positive")
    if args.base_jz is not None and args.base_jz < 0:
        parser.error("base-jz must be non-negative")
    if args.payload_jz_com is not None and args.payload_jz_com < 0:
        parser.error("payload-jz-com must be non-negative")
    dims = (args.payload_length_m, args.payload_width_m)
    if (dims[0] is None) != (dims[1] is None) or any(
        value is not None and value <= 0 for value in dims
    ):
        parser.error("provide both positive payload-length-m and payload-width-m")
    return args


def main(argv=None):
    args = parse_args(argv)
    all_cases = experiment_cases()
    chosen_cases = (
        all_cases if args.case == "all"
        else [case for case in all_cases if case["case_id"] == args.case]
    )
    controllers = _select(CONTROLLERS, args.controller)
    print_plan(chosen_cases, controllers, args)
    if not args.execute:
        print("\nDry run only. Add --execute when the robot and four background nodes are ready.")
        return 0

    session_dir = Path(args.output_root).expanduser().resolve() / args.session_id
    session_dir.mkdir(parents=True, exist_ok=True)
    manifest = session_dir / "payload_experiment_manifest.json"
    for repeat in range(1, args.repeat + 1):
        run_controllers = controllers
        if args.controller == "all":
            shift = (repeat - 1) % len(controllers)
            run_controllers = controllers[shift:] + controllers[:shift]
        for case in chosen_cases:
            metadata = condition_metadata(case, args)
            output_dir = session_dir / case["case_id"]
            output_dir.mkdir(parents=True, exist_ok=True)
            for controller in run_controllers:
                run_id = f"{args.session_id}_{case['case_id']}_{controller}_r{repeat:02d}"
                prompt = (
                    f"\nREADY {run_id}: payload={case['payload_mass_kg']:.1f} kg, "
                    f"placement={case['placement']}, d={case['payload_offset_m']:.2f} m "
                    f"toward {metadata['payload_offset_direction']}. "
                    "Put robot at the marked start pose, verify camera/EKF, then press Enter. "
                )
                if not args.yes:
                    input(prompt)
                command = controller_command(
                    controller, metadata, output_dir, run_id, args
                )
                started = datetime.now().astimezone().isoformat()
                result = subprocess.run(command, check=False)
                record = {
                    "run_id": run_id,
                    "controller": controller,
                    "repeat": repeat,
                    "started_at": started,
                    "finished_at": datetime.now().astimezone().isoformat(),
                    "return_code": result.returncode,
                    "condition": metadata,
                    "command": command,
                }
                append_manifest(manifest, args.session_id, record)
                if result.returncode != 0:
                    raise SystemExit(
                        f"Run failed with code {result.returncode}; stopped. See {manifest}"
                    )
    print(f"Completed. Manifest: {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
