#!/usr/bin/env python3
"""Pure-SMC Square baseline with embedded paper capture."""

from amr_control.bsmc_square import BSMCSquare
from amr_control.controller_modes import force_smc
from amr_control.controller_paper_runtime import run_controller


def configure_smc_square(node):
    """Apply pure-SMC mode and Square-only sliding-surface parameters."""
    force_smc(node, "square")
    node.declare_parameter("sliding_c", 1.0)
    node.c = max(1e-6, float(node.get_parameter("sliding_c").value))
    node.phi1 = max(1e-6, float(node.phi1))
    node.phi2 = max(1e-6, float(node.phi2))
    node.get_logger().info(
        f"SMC Square sliding surface: c={node.c:.6g}, "
        f"phi1={node.phi1:.6g}, phi2={node.phi2:.6g}."
    )


def main(args=None):
    run_controller(
        BSMCSquare,
        "SMC",
        "square",
        configure=configure_smc_square,
        args=args,
    )


if __name__ == "__main__":
    main()
