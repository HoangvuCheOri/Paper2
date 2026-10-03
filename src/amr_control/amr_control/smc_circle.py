#!/usr/bin/env python3
"""Pure-SMC Circle baseline with embedded paper capture."""

from amr_control.bsmc_circle import BSMCCircle
from amr_control.controller_modes import force_smc
from amr_control.controller_paper_runtime import run_controller


def main(args=None):
    run_controller(
        BSMCCircle,
        "SMC",
        "circle",
        configure=lambda node: force_smc(node, "circle"),
        args=args,
    )


if __name__ == "__main__":
    main()

