#!/usr/bin/env python3
"""Pure-SMC Figure-eight baseline with embedded paper capture."""

from amr_control.bsmc_eight import BSMCEight
from amr_control.controller_modes import force_smc
from amr_control.controller_paper_runtime import run_controller


def main(args=None):
    run_controller(
        BSMCEight,
        "SMC",
        "eight",
        configure=lambda node: force_smc(node, "eight"),
        args=args,
    )


if __name__ == "__main__":
    main()

