# Experimental paper bundle

This folder is an Overleaf-ready experimental-results bundle generated on
2026-07-26.

## Files

- `main.tex`: compileable integration draft.
- `sections/experimental_results.tex`: drop-in replacement for the
  `Physical Experimental Results` section in the original manuscript.
- `figures/experimental/`: all selected nominal and disturbance figures in
  both PDF and PNG form, grouped by experiment.
- `data/`: source metric tables used in the experimental section.

## Important interpretation limits

- Circle nominal: five selected laps per controller.
- Figure-eight: best available pilot per controller.
- Square: BSMC and Backstepping final historical logs, but SMC is a tuning
  pilot and must not be described as validation.
- Circle disturbance: manually applied pilot disturbances; yaw-90 SMC is
  safety-censored.
- Do not report mean plus/minus standard deviation until equal-repeat,
  locked-gain validation runs are collected.

## Integration into the supplied manuscript

In the original LaTeX file:

1. Add `\newcommand{\ExpDir}{figures/experimental}` after `\FigDir`.
2. Replace the complete section beginning with
   `\section{Physical Experimental Results}` and ending before
   `\section{Conclusion and Future Work}` with:

   ```tex
   \input{sections/experimental_results}
   ```

3. Upload this directory structure without flattening it.

The simulation assets referenced by the supplied manuscript
(`results/figures/*.pdf`, `fig1_arch.png`, and `kinematics.png`) were not
present in the local workspace and therefore are not fabricated in this
bundle.
