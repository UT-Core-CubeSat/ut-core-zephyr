## Summary

<!-- What does this PR do, and why? -->

Closes #

## Subsystem(s) affected

<!-- cdh / gnss / eps / solar / comms / adcs / build-system / docs -->

## Checklist

- [ ] Builds clean via `west build` (or app Makefile) with no new warnings
- [ ] Tested on hardware, or in Renode / `ut_core_sim` if hardware unavailable
- [ ] No debug-only code left in (stray `printk` spam, hardcoded test
      values, commented-out blocks) unless it belongs in `apps/tests/`
- [ ] Linked to an issue (`Closes #...` above)
- [ ] Devicetree/Kconfig changes are explained below, if any
- [ ] Relevant subsystem owner tagged for review

## Devicetree / Kconfig changes

<!-- If none, write "N/A". If present, explain why — don't make the
     reviewer reverse-engineer an overlay diff. -->

## Testing notes

<!-- What did you actually run, on what hardware/sim target, and what did
     you observe? -->
