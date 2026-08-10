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




## Summary

<!-- What does this PR do, in a sentence or two? -->

## Related Issues

<!-- e.g. Closes #123, Related to #45 -->

## Checklist

Check off what's relevant to this PR. Leave irrelevant items unchecked.

- [ ] Successfully builds with no errors
- [ ] Tested on hardware
- [ ] Cleaned up debug-only code (stray logging statements, hardcoded test values)
- [ ] Changes to function behavior or prototype are documented via Doxygen
- [ ] Structural or API changes are documented
- [ ] If changes are made to the build system, that it builds for both Windows and Ubuntu/Debian

## Notes for Reviewers

<!-- Anything a reviewer should specifically look at, test, or be aware of? -->
