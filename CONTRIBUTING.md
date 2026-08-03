# Contributing to ut-core-zephyr

This document details the requirements and conventions neccessary to keep the project usable and easy to maintain for future contributures. For onboarding, please see the [Onboarding]() page on the [UT Core Cubesat Website](https://ut-core-cubesat.github.io/ut-core-docs/)

---

## 2. Use GitHub Issues.

All work, bugs, features, hardware, and tasks should go through GitHub Issues.

- **Before starting work**, check for an existing issue. If none exists, open one describing the problem or task before writing code.
- **No PR without a linked issue.** Reference it in the PR description
  (`Closes #123`).
- **Label issues** by subsystem (`cdh`, `gnss`, `eps`, `solar`, `comms`,
  `adcs`, `build-system`, `docs`) and type (`bug`, `feature`, `handoff`).
- If you find a problem while working on something else, open a new issue
  for it rather than silently fixing it in an unrelated PR — makes history
  easier to follow later.

---

## 3. Branching and Commits

- Branch per issue: `issue-123-short-description`.
- Commit messages: short imperative summary line, body if the *why* isn't
  obvious from the diff. Reference the issue number.
- Rebase on `main` before opening a PR; don't merge `main` into your branch
  repeatedly.

---

## 4. Before Opening a PR

- [ ] Builds clean via `west build` (or the app's Makefile) with no new
      warnings.
- [ ] Tested — on hardware if the change touches hardware-facing code, or in
      Renode/`ut_core_sim` if hardware isn't available to you.
- [ ] No debug-only code left in (stray `printk` spam, hardcoded test values,
      commented-out blocks) unless it belongs in `apps/tests/`.
- [ ] Linked to an issue.
- [ ] Devicetree/Kconfig changes are explained in the PR description — don't
      make the reviewer reverse-engineer why an overlay changed.

---

## 6. Documentation

Reference documentation (setup guides, architecture notes, debugging
writeups) lives in `ut-core-docs`, not scattered across README files or PR
descriptions. If you solve a nontrivial problem (a build quirk, a hardware
gotcha, a simulation workaround), write it up there — future contributors
will hit the same wall without it.

---

## 7. Getting Help

- Check `ut-core-docs` first.
- Check closed issues — the same problem has probably come up before.
- If you're stuck on something that isn't documented anywhere, that's a sign
  the docs need updating once you figure it out — please do, rather than
  letting the knowledge leave with you at the end of the semester.
