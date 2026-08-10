# Contributing to ut-core-zephyr

This document details the requirements and conventions necessary to keep the project usable and easy to maintain for future contributors. For documentation, please visit the [UT Core Cubesat Website](https://ut-core-cubesat.github.io/ut-core-docs/)

---

## 1. New Contributors, Start Here

If you're new to the project:

1. Read the main [README](./README.md) for project structure, build instructions, and onboarding resources.
2. Clone the repo and get a build working locally (see below) before touching anything else.
3. Skim open [GitHub Issues](https://github.com/UT-Core-CubeSat/ut-core-zephyr/issues) to get a feel for what's currently being worked on.
4. When you're ready to contribute, jump to [Section 4 (Use GitHub Issues)](#4-use-github-issues) and [Section 6 (Pull Requests)](#6-pull-requests).

---

## 2. Cloning the Repository

Keep in mind two of the main branches.

- **`master`** — stable, deployed firmware. Corresponds to the [stable docs site](https://ut-core-cubesat.github.io/ut-core-zephyr/stable).
- **`unstable`** — active development branch. Corresponds to the [unstable docs site](https://ut-core-cubesat.github.io/ut-core-zephyr/unstable).

> [!NOTE]
> For development, it is recommended you clone the `unstable` branch instead of `master`.

```bash
# Clone master
git clone https://github.com/ut-core-cubesat/ut-core-zephyr.git

# Clone unstable
git clone -b unstable https://github.com/ut-core-cubesat/ut-core-zephyr.git
```

If you're new to Git, the [Git Handbook](https://guides.github.com/introduction/git-handbook/) is a good quick primer, and the [Pro Git Book](https://git-scm.com/book/en/v2) is a free, thorough reference for anything deeper. 

---

## 3. Repository Hygiene — Read This Before Committing Anything

> [!IMPORTANT]
> These have caused real issues for this project before. Please keep these in consideration.

### Avoid committing large files

Large binary files bloat the git history permanently. Even if you delete the file in a later commit, it stays in history and the repo keeps growing forever, and is hard to remove. For example, try cloning the bloated corpse that is [ut-core-zephyr_old](https://github.com/UT-Core-CubeSat/ut-core-zephyr_old).

- Don't commit large binary files (datasets, videos, large images, zips, etc.)
- Don't commit build artifacts, downloaded SDKs/toolchains, modules, or anything regenerable (see below).
- If you genuinely need to track a large binary asset, please think twice if it is really neccessary.
- Before committing, check `git status` and think about whether what you're adding actually belongs in version control.
- Maintain the `.gitignore` file. The gitignore contains patterns to exclude unwanted files from making their way into the remote.

### Don't commit things that can be generated

If a file can be regenerated from something else already in the repo (build outputs, generated docs, compiled binaries, dependency lockfiles that aren't meant to be pinned, etc.), don't commit it — commit the *source* that generates it instead, and make sure `.gitignore` excludes the generated output.

### Use submodules for code from other repositories

If you need to include code that lives in another repository (for instance the control code), **use a git submodule** rather than copy-pasting the code in directly. If this isn't done, version control between the two repositories become a nightmare. This is an existing problem in this repository — please use submodules moving forward.

If you're not familiar with submodules, see [Git Submodules (Pro Git)](https://git-scm.com/book/en/v2/Git-Tools-Submodules)

---

## 4. Use GitHub Issues

All work, bugs, features, hardware, and tasks should go through GitHub Issues.

- **Before starting work**, check for an existing issue. If none exists, open one describing the problem or task before writing code.
- If you find a problem while working on something else, open a new issue for it rather than silently fixing it in an unrelated PR — makes history easier to follow later.

### Creating a good issue

When opening an issue, try to include:

- **What subsystem/area it affects** (and the matching label)
- **What you expected vs. what actually happened**, for bugs
- **Enough context that someone else could pick it up** without asking you first — this matters especially given the team turns over each semester
- Steps to reproduce, if it's a bug you hit while testing/building

A few sentences is usually enough — this doesn't need to be formal, just clear enough that the next reader (possibly a future you) isn't guessing.

---

## 5. Branching and Commits

- **Never push directly to `master` or `unstable`.** All changes go through a branch and a pull request (see [Section 6](#6-pull-requests)).
- How you branch is up to you — a branch per issue, or a single personal branch you work out of for a while, both work. Just make sure it isn't `master` or `unstable` directly.
- Commit messages: short imperative summary line, body if the *why* isn't obvious from the diff. Reference the issue number where relevant.
- Merge or rebase `unstable` into your branch periodically to stay current, rather than letting your branch drift far out of date.
- When opening a PR, make sure to link any related issues in the description (e.g. `Closes #123`) so history stays easy to trace.

---

## 6. Pull Requests

### How to open a pull request

1. Push your branch to the repository (or your fork, if you don't have write access):
   ```bash
   git push origin your-branch-name
   ```
2. On GitHub, navigate to the repo — you'll usually see a banner prompting **"Compare & pull request"** for your recently pushed branch. Click it. (If not, go to the **Pull requests** tab → **New pull request**.)
3. Set the **base branch** to `unstable` (or `doctest` for docs/tooling-only changes) and the **compare branch** to yours.
4. Fill in a title and description. Link any related issues (e.g. `Closes #123` or `Related to #45`).
5. Check off the relevant [checklist items](#7-pull-request-checklist) for the target branch in the PR description.
6. Click **Create pull request**.
7. Address any review feedback or failing CI checks, pushing additional commits to the same branch — they'll automatically show up on the open PR.
8. Once approved and checks pass, it'll be merged in.

For a more detailed walkthrough with screenshots, see GitHub's own guide: [Creating a pull request](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request).

---

## 7. Pull Request checklist

Before making a pull request to the unstable branch, check all of these requirements, if relevant.

- [ ] Successfully builds with no errors
- [ ] Tested on hardware
- [ ] Cleaned up debug-only code (stray logging statements, hardcoded test values)
- [ ] Changes to function behavior or prototype are documented via Doxygen
- [ ] Structural or API changes are documented
- [ ] If changes are made to the build system, ensure it builds for both Windows and Ubuntu/Debian

---

## 8. Documentation

Undocumented code is effectively unmaintainable once the previous group of students move on. **Assume whatever you don't document will be lost or forgotten!**. There are two places for documentation. The general documentation lives at the [ut-core-docs](https://github.com/UT-Core-CubeSat/ut-core-docs) repository, which is hosted [here](https://ut-core-cubesat.github.io/ut-core-docs/). The per-function docs, generated by Doxygen, are located here in the [ut-core-zephyr](https://github.com/UT-Core-CubeSat/ut-core-zephyr) repository.

**How Doxygen works:** Doxygen parses specially-formatted comments directly above the thing they describe. A comment block starting with `/**` or `///` (instead of a plain `/*` or `//`) is treated as documentation:

```
/**
 * @brief Short one-line summary of what this does.
 *
 * Longer explanation if needed — behavior, edge cases, units, etc.
 *
 * @param state  The new power state to apply.
 * @return 0 on success, negative errno on failure.
 */
int set_board_power_state(uint8_t state);
```

A few commands you'll use often:

| Command     | Purpose |
|-------------|---------|
| `@brief`    | One-line summary (shown in listings)
| `@param`    | Describe a function parameter
| `@return`   | Describe the return value
| `@file`     | Document a file itself (put at the top)
| `@defgroup` | Declares a new group/category page
| `@ingroup`  | Adds the current file or function to a group declared elsewhere with `@defgroup`
| `@todo`     | Flag unfinished work — collected into a project-wide TODO list
| `@bug`      | Flag a known issue — collected into a project-wide bug list

Use `@todo` and `@bug` freely when you notice something incomplete or broken but don't have time to fix it now — Doxygen aggregates these into dedicated pages so they're not lost in a comment nobody rereads.

For anything beyond this, the [Doxygen manual's documentation guide](https://www.doxygen.nl/manual/docblocks.html) and [special commands reference](https://www.doxygen.nl/manual/commands.html) cover the full comment syntax and command list.

If you find yourself struggling to understand something that is lacking in documentation, please either document it once resolved, or open a GitHub issue about it so someone else can.


---

## 9. Getting Help

- Check the [docs site](https://ut-core-cubesat.github.io/ut-core-docs/) first.
- Check the **Utah Tech Cubesat Onedrive** for stray documents, presentations, or videos.
- Contact one of the previous team members for clarification.
- If you're stuck on something that isn't documented anywhere, that's a sign the docs need updating once you figure it out — please do, rather than letting the knowledge leave with you at the end of the year.
