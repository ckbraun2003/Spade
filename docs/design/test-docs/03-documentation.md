# Test/Docs — documentation

**Owner:** Test/Docs owns the structure of `docs/`. The lead owns the top level of the design library; each realm owns its own library's content. **Normative.**

## Where things live

- **The design library** is `docs/design/`, laid out in `../README.md`. Each realm has `README.md`, `00-decisions.md`, numbered specs, `07-status.md`, `plans/` and an optional `08-lessons.md`.
- **The front door** is `README.md`, `CONTRIBUTING.md`, `CHANGELOG.md`, `AGENTS.md` and `CLAUDE.md` at the root. They point into the library and do not restate it: status lives in the realms' `07-status.md`, not in the README.
- **Mechanisms live beside the code** in comments and in `engine/**/README.md`. A spec says what and why; the comment says how this code does it.
- **`docs/v1-transfer-register.md`** stays where it is, because `tests/test_transfer_register.cpp` reads it by path.
- **`tasks/`, `.superpowers/` and `design-specs/`** are local, untracked scratch. Anything another session or a fresh clone must see goes in a tracked file.

## Rules

1. **One fact, one place.** Decision status lives only in a `00-decisions.md`; current state lives only in a `07-status.md`. Everywhere else cites them.
2. **Cite a document by name and section, never by line number.** Qualify an ID by its series (`engine D7`, `SPIR-V rule P1`).
3. **Never edit the contents of `superseded/`.** It records what was believed and when. Moving files into it is fine.
4. **A behaviour change carries its doc change** in the same commit.
5. **A number in a doc says where it came from** (`TD-8`). A count is either recomputed by something or dated with its tree and commit.
6. **A correction is a one-line dated note.** The story belongs in the commit message.
7. **Recording an absence is not a broken link.** A spec that says a file was deleted, or a rule was retired, is right to name something that is not there. Check history (`git log --diff-filter=A -- <path>`) before "fixing" such a reference.

There are no machine checks over the library (restructure spec §4). Review catches drift.
