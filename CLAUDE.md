
## Writing Style (persistent artifacts)

Applies to: docs (`README.md`, `docs/**`, code comments), commit
messages, PR descriptions, launch-file comments, anything that lands
in the repo. Does NOT apply to chat replies (those can be Korean).

- **English only.** Even when the chat is in Korean, the persistent
  artifact stays English. Proper nouns (people's names, place names)
  may be romanized (e.g. `Park Sungjun`) rather than left in Hangul.
- **Plain prose.** No `**bold**` emphasis in prose. Use bold only for
  table cells where it carries semantic weight (e.g. winning value in
  a metric column), and even there, sparingly.
- **State directly.** Lead with the fact, not "Note that …" or "It
  should be noted that …". Trim filler.
- **Short inline comments.** Comments between code lines are at most
  2 lines (3 in exceptional cases). Long prose lives only at the top
  of a file: the C file-header comment or the Python module docstring.
  No internal jargon in comments (run numbers, milestone labels,
  work-log references) — spell out the surviving constraint instead.

---

## Git Commit Policy

Commit code changes automatically as work progresses. Follow these rules strictly.

### When to commit
- After completing each logical unit of work (a feature, fix, or refactor — not every keystroke).
- After tests pass for the changed area.
- Before switching context to an unrelated task.
- Never end a session with a dirty working tree.

### Commit message format
Use Conventional Commits:
- `feat: <what was added>`
- `fix: <what was fixed>`
- `refactor: <what was restructured>`
- `perf: <performance change>`
- `docs: <doc changes>`
- `test: <test changes>`
- `chore: <tooling, deps, config>`

Rules:
- Subject ≤ 72 chars, imperative mood, no trailing period.
- For non-trivial changes, add a blank line and a body explaining **why**, not what.
- Scope prefix when useful: `feat(ppo): add observation masking`.

### Staging
- Prefer explicit paths or `git add -p`. Do **not** use `git add .` or `git add -A` without first reviewing `git status`.
- Keep commits atomic — one logical change per commit. Split unrelated edits.

### Pre-commit checklist
1. `git status` + `git diff --staged` to verify scope.
2. Run the project's test command if one exists.
3. Run linter / formatter if configured.
4. Abort the commit if any of the above fail; fix first, commit after.

### Never commit
- Secrets, API keys, tokens, `.env*` files.
- Large binaries, datasets, checkpoints, rosbags, model weights (>10 MB). Add to `.gitignore` instead.
- Debug prints, `TODO: remove`, or commented-out code left in by accident.
- Broken code to `main`. Use a WIP branch if you must checkpoint mid-work.

### Never do without explicit confirmation
- `git push --force` or `--force-with-lease` on shared branches.
- `git reset --hard`, `git clean -fdx`, or anything that discards uncommitted work.
- History rewrites (`rebase -i`, `commit --amend`) on already-pushed commits.
- Deleting or renaming branches.

### Attribution
Do not add `Co-authored-by: Claude` or any AI-attribution trailer to commit messages.

### Push policy
Push only when the user explicitly requests it in the current conversation.
Never push on your own initiative — not after commits, not at session end.
Commit locally and wait for a push request.

---

## Progress Tracking

Maintain a persistent progress log under `docs/`. This is the project's single source of truth for "where are we?" — separate from git history (which is for *what changed*) and from CLAUDE.md (which is for *stable rules*).

Structure:
- `docs/PROGRESS.md` — index only (pointers + current `## Open` list + writing conventions). Keep under ~50 lines.
- `docs/progress/<topic>.md` — per-topic work logs. The actual entries live here.
- `docs/SIM_TO_REAL.md` (and similar planning docs) — forward-looking plans with `[ ]` checkboxes, not work logs.

### Session start
At the beginning of every session:
1. Read `docs/PROGRESS.md` — see the topic list + `## Open`. Open the relevant topic file(s) for the active work.
2. If `PROGRESS.md` doesn't exist yet, create it as an index using the existing one as a template.
3. Briefly summarize the current state back to me before proposing next steps.

### When to update the progress log
- **In the same commit as the code change it documents.** Stage the progress bullet together with the code — one commit per logical unit, not a separate `docs(progress)` commit afterward. The bullet records *what* and *why*; it need not cite a SHA (the commit itself is the link).
- When an experiment finishes (whether it succeeded, failed, or was inconclusive — log all three).
- When a design decision is made. Record *what* and *why*, not just *what*.
- When you discover something non-obvious (a bug cause, a library quirk, a baseline number).
- When a blocker appears or is resolved.

### Where to put a new entry
- Add new entries to the **top** of the relevant `docs/progress/<topic>.md` (newest first within each topic file).
- If no existing topic file fits, create a new one and link it from `docs/PROGRESS.md`.
- If a topic file grows past ~500 lines, consider splitting it further.

### Update format
Use this structure inside the topic file:

```markdown
