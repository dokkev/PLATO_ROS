# SPEC.md

Status: Accepted
Owner: Main agent
Last updated: 2026-03-29

## User Request

- Original request: mimic the Anthropic-style long-running harness structure as well as current Codex allows

## Product Goal

- Add a repo-local, file-based agent harness that separates planning, implementation, QA, review, and handoff work.
- Make the harness usable for future bounded tasks in `PLATO_ROS` without introducing automation or new dependencies.

## Scope

- Add `AGENTS.md` with role definitions and a concrete operating loop.
- Add working artifact files for spec, current plan, sprint contract, QA report, and handoff.
- Link the harness from `CLAUDE.md` so long-running tasks can find it from existing repo-local instructions.

## Non-Goals

- No automatic subagent orchestration.
- No scripts, services, or custom tooling.
- No attempt to retrofit existing feature work into the harness in this turn.

## Constraints

- Keep the change small and reviewable.
- Do not disturb the existing code or package structure.
- Use plain Markdown files only.
- The repository already has unrelated local changes; avoid touching product code.

## Architecture Sketch

- `CLAUDE.md` remains the repo-local instruction entry point.
- `AGENTS.md` defines the multi-agent harness loop and role mapping.
- `SPEC.md`, `CURRENT_PLAN.md`, `SPRINT_CONTRACT.md`, `QA_REPORT.md`, and `HANDOFF.md` act as persistent handoff artifacts.
- `docs/decisions/ADR-*.md` is reserved for decisions that should outlive a single sprint.

## Feature Backlog

- Slice 1: bootstrap the harness files and link them from repo-local instructions
- Slice 2: pilot the harness on one bounded feature in `PLATO_ROS`
- Slice 3: refine evaluator prompts and artifact usage based on the pilot

## Acceptance Criteria

- `AGENTS.md` exists and defines planner, explorer, worker, QA, and review roles.
- `SPEC.md`, `CURRENT_PLAN.md`, `SPRINT_CONTRACT.md`, `QA_REPORT.md`, and `HANDOFF.md` exist at the repo root with usable structure.
- `CLAUDE.md` points long-running work at the repo-local harness files.
- The change stays localized to Markdown process artifacts.

## Test Strategy

- Inspect file contents for coherence and internal consistency.
- Check `git status` for the intended modified and added files.
- Review `CLAUDE.md` and `AGENTS.md` with line numbers after the patch.

## Open Questions

- Does this Codex environment automatically consult `AGENTS.md`, or will the harness rely mainly on `CLAUDE.md` plus explicit use by the operator?
