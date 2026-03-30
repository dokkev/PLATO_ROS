# SPRINT_CONTRACT.md

Status: Completed
Sprint ID: 001
Owner: Main agent
Last updated: 2026-03-29

## Objective

- Bootstrap a minimal Anthropic-style harness structure for this repository using repo-local Markdown artifacts.

## In Scope

- Add `AGENTS.md` with role mapping and operating loop.
- Add `SPEC.md`, `CURRENT_PLAN.md`, `SPRINT_CONTRACT.md`, `QA_REPORT.md`, and `HANDOFF.md`.
- Add a minimal `docs/decisions/ADR-TEMPLATE.md`.
- Link the harness from `CLAUDE.md`.

## Out Of Scope

- No automation scripts.
- No code generation or product feature changes.
- No custom agent runtime or automatic contract negotiation.

## Owned Files Or Packages

- `AGENTS.md`
- `SPEC.md`
- `CURRENT_PLAN.md`
- `SPRINT_CONTRACT.md`
- `QA_REPORT.md`
- `HANDOFF.md`
- `docs/decisions/ADR-TEMPLATE.md`
- `CLAUDE.md`

## Done Definition

- Done means the repo contains persistent Markdown artifacts for planning, contracts, QA, and handoff.
- Done means `CLAUDE.md` points long-running tasks at the harness files.
- Done means the artifact files are seeded with the bootstrap task rather than left completely blank.

## Verification Plan

- Check `git status` for the intended file additions and modifications only.
- Read back the new files and `CLAUDE.md` to confirm the structure and references are coherent.

## Risks To Watch

- `AGENTS.md` may not be auto-loaded in every Codex path, so the harness may still depend on `CLAUDE.md` and operator discipline.

## Handoff To QA And Review

- What should QA prove? The files exist, are internally consistent, and satisfy the sprint objective.
- What should review inspect? Whether the scaffold is minimal, realistic for Codex, and not pretending to be more automated than it is.
