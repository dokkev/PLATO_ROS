# AGENTS.md

Use this file as the repo-local harness for long-running or multi-agent work in `PLATO_ROS`.

## Goal

Keep the main thread focused on requirements, decisions, and final integration. Push bounded exploration, implementation, QA, and review work into explicit roles with file-based handoffs.

## Built-In Types And Assigned Roles

Codex built-in subagent types are:

- `default`
- `explorer`
- `worker`

Use them with these assigned roles:

- Main agent: owns user communication, prioritization, final decisions, and final integration
- Planner: usually `default`; expands a short request into `SPEC.md`
- Explorer: usually `explorer`; maps code paths, risks, and test infrastructure
- Worker: `worker`; implements one bounded slice from `SPRINT_CONTRACT.md`
- QA: usually `default` or `explorer`; verifies contract behavior and records evidence in `QA_REPORT.md`
- Review: usually `default` or `explorer`; checks maintainability, architecture drift, naming, docs, and tests

## Operating Loop

1. Planner phase
   - Restate the request.
   - Expand it into `SPEC.md`.
   - Define scope, non-goals, acceptance criteria, and test strategy.
2. Exploration phase
   - Use one or more `explorer` agents for read-heavy work.
   - Summarize architecture, relevant files, test entry points, and risks.
   - Update `CURRENT_PLAN.md`.
3. Contract phase
   - Write a bounded slice into `SPRINT_CONTRACT.md`.
   - Define what "done" means before implementation starts.
4. Implementation phase
   - Assign one `worker` to the bounded slice.
   - Keep edits local to the owned files.
   - Avoid speculative refactors.
5. Evaluation phase
   - Run QA and review separately from implementation.
   - Record evidence and failures in `QA_REPORT.md`.
   - Fix only confirmed failures or contract misses.
6. Handoff phase
   - Update `HANDOFF.md` with current state, remaining risks, and next actions.

## Rules

- Use extra harness only when it is load-bearing. Short, clear fixes do not need full ceremony.
- Separate builder and evaluator roles on ambiguous, long-running, or quality-sensitive tasks.
- Prefer file-based handoff over chat-heavy state.
- Parallelize read-heavy work freely when tasks are independent.
- Serialize write-heavy work unless file ownership is clearly disjoint.
- Do not let multiple workers edit the same file set at the same time.
- Keep the main thread clean: summarize findings, do not dump raw exploration logs.
- Evaluators must use explicit grading criteria, fail thresholds, edge cases, and pass/fail evidence.

## Required Artifacts

- `SPEC.md`
- `CURRENT_PLAN.md`
- `SPRINT_CONTRACT.md`
- `QA_REPORT.md`
- `HANDOFF.md`

Use `docs/decisions/ADR-*.md` only for decisions that should survive beyond a single sprint.
