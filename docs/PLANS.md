# Plans

This file records current priorities, deferred work, and non-goals. Keep entries
short enough that an agent can quickly decide what matters.

## Current Focus

- Keep the source workspace focused on deployed hand control, tactile
  feedback, and hardware support.
- Avoid workspace-wide builds on machines where they freeze the system; prefer
  focused package builds.

## Active Tasks

### Repository Harness Cleanup

Status: Done

Goal:

- Replace template harness docs with current package maps, commands, decisions,
  and runbooks.

Validation:

- `colcon list --names-only` confirmed package discovery.
- Harness docs were checked for placeholder text and stale removed-guideline
  references.
- No build was required for documentation-only edits.

## Backlog

- Replace placeholder `package.xml` descriptions and license fields where
  ownership is clear.
- Add or document a standard fake-hardware smoke test for PLATO if one exists or
  is introduced.
- Add deterministic tests for config parsers when touching actuator, linkage, or
  grasp-plan YAML parsing.
- Decide whether to migrate `docs/frimware/` to `docs/firmware/` and update all
  references in one dedicated change.
- Consider a lightweight docs freshness check for `AGENTS.md` and root `docs/`
  links.
- Consolidate older root command notes from `cli_commands.md` into a clearer
  operator runbook if they continue to be used.

## Not Now

- Broad package restructuring.
- Renaming launch arguments or topics without an explicit compatibility plan.
- Reformatting unrelated source files.

## Open Questions

- Are `tactile_sensing` and `sdr_grasp_msgs` always expected in the same
  workspace, or should setup docs point to an external source?
