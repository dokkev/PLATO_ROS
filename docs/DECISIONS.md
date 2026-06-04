# Decisions

This file records important project decisions that affect architecture, interfaces, safety, experiments, or future refactoring.

Newest decisions should go first.

---

## YYYY-MM-DD - Decision title

Status: Proposed / Accepted / Deprecated / Replaced

### Context

TODO: What problem, trade-off, or constraint led to this decision?

### Decision

TODO: What was decided?

### Reason

TODO: Why is this better than the alternatives?

### Consequences

Positive:

* TODO

Trade-offs:

* TODO

### Alternatives considered

* TODO

### Related files

* TODO

---

## Example - Prefer explicit interfaces

Status: Accepted

### Context

Hidden behavior and implicit assumptions make the code harder to inspect, test, and refactor.

### Decision

Prefer explicit interfaces and locally understandable data flow.

### Reason

Explicit interfaces make the repository easier for humans and agents to modify safely.

### Consequences

Positive:

* Easier review
* Easier testing
* Easier future refactoring

Trade-offs:

* Some simple cases may require slightly more boilerplate

### Alternatives considered

* Hidden defaults
* Mode-specific behavior spread across multiple files

### Related files

* TODO
