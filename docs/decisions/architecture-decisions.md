# Architecture Decisions

This log records accepted decisions so future contributors and AI agents do not redefine the architecture without discussion.

## ADR-001: FastAPI Backend

**Decision:** Use FastAPI as the backend API layer.

**Reason:** It provides a lightweight Python interface for perception modules and dashboard integration.

**Status:** Accepted

## ADR-002: CARLA Simulation

**Decision:** Use CARLA as the primary controlled simulation environment.

**Reason:** It provides configurable roads, traffic, weather, actors, and sensors for repeatable experiments.

**Status:** Accepted

## ADR-003: Fixed-Resolution Baseline

**Decision:** Maintain fixed-resolution mapping as an experimental baseline.

**Reason:** A comparable baseline is required for quantitative evaluation of ADAPT-X.

**Status:** Accepted

## Decision Template

### ADR-XXX: Title

**Decision:**

**Reason:**

**Alternatives considered:**

**Impact:**

**Risks:**

**Status:** Proposed / Accepted / Superseded
