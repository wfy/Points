# 01: Anchor WaistBoundaryElevation and Full Jumper Inclusion

**What to build:**
Explicitly extract and anchor `WaistBoundaryElevation` at 1.5m below the lowest candidate crossarm and tension jumper nadir in `modules/tower_detector.py`. Ensure that all cantilever crossarms, diagonal under-bracing angle steels, and tension jumper loops are 100% captured within the upper envelope, completely eliminating diagonal bracing severing at the transition elevation.

**Blocked by:** None (can start immediately)

**Status:** ready-for-agent

- [x] Anchor `WaistBoundaryElevation` at `min(crossarm_candidate_z) - 1.5m` (with fallback to 0.48~0.60 H when no crossarms detected).
- [x] Ensure all candidate crossarm slices and tension jumper points are strictly enclosed within upper envelope.
- [x] Add unit test verifying that diagonal under-bracing angle steels located within 1.5m below the lowest crossarm are 100% retained.
