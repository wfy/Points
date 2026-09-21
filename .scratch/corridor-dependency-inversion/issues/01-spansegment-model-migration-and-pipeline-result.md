# 01: SpanSegment Model Migration and PipelineResult Enhancement

**What to build:**
Migrate the `SpanSegment` dataclass from `modules/corridor_cutter.py` to `modules/models.py` to establish a single source of truth for domain models. Enhance `PipelineResult` with an optional `spans: Optional[List[SpanSegment]] = None` field and update `summary()` to reflect span counts when present. Re-export `SpanSegment` in `modules/corridor_cutter.py` and `modules/__init__.py` to maintain backward compatibility.

**Blocked by:** None (can start immediately)

**Status:** ready-for-agent

- [x] Move `SpanSegment` dataclass definition into `modules/models.py`.
- [x] Add `spans: Optional[List[SpanSegment]] = None` to `PipelineResult` in `modules/models.py`.
- [x] Re-export `SpanSegment` in `modules/corridor_cutter.py` and `modules/__init__.py`.
- [x] Add unit tests in `tests/test_span_models.py` verifying `SpanSegment` creation, attribute access, and integration with `PipelineResult`.
