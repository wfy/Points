import unittest
import numpy as np
from modules.models import SpanSegment, PipelineResult

class TestSpanModels(unittest.TestCase):
    def test_span_segment_attributes(self):
        span = SpanSegment(
            span_index=0,
            tower_from_idx=1,
            tower_to_idx=2,
            tower_from_pos=np.array([100.0, 200.0]),
            tower_to_pos=np.array([300.0, 400.0]),
            span_length=282.84,
            point_indices=np.array([10, 11, 12], dtype=int),
            output_las_path="output/span_0.las"
        )
        self.assertEqual(span.span_index, 0)
        self.assertEqual(span.tower_from_idx, 1)
        self.assertEqual(span.tower_to_idx, 2)
        np.testing.assert_array_equal(span.tower_from_pos, np.array([100.0, 200.0]))
        np.testing.assert_array_equal(span.tower_to_pos, np.array([300.0, 400.0]))
        self.assertAlmostEqual(span.span_length, 282.84)
        np.testing.assert_array_equal(span.point_indices, np.array([10, 11, 12]))
        self.assertEqual(span.output_las_path, "output/span_0.las")

    def test_pipeline_result_with_spans(self):
        span = SpanSegment(
            span_index=0,
            tower_from_idx=0,
            tower_to_idx=1,
            tower_from_pos=np.array([0.0, 0.0]),
            tower_to_pos=np.array([100.0, 0.0]),
            span_length=100.0,
            point_indices=np.array([0, 1, 2], dtype=int)
        )
        res = PipelineResult(
            num_points=100,
            classification=np.zeros(100, dtype=np.uint8),
            towers=[],
            wires=[],
            spans=[span]
        )
        self.assertIsNotNone(res.spans)
        self.assertEqual(len(res.spans), 1)
        self.assertEqual(res.spans[0].span_length, 100.0)
        
        summary = res.summary()
        self.assertEqual(summary.get("span_count"), 1)

    def test_pipeline_result_default_spans_is_none(self):
        res = PipelineResult(
            num_points=50,
            classification=np.zeros(50, dtype=np.uint8),
            towers=[],
            wires=[]
        )
        self.assertIsNone(res.spans)
        summary = res.summary()
        self.assertNotIn("span_count", summary)

if __name__ == "__main__":
    unittest.main()
