import os
import tempfile
import unittest
from unittest.mock import patch, MagicMock
import numpy as np
import laspy
from modules.models import TowerEntity, SpanSegment
from modules.corridor_cutter import (
    CorridorCutter,
    order_towers_along_line,
    cut_corridors_by_spans,
    export_split_spans
)

class TestCorridorCutter(unittest.TestCase):
    def test_order_towers_linear(self):
        # 4 towers placed along a line: x = 0, 100, 200, 300
        towers = [
            TowerEntity(cx=200.0, cy=0.0, max_z=30.0),
            TowerEntity(cx=0.0, cy=0.0, max_z=30.0),
            TowerEntity(cx=300.0, cy=0.0, max_z=30.0),
            TowerEntity(cx=100.0, cy=0.0, max_z=30.0),
        ]
        ordered = CorridorCutter.order_towers(towers)
        self.assertEqual(len(ordered), 4)
        # Sequence must be monotonic (either 1 -> 3 -> 0 -> 2 or reversed)
        coords = [towers[idx].cx for idx in ordered]
        is_increasing = coords == sorted(coords)
        is_decreasing = coords == sorted(coords, reverse=True)
        self.assertTrue(is_increasing or is_decreasing, f"Coords not monotonic: {coords}")

    def test_order_towers_corner_turn(self):
        # 4 towers with a 45 degree turn: (0,0) -> (100,0) -> (170, 70) -> (240, 140)
        towers = [
            TowerEntity(cx=170.0, cy=70.0, max_z=30.0),
            TowerEntity(cx=0.0, cy=0.0, max_z=30.0),
            TowerEntity(cx=240.0, cy=140.0, max_z=30.0),
            TowerEntity(cx=100.0, cy=0.0, max_z=30.0),
        ]
        ordered = CorridorCutter.order_towers(towers)
        self.assertEqual(len(ordered), 4)
        # Tower 1 (0,0) and Tower 2 (240,140) must be terminal ends
        self.assertTrue(
            (ordered[0] == 1 and ordered[-1] == 2) or (ordered[0] == 2 and ordered[-1] == 1)
        )

    def test_order_towers_boundary_cases(self):
        self.assertEqual(CorridorCutter.order_towers([]), [])
        single = [TowerEntity(cx=10.0, cy=20.0, max_z=30.0)]
        self.assertEqual(CorridorCutter.order_towers(single), [0])
        two = [TowerEntity(cx=10.0, cy=20.0, max_z=30.0), TowerEntity(cx=50.0, cy=60.0, max_z=30.0)]
        self.assertEqual(CorridorCutter.order_towers(two), [0, 1])

        # Coincident / near-zero delta towers must not trigger division by zero or NaN
        coincident = [
            TowerEntity(cx=0.0, cy=0.0, max_z=30.0),
            TowerEntity(cx=50.0, cy=0.0, max_z=30.0),
            TowerEntity(cx=50.0000001, cy=0.0, max_z=30.0),
            TowerEntity(cx=100.0, cy=0.0, max_z=30.0),
        ]
        ordered = CorridorCutter.order_towers(coincident)
        self.assertEqual(len(ordered), 4)

    def test_cut_spans_obb(self):
        # 2 towers: (0, 0) and (100, 0)
        towers = [
            TowerEntity(cx=0.0, cy=0.0, max_z=30.0),
            TowerEntity(cx=100.0, cy=0.0, max_z=30.0),
        ]
        # Points:
        # P0: (50, 0, 15) -> inside corridor
        # P1: (50, 10, 15) -> inside corridor (half_width=20)
        # P2: (50, 35, 15) -> outside corridor (y > 20)
        # P3: (-15, 0, 15) -> outside buffer (buffer=10, x < -10)
        # P4: (105, 0, 15) -> inside buffer (x <= 110)
        points = np.array([
            [50.0, 0.0, 15.0],
            [50.0, 10.0, 15.0],
            [50.0, 35.0, 15.0],
            [-15.0, 0.0, 15.0],
            [105.0, 0.0, 15.0],
        ])

        spans = CorridorCutter.cut_spans(
            points=points,
            tower_infos=towers,
            corridor_half_width=20.0,
            buffer_length=10.0
        )
        self.assertEqual(len(spans), 1)
        span = spans[0]
        self.assertEqual(span.span_index, 0)
        self.assertAlmostEqual(span.span_length, 100.0)
        # Expected indices: 0, 1, 4
        np.testing.assert_array_equal(np.sort(span.point_indices), np.array([0, 1, 4]))

    def test_export_spans(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            las_path = os.path.join(tmpdir, "test.las")
            header = laspy.LasHeader(point_format=3, version="1.2")
            las = laspy.LasData(header)
            las.x = np.array([10.0, 20.0, 30.0, 40.0])
            las.y = np.array([0.0, 0.0, 0.0, 0.0])
            las.z = np.array([5.0, 5.0, 5.0, 5.0])
            las.write(las_path)

            span = SpanSegment(
                span_index=0,
                tower_from_idx=0,
                tower_to_idx=1,
                tower_from_pos=np.array([0.0, 0.0]),
                tower_to_pos=np.array([100.0, 0.0]),
                span_length=100.0,
                point_indices=np.array([0, 2], dtype=int)
            )

            out_paths = CorridorCutter.export_spans(las_path, [span], output_dir=tmpdir)
            self.assertEqual(len(out_paths), 1)
            self.assertTrue(os.path.exists(out_paths[0]))
            
            # Read back sliced span
            sliced = laspy.read(out_paths[0])
            self.assertEqual(len(sliced.x), 2)
            np.testing.assert_array_equal(sliced.x, np.array([10.0, 30.0]))

    def test_backward_compatible_shims(self):
        towers = [
            TowerEntity(cx=0.0, cy=0.0, max_z=30.0),
            TowerEntity(cx=100.0, cy=0.0, max_z=30.0),
        ]
        ordered = order_towers_along_line(towers)
        self.assertEqual(ordered, [0, 1])

        points = np.array([[50.0, 0.0, 10.0]])
        spans = cut_corridors_by_spans(points, towers)
        self.assertEqual(len(spans), 1)

        with tempfile.TemporaryDirectory() as tmpdir:
            dummy_las = os.path.join(tmpdir, "test_sign.las")
            header = laspy.LasHeader(point_format=3, version="1.2")
            las = laspy.LasData(header)
            las.x = np.array([50.0])
            las.y = np.array([0.0])
            las.z = np.array([10.0])
            las.write(dummy_las)

            paths = export_split_spans(dummy_las, spans)
            self.assertEqual(len(paths), 1)
            self.assertTrue(os.path.exists(paths[0]))

    @patch("modules.pipeline_executor.PipelineExecutor.run", autospec=True)
    def test_split_raw_corridor_delegation(self, mock_executor_run):
        from unittest.mock import MagicMock
        with tempfile.TemporaryDirectory() as tmpdir:
            las_path = os.path.join(tmpdir, "raw.las")
            header = laspy.LasHeader(point_format=3, version="1.2")
            las = laspy.LasData(header)
            las.x = np.array([0.0, 50.0, 100.0])
            las.y = np.array([0.0, 0.0, 0.0])
            las.z = np.array([10.0, 10.0, 10.0])
            las.write(las_path)

            mock_res = MagicMock()
            mock_res.towers = [
                TowerEntity(cx=0.0, cy=0.0, max_z=30.0),
                TowerEntity(cx=100.0, cy=0.0, max_z=30.0),
            ]
            mock_executor_run.return_value = mock_res

            out_dir = os.path.join(tmpdir, "spans_raw")
            paths = CorridorCutter.split_raw_corridor(las_path, output_dir=out_dir)
            self.assertEqual(len(paths), 1)
            self.assertTrue(os.path.exists(paths[0]))
            mock_executor_run.assert_called_once()

    def test_pipeline_executor_short_circuit_with_spans(self):
        from modules.config import PipelineConfig, PipelineStage
        from modules.pipeline_executor import PipelineExecutor
        cfg = PipelineConfig()
        cfg.corridor.split_spans = True
        executor = PipelineExecutor(config=cfg)

        points = np.array([
            [0.0, 0.0, 10.0],
            [50.0, 0.0, 10.0],
            [100.0, 0.0, 10.0]
        ])

        with patch("modules.pipeline_executor.detect_towers") as mock_dt, \
             patch("modules.pipeline_executor.separate_ground") as mock_sg:
            mock_dt.return_value = (
                np.zeros(3, dtype=bool),
                np.zeros(3, dtype=bool),
                np.zeros(3, dtype=bool),
                [
                    TowerEntity(cx=0.0, cy=0.0, max_z=30.0),
                    TowerEntity(cx=100.0, cy=0.0, max_z=30.0)
                ]
            )
            mock_sg.return_value = (
                np.zeros(3, dtype=bool),
                np.array([0]),
                np.array([1, 2]),
                np.array([[50.0, 0.0, 10.0], [100.0, 0.0, 10.0]]),
                np.array([1.0, 1.0])
            )
            res = executor.run(points, config=cfg, stop_after=PipelineStage.TOWER)
            self.assertIsNotNone(res.spans)
            self.assertEqual(len(res.spans), 1)
            self.assertEqual(res.spans[0].span_index, 0)

    def test_pipeline_executor_in_memory_split_spans(self):
        from modules.config import PipelineConfig
        from modules.pipeline_executor import PipelineExecutor
        from modules.models import WireExtractionResult

        cfg = PipelineConfig()
        cfg.corridor.split_spans = True
        cfg.export.open_qtmodeler = False
        executor = PipelineExecutor(config=cfg)

        with tempfile.TemporaryDirectory() as tmpdir:
            in_las = os.path.join(tmpdir, "in.las")
            out_las = os.path.join(tmpdir, "out_sign.las")
            header = laspy.LasHeader(point_format=3, version="1.2")
            las = laspy.LasData(header)
            las.x = np.array([0.0, 50.0, 100.0])
            las.y = np.array([0.0, 0.0, 0.0])
            las.z = np.array([2.0, 2.0, 2.0])
            las.write(in_las)

            with patch("modules.pipeline_executor.detect_towers") as mock_dt, \
                 patch("modules.pipeline_executor.separate_ground") as mock_sg, \
                 patch("modules.wire_extractor.WireExtractor.extract") as mock_we:
                mock_dt.return_value = (
                    np.zeros(3, dtype=bool),
                    np.zeros(3, dtype=bool),
                    np.zeros(3, dtype=bool),
                    [
                        TowerEntity(cx=0.0, cy=0.0, max_z=30.0),
                        TowerEntity(cx=100.0, cy=0.0, max_z=30.0)
                    ]
                )
                mock_sg.return_value = (
                    np.zeros(3, dtype=bool),
                    np.array([0]),
                    np.array([1, 2]),
                    np.array([[50.0, 0.0, 2.0], [100.0, 0.0, 2.0]]),
                    np.array([1.0, 1.0])
                )
                mock_we.return_value = WireExtractionResult()

                res = executor.run_file(in_las, out_las, config=cfg)
                self.assertIsNotNone(res.spans)
                self.assertEqual(len(res.spans), 1)
                self.assertIn("split_span_paths", res.metadata)
                self.assertEqual(len(res.metadata["split_span_paths"]), 1)
                self.assertTrue(os.path.exists(res.metadata["split_span_paths"][0]))

if __name__ == "__main__":
    unittest.main()
