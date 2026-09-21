import unittest
import numpy as np

from modules.models import WireCluster, WireExtractionResult, ExtractionResult

class TestWireModels(unittest.TestCase):
    def setUp(self):
        self.w1 = WireCluster(
            members=[0, 1, 2],
            center=np.array([10.0, 0.0, 20.0]),
            dir=np.array([1.0, 0.0, 0.0]),
            span=30.0,
            linearity=0.92,
            min_var=0.05,
            line_id=1,
            global_indices=np.array([100, 101, 102], dtype=int),
            is_suspect=False,
            is_jumper=False
        )
        self.w2 = WireCluster(
            members=[3, 4],
            center=np.array([10.0, 5.0, 20.0]),
            dir=np.array([1.0, 0.0, 0.0]),
            span=30.0,
            linearity=0.88,
            min_var=0.08,
            line_id=2,
            global_indices=np.array([200, 201], dtype=int),
            is_suspect=True,
            is_jumper=False
        )

    def test_wire_cluster_attributes_and_dict_access(self):
        self.assertEqual(self.w1.line_id, 1)
        np.testing.assert_array_equal(self.w1.global_indices, np.array([100, 101, 102]))
        self.assertFalse(self.w1.is_suspect)
        self.assertFalse(self.w1.is_jumper)

        # Dict-like access
        self.assertEqual(self.w1['line_id'], 1)
        self.assertEqual(self.w1.get('span'), 30.0)
        self.assertEqual(self.w1.to_dict()['line_id'], 1)

    def test_wire_extraction_result_modern_creation(self):
        all_cable_idx = np.array([100, 101, 102, 200, 201], dtype=int)
        res = WireExtractionResult(
            wires=[self.w1, self.w2],
            cable_indices=all_cable_idx,
            jumper_indices=np.array([], dtype=int)
        )

        self.assertEqual(len(res.wires), 2)
        np.testing.assert_array_equal(res.cable_indices, all_cable_idx)
        np.testing.assert_array_equal(res.cable_pts_idx, all_cable_idx)
        self.assertEqual(res.all_confirmed, [self.w1, self.w2])
        self.assertEqual(res.suspect_line_ids, {2})
        self.assertEqual(res.find_line_func(42), 42)

        # Test point_line_id lazy array generation
        pli = res.point_line_id
        self.assertEqual(pli[100], 1)
        self.assertEqual(pli[101], 1)
        self.assertEqual(pli[102], 1)
        self.assertEqual(pli[200], 2)
        self.assertEqual(pli[201], 2)
        self.assertEqual(pli[0], 0)

    def test_wire_extraction_result_unpacking_compatibility(self):
        all_cable_idx = np.array([100, 101, 102, 200, 201], dtype=int)
        res = WireExtractionResult(
            wires=[self.w1, self.w2],
            cable_indices=all_cable_idx
        )

        # 6-tuple unpack
        c_idx, p_id, confirmed, suspect, find_fn, ins_idx = res
        np.testing.assert_array_equal(c_idx, all_cable_idx)
        self.assertEqual(len(confirmed), 2)
        self.assertEqual(suspect, {2})
        self.assertEqual(find_fn(5), 5)

        # Indexing access
        self.assertEqual(res[3], {2})

    def test_legacy_extraction_result_kwargs_compatibility(self):
        legacy_pli = np.zeros(300, dtype=int)
        legacy_pli[[100, 101]] = 1
        res = ExtractionResult(
            cable_pts_idx=np.array([100, 101], dtype=int),
            point_line_id=legacy_pli,
            all_confirmed=[self.w1],
            suspect_line_ids={1},
            find_line_func=lambda x: x + 1,
            insulator_pts_idx=np.array([50], dtype=int)
        )

        self.assertEqual(len(res.wires), 1)
        np.testing.assert_array_equal(res.cable_indices, np.array([100, 101]))
        np.testing.assert_array_equal(res.insulator_indices, np.array([50]))
        self.assertEqual(res.suspect_line_ids, {1})
        self.assertEqual(res.find_line_func(1), 2)
        np.testing.assert_array_equal(res.point_line_id, legacy_pli)

if __name__ == '__main__':
    unittest.main()
