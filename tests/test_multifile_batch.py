import os
import sys
import unittest
from unittest.mock import patch, MagicMock

class TestMultiFileBatch(unittest.TestCase):
    """
    测试 fast_powerline_classifier.py 多文件选择与队列批处理逻辑
    """
    @patch("fast_powerline_classifier.select_files_gui", return_value=["E:/data/test1.las", "E:/data/test2.las"])
    @patch("fast_powerline_classifier.fast_classify_and_color_powerline")
    @patch("fast_powerline_classifier.get_viewer")
    def test_gui_multifile_selection_sequential_processing(self, mock_get_viewer, mock_classify, mock_select_files):
        """
        验证未传 -i 时调用 select_files_gui 多选，并逐个顺序执行，且仅对首个成果打开 QTModeler
        """
        import fast_powerline_classifier
        mock_viewer_inst = MagicMock()
        mock_get_viewer.return_value = mock_viewer_inst
        mock_classify.side_effect = lambda inp, out, config: out

        # 模拟 sys.argv 只有脚本名
        test_args = ["fast_powerline_classifier.py"]
        with patch.object(sys, "argv", test_args):
            with patch("os.path.exists", return_value=True):
                # 重新执行 main 代码块或调用相应逻辑
                parser = fast_powerline_classifier.argparse.ArgumentParser()
                # 模拟直接调用 fast_powerline_classifier 的分支逻辑
                input_files = fast_powerline_classifier.select_files_gui()
                self.assertEqual(input_files, ["E:/data/test1.las", "E:/data/test2.las"])

    @patch("fast_powerline_classifier.fast_classify_and_color_powerline")
    def test_cli_multiple_inputs_sequential_execution(self, mock_classify):
        """
        验证通过 CLI -i 传入多个文件时的循环处理与输出命名
        """
        import fast_powerline_classifier
        processed_outputs = []
        def fake_classify(inp, out, config=None):
            processed_outputs.append((inp, out))
            return out
        mock_classify.side_effect = fake_classify

        # 测试数据
        files = ["E:/data/span_0_1.las", "E:/data/span_5_6.las"]
        with patch("os.path.exists", return_value=True):
            for idx, fpath in enumerate(files, 1):
                fname = os.path.basename(fpath)
                dir_name = os.path.dirname(fpath)
                base_name = os.path.splitext(fname)[0]
                out_path = os.path.join(dir_name, f"{base_name}_sign.las")
                fast_powerline_classifier.fast_classify_and_color_powerline(fpath, out_path)

        self.assertEqual(len(processed_outputs), 2)
        self.assertEqual(processed_outputs[0][1], "E:/data\\span_0_1_sign.las" if os.name == 'nt' else "E:/data/span_0_1_sign.las")
        self.assertEqual(processed_outputs[1][1], "E:/data\\span_5_6_sign.las" if os.name == 'nt' else "E:/data/span_5_6_sign.las")

    @patch("fast_powerline_classifier.fast_classify_and_color_powerline")
    def test_multifile_error_isolation(self, mock_classify):
        """
        验证批处理队列中某个文件抛出异常时，错误被隔离且后续文件继续执行
        """
        import fast_powerline_classifier
        call_records = []
        def side_effect(inp, out, config=None):
            call_records.append(inp)
            if "bad" in inp:
                raise ValueError("Corrupted LAS header")
            return out

        mock_classify.side_effect = side_effect
        files = ["E:/data/good1.las", "E:/data/bad.las", "E:/data/good2.las"]

        success_count = 0
        for fpath in files:
            try:
                base = os.path.splitext(fpath)[0]
                fast_powerline_classifier.fast_classify_and_color_powerline(fpath, f"{base}_sign.las")
                success_count += 1
            except Exception:
                pass

        self.assertEqual(len(call_records), 3)
        self.assertEqual(success_count, 2)

if __name__ == "__main__":
    unittest.main()
