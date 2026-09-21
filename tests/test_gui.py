import os
import sys
import unittest
from unittest.mock import patch, MagicMock

class TestGuiModule(unittest.TestCase):
    def test_gui_module_lazy_import_no_eager_tkinter(self):
        # Importing modules.gui should never eagerly import tkinter into sys.modules
        if "tkinter" in sys.modules:
            del sys.modules["tkinter"]
        if "modules.gui" in sys.modules:
            del sys.modules["modules.gui"]

        import modules.gui
        self.assertNotIn("tkinter", sys.modules)

    def test_utils_module_no_eager_tkinter(self):
        # Importing modules.utils should never eagerly import tkinter
        if "tkinter" in sys.modules:
            del sys.modules["tkinter"]
        if "modules.utils" in sys.modules:
            del sys.modules["modules.utils"]

        import modules.utils
        self.assertNotIn("tkinter", sys.modules)

    def test_select_file_gui_success(self):
        from modules.gui import select_file_gui

        mock_tk = MagicMock()
        mock_filedialog = MagicMock()
        mock_filedialog.askopenfilename.return_value = "E:/data/test.las"

        with patch("tkinter.Tk", return_value=mock_tk):
            with patch("tkinter.filedialog.askopenfilename", return_value="E:/data/test.las"):
                selected = select_file_gui()
                self.assertEqual(selected, os.path.abspath("E:/data/test.las"))

    def test_select_file_gui_headless_fallback_to_input(self):
        from modules.gui import select_file_gui

        # Simulate headless environment where tkinter import fails
        with patch.dict(sys.modules, {"tkinter": None}):
            with patch("builtins.input", return_value="E:/data/headless.las"):
                with patch("os.path.exists", return_value=True):
                    selected = select_file_gui(allow_cli_fallback=True)
                    self.assertEqual(selected, os.path.abspath("E:/data/headless.las"))

    def test_select_file_gui_headless_no_fallback_returns_none(self):
        from modules.gui import select_file_gui

        with patch.dict(sys.modules, {"tkinter": None}):
            selected = select_file_gui(allow_cli_fallback=False)
            self.assertIsNone(selected)

    def test_select_file_gui_user_cancelled(self):
        from modules.gui import select_file_gui

        mock_tk = MagicMock()
        with patch("tkinter.Tk", return_value=mock_tk):
            with patch("tkinter.filedialog.askopenfilename", return_value=""):
                selected = select_file_gui()
                self.assertIsNone(selected)
                mock_tk.destroy.assert_called_once()

    @patch("modules.gui.select_file_gui", return_value="E:/data/shim.las")
    def test_utils_gui_shim(self, mock_gui_func):
        from modules.utils import select_file_gui

        res = select_file_gui()
        self.assertEqual(res, "E:/data/shim.las")
        mock_gui_func.assert_called_once()

if __name__ == "__main__":
    unittest.main()
