import os
import unittest
from unittest.mock import patch, MagicMock
from modules.config import ExportConfig
from modules.viewer import (
    ViewerHandler,
    NullViewer,
    SystemDefaultViewer,
    QTModelerViewer,
    QTModelerFinder,
    get_viewer,
    find_qtmodeler,
)

class TestViewerModule(unittest.TestCase):
    def test_null_viewer_contract(self):
        viewer = NullViewer()
        self.assertFalse(viewer.is_available())
        self.assertTrue(viewer.open("dummy.las"))
        self.assertTrue(viewer.close())

    def test_system_default_viewer_contract(self):
        viewer = SystemDefaultViewer()
        self.assertTrue(viewer.is_available())
        self.assertTrue(viewer.close())

    @patch("os.startfile", create=True)
    def test_system_default_viewer_open_windows(self, mock_startfile):
        viewer = SystemDefaultViewer()
        with patch("sys.platform", "win32"):
            success = viewer.open("dummy.las")
            self.assertTrue(success)
            mock_startfile.assert_called_once()

    def test_finder_with_env_override(self):
        fake_path = r"E:\Custom\QTModeler.exe"
        with patch.dict(os.environ, {"QTMODELER_PATH": fake_path}):
            with patch("os.path.exists", return_value=True):
                found = find_qtmodeler()
                self.assertEqual(found, fake_path)

    def test_finder_missing(self):
        with patch.dict(os.environ, {"QTMODELER_PATH": ""}):
            with patch("os.path.exists", return_value=False):
                found = find_qtmodeler()
                self.assertIsNone(found)

    @patch("modules.viewer.find_qtmodeler", return_value=r"C:\fake\QTModeler.exe")
    @patch("os.path.exists", return_value=True)
    @patch("subprocess.Popen")
    def test_qtmodeler_viewer_open_success(self, mock_popen, mock_exists, mock_find):
        viewer = QTModelerViewer()
        self.assertTrue(viewer.is_available())
        success = viewer.open("test.las")
        self.assertTrue(success)
        mock_popen.assert_called_once()

    @patch("modules.viewer.find_qtmodeler", return_value=None)
    @patch.object(SystemDefaultViewer, "open", return_value=True)
    def test_qtmodeler_viewer_fallback_when_missing(self, mock_system_open, mock_find):
        viewer = QTModelerViewer()
        self.assertFalse(viewer.is_available())
        success = viewer.open("test.las")
        self.assertTrue(success)
        mock_system_open.assert_called_once_with(os.path.abspath("test.las"))

    def test_factory_with_config_disabled(self):
        cfg = ExportConfig(open_qtmodeler=False)
        viewer = get_viewer(config=cfg)
        self.assertIsInstance(viewer, NullViewer)

    def test_factory_explicit_names(self):
        self.assertIsInstance(get_viewer(name="null"), NullViewer)
        self.assertIsInstance(get_viewer(name="system"), SystemDefaultViewer)
        self.assertIsInstance(get_viewer(name="qtmodeler"), QTModelerViewer)

    @patch("modules.viewer.find_qtmodeler", return_value=None)
    def test_factory_default_fallback_to_system_when_qt_missing(self, mock_find):
        cfg = ExportConfig(open_qtmodeler=True)
        viewer = get_viewer(config=cfg)
        self.assertIsInstance(viewer, SystemDefaultViewer)

    @patch("subprocess.run")
    def test_qtmodeler_viewer_close(self, mock_run):
        viewer = QTModelerViewer()
        with patch("sys.platform", "win32"):
            success = viewer.close()
            self.assertTrue(success)
            mock_run.assert_called_once()
            cmd = mock_run.call_args[0][0]
            self.assertIn("taskkill", cmd)

    def test_custom_executable_path_is_available(self):
        fake_bin = r"C:\custom\bin\QTModeler.exe"
        viewer = QTModelerViewer(executable_path=fake_bin)
        with patch("os.path.exists", side_effect=lambda p: p == fake_bin):
            self.assertTrue(viewer.is_available())

    def test_factory_unknown_name_raises(self):
        with self.assertRaises(ValueError):
            get_viewer(name="invalid_viewer_name")

    @patch("modules.viewer.get_viewer")
    def test_utils_viewer_shims(self, mock_get_viewer):
        from modules.utils import close_qtmodeler, open_in_qtmodeler

        mock_instance = MagicMock()
        mock_instance.close.return_value = True
        mock_instance.open.return_value = True
        mock_get_viewer.return_value = mock_instance

        self.assertTrue(close_qtmodeler())
        self.assertTrue(open_in_qtmodeler("dummy.las"))
        mock_instance.close.assert_called_once()
        mock_instance.open.assert_called_once_with("dummy.las")

if __name__ == "__main__":
    unittest.main()
