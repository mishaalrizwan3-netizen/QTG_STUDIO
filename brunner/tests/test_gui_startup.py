import os,sys,unittest
from pathlib import Path
from unittest.mock import patch
os.environ.setdefault("QT_QPA_PLATFORM","offscreen")
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import Super_Mushak_Logger_GUI as gui

class StartupTests(unittest.TestCase):
    def test_startup_connects_once_and_beep_does_not_reconnect(self):
        app=gui.QApplication.instance() or gui.QApplication([])
        with patch.object(gui.SuperMushakLoggerApp,"_start_hardware_connections") as connect:
            widget=gui.SuperMushakLoggerApp()
            widget.ui_timer.stop()
            try:
                connect.assert_called_once_with()
                with patch.object(gui.threading,"Thread"):
                    widget._play_beep("start")
                    widget._play_beep("stop")
                connect.assert_called_once_with()
            finally:
                widget.close()
