import os
import os.path as osp
import sys
import locale
import ctypes

import psutil
from PyQt5.QtGui import QFontDatabase
from qtpy import QtCore, QtGui
from qtpy import QtWidgets

from src.labelme.labelme import __appname__
from src.labelme.labelme.uicommponet.note import Notes
from src.labelme.labelme.utils import newIcon
from src.labelme.labelme.flow import FlowModel
from src.labelme.labelme.uicommponet import image_rc


def main():
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
    app = QtWidgets.QApplication(sys.argv)
    menu_qss = """
        QMenuBar {
            background-color: #EBEFF6;
            margin: 0 px;
            padding: 0 px;
        }
        QMenuBar::item {
            spacing: 3px;           
            padding: 14px 20px;
            background-color: #EBEFF6;
            color: #4E5865;
        }
        QMenuBar::item:hover {
            background-color: #DFE5EF;
            border: 5px solid #EBEFF6;
        }
        QMenuBar::item:selected {    
            background-color: #DFE5EF;
            border: 5px solid #EBEFF6;
        }
        QMenuBar::item:pressed {
            background: #C3CEDF;
            border: 5px solid #EBEFF6;
        }
    """
    app.setStyleSheet(menu_qss)
    app.setApplicationName(__appname__)
    app.setWindowIcon(newIcon("Ascendlogo", "svg"))
    translator = QtCore.QTranslator()
    filepath = "/src/labelme/labelme/translate"
    dirlist = os.listdir(osp.dirname(osp.abspath(__file__)))
    for curfile in dirlist:
        curpath = osp.join(osp.dirname(osp.abspath(__file__)), curfile)
        if osp.isdir(curpath) and curfile == "translate":
            filepath = "/translate"
            break
    windll = ctypes.windll.kernel32
    translator.load(
        locale.windows_locale[windll.GetUserDefaultUILanguage()],
        osp.dirname(osp.abspath(__file__)) + filepath
    )
    app.installTranslator(translator)
    fontdb = QFontDatabase()
    fontId1 = fontdb.addApplicationFont(":/font/font/HarmonyOS_Sans_SC_Medium.ttf")
    fontId2 = fontdb.addApplicationFont(":/font/font/HarmonyOS_Sans_SC_Light.ttf")
    fontId3 = fontdb.addApplicationFont(":/font/font/HarmonyOS_Sans_SC_Regular.ttf")
    # fontdb.applicationFontFamilies(fontId1)
    # fontdb.applicationFontFamilies(fontId2)
    font = QtGui.QFont()
    font.setFamily("HarmonyOS Sans SC Medium")
    font.setPointSize(9)
    font.setBold(False)
    font.setWeight(50)
    app.setFont(font)
    pl = psutil.pids()
    result = 0
    for pid in pl:
        if psutil.Process(pid).name() == "Ascend AI Devkit Model Adapter.exe":
            if isinstance(pid, int):
                result += 1
    if result > 1:
        app.setApplicationName("提示")
        notes = Notes("提示", "程序正在运行中.", "错误")
        notes.show()
        sys.exit(app.exec_())

    flow_model = FlowModel()

    flow_model.show()

    sys.exit(app.exec_())


# this main block is required to generate executable by pyinstaller
if __name__ == "__main__":
    main()

