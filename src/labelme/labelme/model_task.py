import argparse
import math
import os
import signal
import sys
import webbrowser
import winreg
import requests

from subprocess import Popen
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QVersionNumber
from PyQt5.QtGui import QPainter, QColor, QPen, QCursor
from PyQt5.QtWidgets import QWidget, QMainWindow, QFormLayout

from src.labelme.labelme.logger import logger
from src.labelme.labelme.labelme_main import labelme_main
from src.labelme.labelme.choose_cls_flags import ClsFlags

from src.labelme.labelme.app import MainWindow

from src.labelme.labelme.uicommponet.choice import Ui_Form
from src.labelme.labelme.uicommponet.labelme_title import LabelmeTitle
from src.labelme.labelme.uicommponet.upgrade import Upgrade, UpgradeThread

path = os.path.dirname(os.path.abspath(__file__))
Left, Top, Right, Bottom, LeftTop, RightTop, LeftBottom, RightBottom = range(8)


class ModelTask(QWidget, Ui_Form):
    Margins = 5

    def __init__(self, version_data, parent=None):
        super(ModelTask, self).__init__(parent)
        self.version_data = version_data
        self.win = None
        self.m_flag = None
        self.m_Position = None
        self._mpos = None
        self._pressed = False
        self.Direction = None
        # 隐藏头部
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        self.setMouseTracking(True)
        self.setupUi(self)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.cls_flags = None
        self.setWindowTitle(self.tr("Ascend Devkit Model Adapter"))
        self.title_label.setText(self.tr("Model Adapter"))
        self.label.setText(self.tr("Welcome"))
        self.label_5.setText(self.tr("Model Adapter"))
        self.label_7.setText(self.tr("Please select task"))
        self.cls_button.clicked.connect(lambda: self.show_labelme_main(self.tr("Classification")))
        self.det_button.clicked.connect(lambda: self.show_labelme_main(self.tr("Detection")))
        self.seg_button.clicked.connect(lambda: self.show_labelme_main(self.tr("Segmentation")))
        self.point_button.clicked.connect(lambda: self.show_labelme_main(self.tr("Keypoint")))
        self.help.clicked.connect(self.click_link)
        self.close_button.clicked.connect(self.close)
        self.hide_button.clicked.connect(self.showMinimized)

        self.showNormal()
        if self.read_reg() != "" and self.version_data.get('version') != " ":
            if QVersionNumber.fromString(self.version_data.get('version')) > QVersionNumber.fromString(self.read_reg()):
                self.upgrade = Upgrade(self.tr("INFO"),
                                       self.tr("A new version is available. Do you want to update it now?"))
                self.upgrade.yes.clicked.connect(self.download_and_update)
                self.upgrade.setWindowModality(Qt.NonModal)
                self.upgrade.show()

    def click_link(self):
        voc = self.version_data.get('voc')
        webbrowser.open(voc)

    def download_and_update(self):
        url = self.version_data.get('url')
        version = self.version_data.get('version')
        size = math.ceil(int(requests.head(url).headers.get('Content-Length'))/1024)
        self.filename = f'Ascend-devkit-model-adapter_{version}_win-x86_64_update.exe'
        self.upgrade.progressBar.show()
        self.downloadfile(url, self.filename, size)

    def downloadfile(self, url, filename, size):
        self.close()
        self.upgrade.msg.setText(self.tr("Downloading upgrade package..."))
        self.upgrade.setWindowModality(Qt.ApplicationModal)
        self.upgrade.close_button.setEnabled(False)
        self.upgrade.frame_7.hide()
        self.upgrade_thread = UpgradeThread(url, filename, size)
        self.upgrade_thread.process_value.connect(self.upgrade.progressBar.setValue)
        self.upgrade_thread.thread_end.connect(self.threadEnd)
        self.upgrade_thread.start()

    def threadEnd(self):
        self.upgrade.progressBar.setValue(100)
        self.upgrade.msg.setText(self.tr("Upgrade package download completed! Please stand by to install."))
        p = Popen('cmd.exe /c ' + self.filename)
        os.kill(os.getpid(), signal.SIGINT)

    def read_reg(self):
        location = r"Software\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\Ascend AI Devkit Model Adapter"
        try:
            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, location)
            i = 0
            while True:
                a = winreg.EnumValue(key, i)
                if a[0] == 'DisplayVersion':
                    return a[1]
                i += 1
        except Exception as e:
            return ""

    def main(self, model_no, version_data, filename=None, flags=argparse.SUPPRESS):
        self.close()
        config, filename, output_file, output_dir, model_task_win, reset_config = labelme_main(self, filename, flags)
        self.win = MainWindow(
            config=config,
            filename=filename,
            output_file=output_file,
            output_dir=output_dir,
            model_no=model_no,
            version_data=version_data,
            model_task_win=model_task_win,
        )
        if reset_config:
            logger.info("Resetting Qt config: %s" % self.win.settings.fileName())
            self.win.settings.clear()
            sys.exit(0)
        model_task_win = self
        self.final_win = FinalWin(model_task_win, model_no, version_data, filename, flags)
        self.final_win.show()
        self.final_win.showMaximized()
        self.final_win.raise_()

    def show_labelme_main(self, model_no):
        logger.info(f"You choose Model {model_no}")
        if model_no == self.tr("Classification"):
            self.cls_flags = ClsFlags(self, model_no, self.version_data)
            self.cls_flags.show()
        elif model_no == self.tr("Detection"):
            self.close()
            self.main(model_no, self.version_data)
        elif model_no == self.tr("Segmentation"):
            self.close()
            self.main(model_no, self.version_data)
        elif model_no == self.tr("Keypoint"):
            self.close()
            self.main(model_no, self.version_data)

    def closeEvent(self, event):
        if self.cls_flags:
            self.cls_flags.close()

    def move(self, pos):
        if self.windowState() == Qt.WindowMaximized or self.windowState() == Qt.WindowFullScreen:
            return
        super(ModelTask, self).move(pos)

    def showMaximized(self):
        """最大化,要去除上下左右边界,如果不去除则边框地方会有空隙"""
        super(ModelTask, self).showMaximized()
        self.setContentsMargins(0, 0, 0, 0)

    def showNormal(self):
        """还原,要保留上下左右边界,否则没有边框无法调整"""
        super(ModelTask, self).showNormal()

    def paintEvent(self, event):
        """由于是全透明背景窗口,重绘事件中绘制透明度为1的难以发现的边框,用于调整窗口大小"""
        super(ModelTask, self).paintEvent(event)
        painter = QPainter(self)
        painter.setPen(QPen(QColor(255, 255, 255, 1), 2 * self.Margins))
        painter.drawRect(self.rect())

    def mousePressEvent(self, event):
        """鼠标点击事件"""
        super(ModelTask, self).mousePressEvent(event)
        if event.button() == Qt.LeftButton and (
                event.globalPos().y() - self.pos().y() < self.Margins or event.globalPos().x() - self.pos().x() < self.Margins or event.globalPos().y() - self.pos().y() > self.height() - self.Margins or event.globalPos().x() - self.pos().x() > self.width() - self.Margins):
            # 尺寸变化
            self._mpos = event.pos()
            self._pressed = True
        elif event.button() == Qt.LeftButton and self.isMaximized() == False and event.globalPos().y() - self.pos().y() <= self.title.height():
            # 位置变化
            self.m_flag = True
            self.m_Position = event.globalPos() - self.pos()
            self.setCursor(QtGui.QCursor(Qt.OpenHandCursor))

    def mouseReleaseEvent(self, event):
        '''鼠标弹起事件'''
        super(ModelTask, self).mouseReleaseEvent(event)
        self.m_flag = False
        self._pressed = False
        self.Direction = None
        self.setCursor(Qt.ArrowCursor)

    def mouseMoveEvent(self, event):
        """鼠标移动事件"""
        super(ModelTask, self).mouseMoveEvent(event)
        self.setMouseTracking(True)
        pos = event.pos()
        if Qt.LeftButton and self.m_flag:
            self.move(event.globalPos() - self.m_Position)  # 更改窗口位置
            return


class FinalWin(QMainWindow):
    Margins = 5

    def __init__(self, model_task_win, model_no, version_data, filename, flags):
        super().__init__()
        self.setWindowTitle(self.tr("Ascend Devkit Model Adapter"))
        self.setMinimumSize(QtCore.QSize(1376, 770))
        self.labelme_title = LabelmeTitle()
        config, filename, output_file, output_dir, model_task_win, reset_config = labelme_main(model_task_win, filename,
                                                                                               flags)
        win = MainWindow(
            config=config,
            filename=filename,
            output_file=output_file,
            output_dir=output_dir,
            model_no=model_no,
            version_data=version_data,
            model_task_win=model_task_win,
            labelme_title=self.labelme_title,
            final_win=self,
        )
        if reset_config:
            logger.info("Resetting Qt config: %s" % win.settings.fileName())
            win.settings.clear()
            sys.exit(0)

        self.labelme_win = win
        self.model_task_win = model_task_win

        self.labelme_title.setFixedHeight(28)
        self.labelme_title.label_title.setText(self.tr("Ascend Devkit Model Adapter"))
        self.q = QWidget()
        self.q.setMouseTracking(True)
        self.play_out = QFormLayout(self.q)
        self.play_out.setContentsMargins(0, 0, 0, 0)
        self.play_out.setSpacing(0)
        self.play_out.addWidget(self.labelme_title)
        self.play_out.addWidget(self.labelme_win)

        self.m_flag = None
        self.m_Position = None
        self._mpos = None
        self._pressed = False
        self.Direction = None
        # 隐藏头部
        self.setWindowFlag(Qt.FramelessWindowHint)
        self.setMouseTracking(True)

        self.q.setLayout(self.play_out)
        self.setCentralWidget(self.q)

        self.setContentsMargins(
            self.Margins, self.Margins, self.Margins, self.Margins)
        self.setAttribute(Qt.WA_TranslucentBackground)

        self.labelme_title.close_button.clicked.connect(self.labelme_win.closeLabelme)
        self.labelme_title.hide_button.clicked.connect(self.click_hide)
        self.labelme_title.max_button.clicked.connect(self.click_max)

    def click_hide(self):
        self.showMinimized()

    def click_max(self):
        if self.isMaximized():
            self.showNormal()
        else:
            self.showMaximized()

    def click_close(self):
        self.labelme_win.close()
        # if self.labelme_win.isVisible():
        # self.close()
        # self.labelme_title.close()

    def closeEvent(self, event) -> None:
        self.labelme_win.closeLabelme()

    def move(self, pos):
        if self.windowState() == Qt.WindowMaximized or self.windowState() == Qt.WindowFullScreen:
            return
        super(FinalWin, self).move(pos)

    def showMaximized(self):
        """最大化,要去除上下左右边界,如果不去除则边框地方会有空隙"""
        super(FinalWin, self).showMaximized()
        self.setContentsMargins(0, 0, 0, 0)

    def showNormal(self):
        """还原,要保留上下左右边界,否则没有边框无法调整"""
        super(FinalWin, self).showNormal()
        self.setContentsMargins(
            self.Margins, self.Margins, self.Margins, self.Margins)

    def paintEvent(self, event):
        """由于是全透明背景窗口,重绘事件中绘制透明度为1的难以发现的边框,用于调整窗口大小"""
        super(FinalWin, self).paintEvent(event)
        painter = QPainter(self)
        painter.setPen(QPen(QColor(255, 255, 255, 1), 2 * self.Margins))
        painter.drawRect(self.rect())

    def mousePressEvent(self, event):
        """鼠标点击事件"""
        super(FinalWin, self).mousePressEvent(event)
        if event.button() == Qt.LeftButton and (
                event.globalPos().y() - self.pos().y() < self.Margins or event.globalPos().x() - self.pos().x() < self.Margins or event.globalPos().y() - self.pos().y() > self.height() - self.Margins or event.globalPos().x() - self.pos().x() > self.width() - self.Margins):
            # 尺寸变化
            self._mpos = event.pos()
            self._pressed = True
        elif event.button() == Qt.LeftButton and self.isMaximized() == False and event.globalPos().y() - self.pos().y() <= self.labelme_title.height():
            # 位置变化
            self.m_flag = True
            self.m_Position = event.globalPos() - self.pos()
            self.setCursor(QtGui.QCursor(Qt.OpenHandCursor))

    def mouseReleaseEvent(self, event):
        '''鼠标弹起事件'''
        super(FinalWin, self).mouseReleaseEvent(event)
        self.m_flag = False
        self._pressed = False
        self.Direction = None
        self.setCursor(Qt.ArrowCursor)

    def mouseMoveEvent(self, event):
        """鼠标移动事件"""
        super(FinalWin, self).mouseMoveEvent(event)
        self.setMouseTracking(True)
        pos = event.pos()
        xPos, yPos = pos.x(), pos.y()
        wm, hm = self.width() - self.Margins, self.height() - self.Margins
        if Qt.LeftButton and self.m_flag:
            self.move(event.globalPos() - self.m_Position)  # 更改窗口位置
            return
        if self.isMaximized() or self.isFullScreen():
            self.Direction = None
            self.setCursor(Qt.ArrowCursor)
            return
        if event.buttons() == Qt.LeftButton and self._pressed:
            self._resizeWidget(pos)
            return
        if self.Margins - 1 <= QCursor.pos().x() - self.pos().x() <= wm and self.Margins - 1 <= QCursor.pos().y() - self.pos().y() <= hm:
            # 里面
            self.Direction = None
            self.setCursor(Qt.ArrowCursor)
        elif xPos <= self.Margins and yPos <= self.Margins:
            # 左上角
            self.Direction = LeftTop
            self.setCursor(Qt.SizeFDiagCursor)
        elif wm <= xPos <= self.width() and hm <= yPos <= self.height():
            # 右下角
            self.Direction = RightBottom
            self.setCursor(Qt.SizeFDiagCursor)
        elif wm <= xPos and yPos <= self.Margins:
            # 右上角
            self.Direction = RightTop
            self.setCursor(Qt.SizeBDiagCursor)
        elif xPos <= self.Margins and hm <= yPos:
            # 左下角
            self.Direction = LeftBottom
            self.setCursor(Qt.SizeBDiagCursor)
        elif 0 <= xPos <= self.Margins and self.Margins <= yPos <= hm:
            # 左边
            self.Direction = Left
            self.setCursor(Qt.SizeHorCursor)
        elif wm <= xPos <= self.width() and self.Margins <= yPos <= hm:
            # 右边
            self.Direction = Right
            self.setCursor(Qt.SizeHorCursor)
        elif self.Margins <= xPos <= wm and 0 <= yPos <= self.Margins:
            # 上面
            self.Direction = Top
            self.setCursor(Qt.SizeVerCursor)
        elif self.Margins <= xPos <= wm and hm <= yPos <= self.height():
            # 下面
            self.Direction = Bottom
            self.setCursor(Qt.SizeVerCursor)

    def _resizeWidget(self, pos):
        """调整窗口大小"""
        if self.Direction == None:
            return
        mpos = pos - self._mpos
        xPos, yPos = mpos.x(), mpos.y()
        geometry = self.geometry()
        x, y, w, h = geometry.x(), geometry.y(), geometry.width(), geometry.height()
        if self.Direction == LeftTop:  # 左上角
            if w - xPos > self.minimumWidth():
                x += xPos
                w -= xPos
            if h - yPos > self.minimumHeight():
                y += yPos
                h -= yPos
        elif self.Direction == RightBottom:  # 右下角
            if w + xPos > self.minimumWidth():
                w += xPos
                self._mpos = pos
            if h + yPos > self.minimumHeight():
                h += yPos
                self._mpos = pos
        elif self.Direction == RightTop:  # 右上角
            if h - yPos > self.minimumHeight():
                y += yPos
                h -= yPos
            if w + xPos > self.minimumWidth():
                w += xPos
                self._mpos.setX(pos.x())
        elif self.Direction == LeftBottom:  # 左下角
            if w - xPos > self.minimumWidth():
                x += xPos
                w -= xPos
            if h + yPos > self.minimumHeight():
                h += yPos
                self._mpos.setY(pos.y())
        elif self.Direction == Left:  # 左边
            if w - xPos > self.minimumWidth():
                x += xPos
                w -= xPos
            else:
                return
        elif self.Direction == Right:  # 右边
            if w + xPos > self.minimumWidth():
                w += xPos
                self._mpos = pos
            else:
                return
        elif self.Direction == Top:  # 上面
            if h - yPos > self.minimumHeight():
                y += yPos
                h -= yPos
            else:
                return
        elif self.Direction == Bottom:  # 下面
            if h + yPos > self.minimumHeight():
                h += yPos
                self._mpos = pos
            else:
                return
        if w > QtWidgets.QDesktopWidget().screenGeometry().width():
            w = QtWidgets.QDesktopWidget().screenGeometry().width()
        if h > QtWidgets.QDesktopWidget().screenGeometry().height():
            h = QtWidgets.QDesktopWidget().screenGeometry().height()
        self.setGeometry(x, y, w, h)

    def mouseDoubleClickEvent(self, event) -> None:
        if event.globalPos().y() - self.pos().y() <= self.labelme_title.height():
            if self.isMaximized():
                self.showNormal()
            else:
                self.showMaximized()
