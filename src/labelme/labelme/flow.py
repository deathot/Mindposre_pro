import json
import webbrowser
import os 
import requests
from PyQt5 import QtGui
from PyQt5.QtCore import Qt

from PyQt5.QtWidgets import QWidget

from src.labelme.labelme.env_check import EnvCheckThread
from src.labelme.labelme.model_task import ModelTask

from src.labelme.labelme.uicommponet.flow import Ui_Form
from src.labelme.labelme.uicommponet.note import Notes


class FlowModel(QWidget, Ui_Form):
    Margins = 5

    def __init__(self):
        super(FlowModel, self).__init__()
        self.m_Position = None
        self.env_check_thread = None
        self.env_install_thread = None
        self.model_task = None
        self.title = None
        self.labelTitleOne = None
        self.labelTitleTwo = None
        self.pixmap = None
        self.style_file = None
        self.m_flag = None
        # 隐藏头部
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.setMouseTracking(True)
        self.setupUi(self)
        self.setContentsMargins(
            self.Margins, self.Margins, self.Margins, self.Margins)
        self.setAttribute(Qt.WA_TranslucentBackground)

        self.setWindowTitle(self.tr("Model Adapter"))
        self.title_label.setText(self.tr("Model Adapter"))
        self.warning_label_1.setText(self.tr("You have not conda environment installed! Please refer to "))
        self.warning_label_2.setText("<A style=\'color: #0077FF; text-decoration: none\'href=\'https://www.baidu.com\'>" + self.tr("installation guide") + "</A>")
        self.warning_label_3.setText(self.tr(" to install."))
        self.status_label.setText(self.tr("Environment checking..."))
        self.retry_button.setText(self.tr("Recheck"))
        self.exit_button.setText(self.tr("Exit"))
        self.warning_label_2.setOpenExternalLinks(False)
        self.warning_label_2.linkActivated.connect(self.click_link)
        self.retry_button.clicked.connect(self.show_model_task)
        self.retry_button.setEnabled(False)
        self.exit_button.clicked.connect(self.close)
        self.show_model_task()
        version_url = "https://ascend-repo.obs.cn-east-2.myhuaweicloud.com/Atlas%20200I%20DK%20A2/DevKit/tools/static/version.json"
        try:
            version_file_path = "./version.json"
            from pathlib import Path
            #download json file and save to disk for further usage.
            version_file = open(version_file_path, "wb")
            response = requests.get(version_url)
            version_file.write(response.content)

            self.version_data = json.loads(response.content.decode())
            # self.version_data = json.loads(version_file.read())

        except Exception as e:
            print("Exception is " + str(e))
            self.version_data = json.loads('{\n"version": " ", "url": " ", "voc": " ", "help": " "\n}')
            self.version_data["help"] = 'https://www.hiascend.com/document/detail/zh/Atlas200IDKA2DeveloperKit/23.0.RC1/Getting%20Started%20with%20Application%20Development/iaqd/iaqd_0002.html'
            self.notes = Notes(self.tr("WARNING"),
                               self.tr("The network connection is closed and transform cannot be worked."), "WARNING")
            self.notes.close_button.clicked.connect(self.notes.close)
            self.notes.button.clicked.connect(self.notes.close)
            self.notes.show()

    def click_link(self):
        help = self.version_data.get("help")
        webbrowser.open(help)
        self.m_flag = False
        self.setCursor(Qt.ArrowCursor)

    def check_thread_end(self, ret):
        self.status_label.setText(self.tr("Check succeeded"))
        if ret == -1:
            self.warning_1.show()
            self.status_1.show()
            self.retry_button.setEnabled(True)
        elif ret == 0:
            self.warning_1.show()
            self.status_1.show()
            self.retry_button.setEnabled(True)
        else:
            self.close()
            self.model_task = ModelTask(self.version_data)
            self.model_task.show()

    def show_model_task(self):
        self.retry_button.setEnabled(False)
        self.warning_1.hide()
        self.status_1.hide()
        self.status_label.setText(self.tr("Environment checking..."))
        self.env_check_thread = EnvCheckThread()
        self.env_check_thread.thread_end.connect(self.check_thread_end)
        self.env_check_thread.start()

    def move(self, pos):
        if self.windowState() == Qt.WindowMaximized or self.windowState() == Qt.WindowFullScreen:
            return
        super(FlowModel, self).move(pos)

    def mousePressEvent(self, event):
        """鼠标点击事件"""
        super(FlowModel, self).mousePressEvent(event)
        if event.button() == Qt.LeftButton:
            # 位置变化
            self.m_flag = True
            self.m_Position = event.globalPos() - self.pos()
            self.setCursor(QtGui.QCursor(Qt.OpenHandCursor))

    def mouseReleaseEvent(self, event):
        '''鼠标弹起事件'''
        super(FlowModel, self).mouseReleaseEvent(event)
        self.m_flag = False
        self.setCursor(Qt.ArrowCursor)

    def mouseMoveEvent(self, event):
        """鼠标移动事件"""
        super(FlowModel, self).mouseMoveEvent(event)
        self.setMouseTracking(True)
        if Qt.LeftButton and self.m_flag:
            self.move(event.globalPos() - self.m_Position)  # 更改窗口位置
            return