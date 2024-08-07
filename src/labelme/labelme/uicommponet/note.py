# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'note.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QWidget


class Ui_frame_body(object):
    def setupUi(self, frame_body, title, msg, ret):
        frame_body.setObjectName("frame_body")
        frame_body.resize(300, 150)
        frame_body.setMaximumSize(QtCore.QSize(16777215, 16777215))
        frame_body.setTabletTracking(True)
        self.verticalLayout = QtWidgets.QVBoxLayout(frame_body)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame_bg = QtWidgets.QFrame(frame_body)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_bg.sizePolicy().hasHeightForWidth())
        self.frame_bg.setSizePolicy(sizePolicy)
        self.frame_bg.setMinimumSize(QtCore.QSize(300, 150))
        self.frame_bg.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.frame_bg.setFont(font)
        self.frame_bg.setStyleSheet("* {\n"
"background: #FEFEFE;\n"
"border-style: none;\n"
"}\n"
"#frame_bg {\n"
"margin: 5px;\n"
"border:1px solid #DFE5EF;\n"
"}")
        self.frame_bg.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_bg.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_bg.setObjectName("frame_bg")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.frame_bg)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.frame_header = QtWidgets.QFrame(self.frame_bg)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_header.sizePolicy().hasHeightForWidth())
        self.frame_header.setSizePolicy(sizePolicy)
        self.frame_header.setMinimumSize(QtCore.QSize(0, 36))
        self.frame_header.setMaximumSize(QtCore.QSize(16777215, 36))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(12)
        self.frame_header.setFont(font)
        self.frame_header.setStyleSheet("background: #FEFEFE;\n"
"border-radius: 2px 2px 0 0;\n"
"color: #000000;\n"
"padding: 0px 5px;\n"
"border-bottom: 1px solid #DFE5EF;")
        self.frame_header.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_header.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_header.setObjectName("frame_header")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.frame_header)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.title = QtWidgets.QLabel(self.frame_header)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.title.sizePolicy().hasHeightForWidth())
        self.title.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(12)
        font.setBold(False)
        font.setWeight(50)
        self.title.setFont(font)
        self.title.setStyleSheet("border-style: none;")
        self.title.setObjectName("title")
        self.horizontalLayout.addWidget(self.title)
        self.close_button = QtWidgets.QPushButton(self.frame_header)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.close_button.sizePolicy().hasHeightForWidth())
        self.close_button.setSizePolicy(sizePolicy)
        self.close_button.setMinimumSize(QtCore.QSize(30, 0))
        self.close_button.setMaximumSize(QtCore.QSize(30, 16777215))
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.close_button.setFont(font)
        self.close_button.setStyleSheet("QPushButton{\n"
"border-style: none;\n"
"background-image: url(:/icon/image/new_close.svg);\n"
"background-position: center;\n"
"background-repeat: norepeat;\n"
"}\n"
"QPushButton:hover{\n"
"    background-color: #DFE5EF;\n"
"    background-image: url(:/icon/image/new_close_hover.svg);\n"
"    background-position: center;\n"
"    background-repeat: norepeat;\n"
"}\n"
"QPushButton:pressed{\n"
"    background-color: #C3CEDF;\n"
"    color:#8D98AA;\n"
"}")
        self.close_button.setObjectName("close_button")
        self.horizontalLayout.addWidget(self.close_button)
        self.verticalLayout_2.addWidget(self.frame_header)
        self.frame_form = QtWidgets.QFrame(self.frame_bg)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_form.sizePolicy().hasHeightForWidth())
        self.frame_form.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.frame_form.setFont(font)
        self.frame_form.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_form.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_form.setObjectName("frame_form")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.frame_form)
        self.verticalLayout_4.setContentsMargins(0, 9, 0, 9)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.frame_5 = QtWidgets.QFrame(self.frame_form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_5.sizePolicy().hasHeightForWidth())
        self.frame_5.setSizePolicy(sizePolicy)
        self.frame_5.setMinimumSize(QtCore.QSize(0, 0))
        self.frame_5.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.frame_5.setFont(font)
        self.frame_5.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_5.setObjectName("frame_5")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.frame_5)
        self.horizontalLayout_2.setContentsMargins(0, 0, 9, 15)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.icon_button = QtWidgets.QPushButton(self.frame_5)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.icon_button.sizePolicy().hasHeightForWidth())
        self.icon_button.setSizePolicy(sizePolicy)
        self.icon_button.setMinimumSize(QtCore.QSize(40, 0))
        self.icon_button.setMaximumSize(QtCore.QSize(40, 16777215))
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.icon_button.setFont(font)
        self.icon_button.setStyleSheet("")
        icon1 = QtGui.QIcon()
        if ret == "INFO":
            icon1.addPixmap(QtGui.QPixmap(":/icon/image/INFO.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        elif ret == "ERROR":
            icon1.addPixmap(QtGui.QPixmap(":/icon/image/ERROR.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        else:
            icon1.addPixmap(QtGui.QPixmap(":/icon/image/WARNING.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        # icon1.addPixmap(QtGui.QPixmap(":/icon/image/INFO.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.icon_button.setIcon(icon1)
        self.icon_button.setIconSize(QtCore.QSize(24, 24))
        self.icon_button.setObjectName("icon_button")
        self.horizontalLayout_2.addWidget(self.icon_button)
        self.msg = QtWidgets.QLabel(self.frame_5)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.msg.sizePolicy().hasHeightForWidth())
        self.msg.setSizePolicy(sizePolicy)
        self.msg.setMinimumSize(QtCore.QSize(0, 0))
        self.msg.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Light")
        font.setPointSize(10)
        self.msg.setFont(font)
        self.msg.setStyleSheet("color: #000000;")
        self.msg.setTextFormat(QtCore.Qt.AutoText)
        self.msg.setScaledContents(False)
        self.msg.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.msg.setWordWrap(True)
        self.msg.setObjectName("msg")
        self.horizontalLayout_2.addWidget(self.msg)
        self.verticalLayout_4.addWidget(self.frame_5)
        self.frame_7 = QtWidgets.QFrame(self.frame_form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_7.sizePolicy().hasHeightForWidth())
        self.frame_7.setSizePolicy(sizePolicy)
        self.frame_7.setMinimumSize(QtCore.QSize(0, 0))
        self.frame_7.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.frame_7.setFont(font)
        self.frame_7.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_7.setObjectName("frame_7")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.frame_7)
        self.horizontalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.button = QtWidgets.QPushButton(self.frame_7)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.button.sizePolicy().hasHeightForWidth())
        self.button.setSizePolicy(sizePolicy)
        self.button.setMinimumSize(QtCore.QSize(64, 24))
        self.button.setMaximumSize(QtCore.QSize(64, 24))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        self.button.setFont(font)
        self.button.setStyleSheet("QPushButton{\n"
"    color: #ffffff;\n"
"    background-color: #0077FF;\n"
"    border-radius: 2px;\n"
"}\n"
"QPushButton:hover{\n"
"    background-color: #52A3FF;\n"
"    border-radius: 2px;\n"
"}\n"
"QPushButton:pressed{\n"
"    background-color: #0077FF;\n"
"    border-radius: 2px;\n"
"}")
        self.button.setObjectName("button")
        self.horizontalLayout_6.addWidget(self.button, 0, QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
        self.verticalLayout_4.addWidget(self.frame_7)
        self.verticalLayout_2.addWidget(self.frame_form)
        self.verticalLayout.addWidget(self.frame_bg)

        self.retranslateUi(frame_body, title, msg)
        QtCore.QMetaObject.connectSlotsByName(frame_body)

    def retranslateUi(self, frame_body, title, msg):
        _translate = QtCore.QCoreApplication.translate
        frame_body.setWindowTitle(_translate("frame_body", title))
        if title == "ERROR":
            title = "错误"
        elif title == "INFO":
            title = "提示"
        elif title == "WARNING":
            title = "警告"
        self.title.setText(_translate("frame_body", title))
        self.msg.setText(msg)
        self.button.setText(_translate("frame_body", "确定"))

class Notes(QWidget, Ui_frame_body):
    def __init__(self, title, msg, ret):
        super().__init__()
        self.m_Position = None
        self.m_flag = None
        self.setWindowFlag(QtCore.Qt.FramelessWindowHint)
        self.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        shadow_effect = QtWidgets.QGraphicsDropShadowEffect(self)
        shadow_effect.setOffset(5, 5)
        shadow_effect.setColor(QColor(0, 0, 0, 26))
        shadow_effect.setBlurRadius(15)
        self.setupUi(self, title, msg, ret)
        self.frame_bg.setGraphicsEffect(shadow_effect)
        self.button.setText(self.tr("Yes"))
        self.close_button.clicked.connect(self.close)
        self.button.clicked.connect(self.close)
        self.setWindowModality(Qt.ApplicationModal)
        self.setGeometry(0, 0, 300, int(len(msg) / 21))
        screen = QtWidgets.QDesktopWidget().screenGeometry()
        size = self.geometry()
        self.move((screen.width() - size.width()) / 2, (screen.height() - size.height()) / 2 - 75)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and self.isMaximized() == False:
            self.m_flag = True
            self.m_Position = event.globalPos() - self.pos()  # 获取鼠标相对窗口的位置
            event.accept()
            self.setCursor(QtGui.QCursor(QtCore.Qt.OpenHandCursor))  # 更改鼠标图标

    def mouseMoveEvent(self, mouse_event):
        if QtCore.Qt.LeftButton and self.m_flag:
            self.move(mouse_event.globalPos() - self.m_Position)  # 更改窗口位置
            mouse_event.accept()

    def mouseReleaseEvent(self, mouse_event):
        self.m_flag = False
        self.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
