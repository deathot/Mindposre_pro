# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'labelme_title.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt


class LabelmeTitle(QtWidgets.QMainWindow):
    def __init__(self):
        super(LabelmeTitle, self).__init__()
        # 隐藏头部
        self.setWindowFlag(Qt.FramelessWindowHint)
        # 隐藏边框
        self.setAttribute(Qt.WA_TranslucentBackground)
        Form_body = QtWidgets.QWidget()
        Form_body.setObjectName("Form_body")
        Form_body.resize(800, 28)
        Form_body.setMouseTracking(True)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(Form_body)
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_3.setSpacing(0)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.frame = QtWidgets.QFrame(Form_body)
        self.frame.setMinimumSize(QtCore.QSize(0, 28))
        self.frame.setMaximumSize(QtCore.QSize(16777215, 28))
        self.frame.setMouseTracking(True)
        self.frame.setStyleSheet("* {\n"
"    background: #EBEFF6;\n"
"    color: #000000;\n"
"}\n"
"\n"
"#frame {\n"
"    border-bottom: 1px solid #DFE5EF;\n"
"}")
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.horizontalLayout_14 = QtWidgets.QHBoxLayout(self.frame)
        self.horizontalLayout_14.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_14.setSpacing(0)
        self.horizontalLayout_14.setObjectName("horizontalLayout_14")
        self.frame_header_3 = QtWidgets.QFrame(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_header_3.sizePolicy().hasHeightForWidth())
        self.frame_header_3.setSizePolicy(sizePolicy)
        self.frame_header_3.setMinimumSize(QtCore.QSize(108, 0))
        self.frame_header_3.setMaximumSize(QtCore.QSize(108, 16777215))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.frame_header_3.setFont(font)
        self.frame_header_3.setMouseTracking(True)
        self.frame_header_3.setStyleSheet("")
        self.frame_header_3.setObjectName("frame_header_3")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.frame_header_3)
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_4.setSpacing(6)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.horizontalLayout_14.addWidget(self.frame_header_3)
        self.frame_header_2 = QtWidgets.QFrame(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_header_2.sizePolicy().hasHeightForWidth())
        self.frame_header_2.setSizePolicy(sizePolicy)
        self.frame_header_2.setMinimumSize(QtCore.QSize(0, 0))
        self.frame_header_2.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.frame_header_2.setSizeIncrement(QtCore.QSize(0, 0))
        self.frame_header_2.setBaseSize(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.frame_header_2.setFont(font)
        self.frame_header_2.setMouseTracking(True)
        self.frame_header_2.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.frame_header_2.setStyleSheet("")
        self.frame_header_2.setObjectName("frame_header_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.frame_header_2)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_title = QtWidgets.QLabel(self.frame_header_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_title.sizePolicy().hasHeightForWidth())
        self.label_title.setSizePolicy(sizePolicy)
        self.label_title.setSizeIncrement(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.label_title.setFont(font)
        self.label_title.setMouseTracking(True)
        self.label_title.setAlignment(QtCore.Qt.AlignCenter)
        self.label_title.setObjectName("label_title")
        self.horizontalLayout_2.addWidget(self.label_title)
        self.horizontalLayout_14.addWidget(self.frame_header_2)
        self.frame_header = QtWidgets.QFrame(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_header.sizePolicy().hasHeightForWidth())
        self.frame_header.setSizePolicy(sizePolicy)
        self.frame_header.setMinimumSize(QtCore.QSize(108, 26))
        self.frame_header.setMaximumSize(QtCore.QSize(108, 26))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.frame_header.setFont(font)
        self.frame_header.setMouseTracking(True)
        self.frame_header.setStyleSheet("")
        self.frame_header.setObjectName("frame_header")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.frame_header)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.hide_button = QtWidgets.QPushButton(self.frame_header)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.hide_button.sizePolicy().hasHeightForWidth())
        self.hide_button.setSizePolicy(sizePolicy)
        self.hide_button.setMinimumSize(QtCore.QSize(0, 0))
        self.hide_button.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.hide_button.setFont(font)
        self.hide_button.setMouseTracking(True)
        self.hide_button.setStyleSheet("QPushButton{\n"
"border-style:none;\n"
"background-image: url(:/icon/image/hide.svg);\n"
"background-position: center;\n"
"background-repeat: norepeat;    \n"
"}\n"
"QPushButton:hover{\n"
"    background-color: #DFE5EF;\n"
"background-image: url(:/icon/image/hide_hover.svg);\n"
"background-repeat: norepeat;\n"
"background-position: center;\n"
"}\n"
"QPushButton:pressed{\n"
"    background-color: #C3CEDF;\n"
"    color:#8D98AA;\n"
"}")
        self.hide_button.setText("")
        self.hide_button.setIconSize(QtCore.QSize(16, 16))
        self.hide_button.setObjectName("hide_button")
        self.horizontalLayout.addWidget(self.hide_button)
        self.max_button = QtWidgets.QPushButton(self.frame_header)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.max_button.sizePolicy().hasHeightForWidth())
        self.max_button.setSizePolicy(sizePolicy)
        self.max_button.setMinimumSize(QtCore.QSize(0, 0))
        self.max_button.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.max_button.setFont(font)
        self.max_button.setMouseTracking(True)
        self.max_button.setStyleSheet("QPushButton{\n"
"border-style:none;\n"
"background-image: url(:/icon/image/zoom.svg);\n"
"background-position: center;\n"
"background-repeat: norepeat;\n"
"}\n"
"QPushButton:hover{\n"
"    background-color: #DFE5EF;\n"
"    background-image: url(:/icon/image/zoom_hover.svg);\n"
"    background-position: center;\n"
"    background-repeat: norepeat;\n"
"}\n"
"QPushButton:pressed{\n"
"    background-color: #C3CEDF;\n"
"    color:#8D98AA;\n"
"}")
        self.max_button.setText("")
        self.max_button.setIconSize(QtCore.QSize(16, 16))
        self.max_button.setObjectName("max_button")
        self.horizontalLayout.addWidget(self.max_button)
        self.close_button = QtWidgets.QPushButton(self.frame_header)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.close_button.sizePolicy().hasHeightForWidth())
        self.close_button.setSizePolicy(sizePolicy)
        self.close_button.setMinimumSize(QtCore.QSize(0, 0))
        self.close_button.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.close_button.setFont(font)
        self.close_button.setMouseTracking(True)
        self.close_button.setStyleSheet("QPushButton{\n"
"border-style:none;\n"
"background-image: url(:/icon/image/close.svg);\n"
"background-position: center;\n"
"background-repeat: norepeat;\n"
"}\n"
"QPushButton:hover{\n"
"    background-color: #DFE5EF;\n"
"    background-image: url(:/icon/image/close_hover.svg);\n"
"    background-position: center;\n"
"    background-repeat: norepeat;\n"
"}\n"
"QPushButton:pressed{\n"
"    background-color: #C3CEDF;\n"
"    color:#8D98AA;\n"
"}")
        self.close_button.setText("")
        self.close_button.setIconSize(QtCore.QSize(16, 16))
        self.close_button.setObjectName("close_button")
        self.horizontalLayout.addWidget(self.close_button)
        self.horizontalLayout_14.addWidget(self.frame_header, 0, QtCore.Qt.AlignTop)
        self.horizontalLayout_3.addWidget(self.frame)

        self.retranslateUi(Form_body)
        QtCore.QMetaObject.connectSlotsByName(Form_body)
        self.setCentralWidget(Form_body)

    def retranslateUi(self, Form_body):
        _translate = QtCore.QCoreApplication.translate
        Form_body.setWindowTitle(_translate("Form_body", "Form"))
        self.label_title.setText(_translate("Form_body", "模型适配工具"))
