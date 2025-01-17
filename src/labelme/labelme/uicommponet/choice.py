# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'choice.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1376, 770)
        Form.setMinimumSize(QtCore.QSize(1376, 770))
        Form.setMaximumSize(QtCore.QSize(16777215, 16777215))
        Form.setStyleSheet("")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(Form)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setSpacing(0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.frame = QtWidgets.QFrame(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame.sizePolicy().hasHeightForWidth())
        self.frame.setSizePolicy(sizePolicy)
        self.frame.setMouseTracking(True)
        self.frame.setStyleSheet("#frame {\n"
"image: url(:/icon/image/choice/choice_bg.png);\n"
"}\n"
"* {\n"
"border-radius: 5px 5px;\n"
"}\n"
"")
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.frame)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.title = QtWidgets.QFrame(self.frame)
        self.title.setEnabled(True)
        self.title.setMinimumSize(QtCore.QSize(0, 48))
        self.title.setMaximumSize(QtCore.QSize(16777215, 48))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setBold(False)
        font.setWeight(50)
        self.title.setFont(font)
        self.title.setMouseTracking(True)
        self.title.setStyleSheet("background-color: rgb(255, 255, 255);\n"
"")
        self.title.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.title.setFrameShadow(QtWidgets.QFrame.Raised)
        self.title.setObjectName("title")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout(self.title)
        self.horizontalLayout_7.setContentsMargins(18, 0, 0, 0)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.title_button_2 = QtWidgets.QPushButton(self.title)
        self.title_button_2.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.title_button_2.sizePolicy().hasHeightForWidth())
        self.title_button_2.setSizePolicy(sizePolicy)
        self.title_button_2.setMinimumSize(QtCore.QSize(0, 0))
        self.title_button_2.setSizeIncrement(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.title_button_2.setFont(font)
        self.title_button_2.setMouseTracking(True)
        self.title_button_2.setStyleSheet("")
        self.title_button_2.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icon/image/flow/编组.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.title_button_2.setIcon(icon)
        self.title_button_2.setIconSize(QtCore.QSize(100, 24))
        self.title_button_2.setObjectName("title_button_2")
        self.horizontalLayout_7.addWidget(self.title_button_2)
        self.title_label_2 = QtWidgets.QLabel(self.title)
        self.title_label_2.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.title_label_2.sizePolicy().hasHeightForWidth())
        self.title_label_2.setSizePolicy(sizePolicy)
        self.title_label_2.setMinimumSize(QtCore.QSize(0, 0))
        self.title_label_2.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Light")
        font.setPointSize(12)
        font.setBold(False)
        font.setWeight(50)
        self.title_label_2.setFont(font)
        self.title_label_2.setMouseTracking(True)
        self.title_label_2.setStyleSheet("color: #DFE5EF;\n"
"background:rgba(0,0,0,0);")
        self.title_label_2.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.title_label_2.setObjectName("title_label_2")
        self.horizontalLayout_7.addWidget(self.title_label_2)
        self.title_label = QtWidgets.QLabel(self.title)
        self.title_label.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.title_label.sizePolicy().hasHeightForWidth())
        self.title_label.setSizePolicy(sizePolicy)
        self.title_label.setMinimumSize(QtCore.QSize(0, 0))
        self.title_label.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(11)
        font.setBold(False)
        font.setWeight(50)
        self.title_label.setFont(font)
        self.title_label.setMouseTracking(True)
        self.title_label.setStyleSheet("color: #000000;\n"
"background:rgba(0,0,0,0);")
        self.title_label.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.title_label.setObjectName("title_label")
        self.horizontalLayout_7.addWidget(self.title_label)
        self.help = QtWidgets.QPushButton(self.title)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.help.sizePolicy().hasHeightForWidth())
        self.help.setSizePolicy(sizePolicy)
        self.help.setMinimumSize(QtCore.QSize(0, 0))
        self.help.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.help.setFont(font)
        self.help.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.help.setMouseTracking(True)
        self.help.setStyleSheet("QPushButton{\n"
"border-style:none;\n"
"}")
        self.help.setText("")
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(":/icon/help.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.help.setIcon(icon1)
        self.help.setIconSize(QtCore.QSize(16, 16))
        self.help.setObjectName("help")
        self.horizontalLayout_7.addWidget(self.help)
        self.title_label_3 = QtWidgets.QLabel(self.title)
        self.title_label_3.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.title_label_3.sizePolicy().hasHeightForWidth())
        self.title_label_3.setSizePolicy(sizePolicy)
        self.title_label_3.setMinimumSize(QtCore.QSize(0, 0))
        self.title_label_3.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(11)
        font.setBold(False)
        font.setWeight(50)
        self.title_label_3.setFont(font)
        self.title_label_3.setMouseTracking(True)
        self.title_label_3.setStyleSheet("color: #000000;\n"
"background:rgba(0,0,0,0);")
        self.title_label_3.setText("")
        self.title_label_3.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.title_label_3.setObjectName("title_label_3")
        self.horizontalLayout_7.addWidget(self.title_label_3)
        self.frame_10 = QtWidgets.QFrame(self.title)
        self.frame_10.setMinimumSize(QtCore.QSize(72, 32))
        self.frame_10.setMaximumSize(QtCore.QSize(72, 32))
        font = QtGui.QFont()
        font.setFamily("Algerian")
        self.frame_10.setFont(font)
        self.frame_10.setMouseTracking(True)
        self.frame_10.setStyleSheet("border:none;\n"
"background-color:rgba(0,0,0,0);\n"
"border-top-left-radius: 0px;\n"
"border-top-right-radius: 0px;\n"
"border-bottom-right-radius: 0px;\n"
"border-bottom-left-radius: 0px;")
        self.frame_10.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_10.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_10.setObjectName("frame_10")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout(self.frame_10)
        self.horizontalLayout_8.setContentsMargins(0, 0, 12, 0)
        self.horizontalLayout_8.setSpacing(0)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.hide_button = QtWidgets.QPushButton(self.frame_10)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.hide_button.sizePolicy().hasHeightForWidth())
        self.hide_button.setSizePolicy(sizePolicy)
        self.hide_button.setMinimumSize(QtCore.QSize(0, 0))
        self.hide_button.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
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
        self.horizontalLayout_8.addWidget(self.hide_button)
        self.close_button = QtWidgets.QPushButton(self.frame_10)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.close_button.sizePolicy().hasHeightForWidth())
        self.close_button.setSizePolicy(sizePolicy)
        self.close_button.setMinimumSize(QtCore.QSize(0, 0))
        self.close_button.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
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
        self.horizontalLayout_8.addWidget(self.close_button)
        self.horizontalLayout_7.addWidget(self.frame_10)
        self.verticalLayout.addWidget(self.title)
        self.frame_3 = QtWidgets.QFrame(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_3.sizePolicy().hasHeightForWidth())
        self.frame_3.setSizePolicy(sizePolicy)
        self.frame_3.setMinimumSize(QtCore.QSize(0, 0))
        self.frame_3.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.frame_3.setMouseTracking(True)
        self.frame_3.setStyleSheet("")
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.frame_3)
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_6.setSpacing(0)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.label_10 = QtWidgets.QLabel(self.frame_3)
        self.label_10.setMinimumSize(QtCore.QSize(0, 70))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.label_10.setFont(font)
        self.label_10.setMouseTracking(True)
        self.label_10.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_10.setText("")
        self.label_10.setObjectName("label_10")
        self.verticalLayout_6.addWidget(self.label_10)
        self.frame_6 = QtWidgets.QFrame(self.frame_3)
        self.frame_6.setMinimumSize(QtCore.QSize(0, 0))
        self.frame_6.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.frame_6.setMouseTracking(True)
        self.frame_6.setStyleSheet("")
        self.frame_6.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_6.setObjectName("frame_6")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.frame_6)
        self.horizontalLayout_2.setContentsMargins(0, 9, 0, 9)
        self.horizontalLayout_2.setSpacing(0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label = QtWidgets.QLabel(self.frame_6)
        self.label.setMinimumSize(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.label.setFont(font)
        self.label.setMouseTracking(True)
        self.label.setStyleSheet("color: rgb(0, 0, 0);")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.horizontalLayout_2.addWidget(self.label)
        self.verticalLayout_6.addWidget(self.frame_6)
        self.frame_7 = QtWidgets.QFrame(self.frame_3)
        self.frame_7.setMinimumSize(QtCore.QSize(0, 0))
        self.frame_7.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.frame_7.setMouseTracking(True)
        self.frame_7.setStyleSheet("")
        self.frame_7.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_7.setObjectName("frame_7")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.frame_7)
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 9)
        self.horizontalLayout_3.setSpacing(0)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_5 = QtWidgets.QLabel(self.frame_7)
        self.label_5.setMinimumSize(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC")
        font.setPointSize(28)
        font.setBold(True)
        font.setWeight(75)
        self.label_5.setFont(font)
        self.label_5.setMouseTracking(True)
        self.label_5.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_3.addWidget(self.label_5)
        self.verticalLayout_6.addWidget(self.frame_7)
        self.frame_14 = QtWidgets.QFrame(self.frame_3)
        self.frame_14.setMinimumSize(QtCore.QSize(0, 0))
        self.frame_14.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.frame_14.setMouseTracking(True)
        self.frame_14.setStyleSheet("")
        self.frame_14.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_14.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_14.setObjectName("frame_14")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.frame_14)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setSpacing(0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_6 = QtWidgets.QLabel(self.frame_14)
        self.label_6.setMinimumSize(QtCore.QSize(0, 0))
        self.label_6.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.label_6.setSizeIncrement(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.label_6.setFont(font)
        self.label_6.setMouseTracking(True)
        self.label_6.setStyleSheet("color: #8D98AA")
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setWordWrap(True)
        self.label_6.setObjectName("label_6")
        self.verticalLayout_2.addWidget(self.label_6)
        self.label_8 = QtWidgets.QLabel(self.frame_14)
        self.label_8.setMinimumSize(QtCore.QSize(0, 0))
        self.label_8.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.label_8.setSizeIncrement(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.label_8.setFont(font)
        self.label_8.setMouseTracking(True)
        self.label_8.setStyleSheet("color: #8D98AA")
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setWordWrap(True)
        self.label_8.setObjectName("label_8")
        self.verticalLayout_2.addWidget(self.label_8)
        self.label_19 = QtWidgets.QLabel(self.frame_14)
        self.label_19.setMinimumSize(QtCore.QSize(0, 60))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.label_19.setFont(font)
        self.label_19.setMouseTracking(True)
        self.label_19.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_19.setText("")
        self.label_19.setObjectName("label_19")
        self.verticalLayout_2.addWidget(self.label_19)
        self.verticalLayout_6.addWidget(self.frame_14)
        self.frame_15 = QtWidgets.QFrame(self.frame_3)
        self.frame_15.setMinimumSize(QtCore.QSize(0, 0))
        self.frame_15.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.frame_15.setMouseTracking(True)
        self.frame_15.setStyleSheet("")
        self.frame_15.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_15.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_15.setObjectName("frame_15")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.frame_15)
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_4.setSpacing(0)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_7 = QtWidgets.QLabel(self.frame_15)
        self.label_7.setMinimumSize(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(12)
        font.setBold(False)
        font.setWeight(50)
        self.label_7.setFont(font)
        self.label_7.setMouseTracking(True)
        self.label_7.setStyleSheet("")
        self.label_7.setAlignment(QtCore.Qt.AlignCenter)
        self.label_7.setObjectName("label_7")
        self.horizontalLayout_4.addWidget(self.label_7)
        self.verticalLayout_6.addWidget(self.frame_15)
        self.verticalLayout.addWidget(self.frame_3)
        self.frame_5 = QtWidgets.QFrame(self.frame)
        self.frame_5.setMinimumSize(QtCore.QSize(0, 300))
        self.frame_5.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.frame_5.setSizeIncrement(QtCore.QSize(0, 0))
        self.frame_5.setMouseTracking(True)
        self.frame_5.setStyleSheet("")
        self.frame_5.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_5.setObjectName("frame_5")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.frame_5)
        self.horizontalLayout_6.setContentsMargins(110, 0, 110, 0)
        self.horizontalLayout_6.setSpacing(0)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.frame_8 = QtWidgets.QFrame(self.frame_5)
        self.frame_8.setMinimumSize(QtCore.QSize(0, 0))
        self.frame_8.setMaximumSize(QtCore.QSize(300, 300))
        self.frame_8.setMouseTracking(True)
        self.frame_8.setStyleSheet("background-repeat: norepeat;\n"
"background-position: 0% 0%;\n"
"border:none;\n"
"border-radius: 0px 0px;")
        self.frame_8.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_8.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_8.setObjectName("frame_8")
        self.cls_button = QtWidgets.QPushButton(self.frame_8)
        self.cls_button.setGeometry(QtCore.QRect(0, -20, 300, 320))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.cls_button.sizePolicy().hasHeightForWidth())
        self.cls_button.setSizePolicy(sizePolicy)
        self.cls_button.setMinimumSize(QtCore.QSize(0, 0))
        self.cls_button.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(14)
        self.cls_button.setFont(font)
        self.cls_button.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.cls_button.setMouseTracking(True)
        self.cls_button.setStyleSheet("QPushButton {\n"
"background-image: url(:/icon/image/choice/分类模型.png);\n"
"border: none;\n"
"}\n"
"QPushButton:hover {\n"
"background-image: url(:/icon/image/choice/分类模型hover.png);\n"
"border: none;\n"
"}")
        self.cls_button.setText("")
        self.cls_button.setIconSize(QtCore.QSize(400, 400))
        self.cls_button.setObjectName("cls_button")
        self.label_9 = QtWidgets.QLabel(self.frame_8)
        self.label_9.setGeometry(QtCore.QRect(30, 230, 241, 31))
        self.label_9.setMinimumSize(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC")
        font.setPointSize(11)
        font.setBold(True)
        font.setWeight(75)
        self.label_9.setFont(font)
        self.label_9.setMouseTracking(True)
        self.label_9.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setWordWrap(True)
        self.label_9.setObjectName("label_9")
        self.label_14 = QtWidgets.QLabel(self.frame_8)
        self.label_14.setGeometry(QtCore.QRect(30, 250, 241, 31))
        self.label_14.setMinimumSize(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.label_14.setFont(font)
        self.label_14.setMouseTracking(True)
        self.label_14.setStyleSheet("color: #8D98AA ;")
        self.label_14.setAlignment(QtCore.Qt.AlignCenter)
        self.label_14.setWordWrap(True)
        self.label_14.setObjectName("label_14")
        self.horizontalLayout_6.addWidget(self.frame_8)
        self.frame_11 = QtWidgets.QFrame(self.frame_5)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_11.sizePolicy().hasHeightForWidth())
        self.frame_11.setSizePolicy(sizePolicy)
        self.frame_11.setMinimumSize(QtCore.QSize(0, 0))
        self.frame_11.setMaximumSize(QtCore.QSize(300, 300))
        self.frame_11.setMouseTracking(True)
        self.frame_11.setStyleSheet("background-repeat: norepeat;\n"
"background-position: 0% 0%;\n"
"border:none;\n"
"border-radius: 0px 0px;")
        self.frame_11.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_11.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_11.setObjectName("frame_11")
        self.det_button = QtWidgets.QPushButton(self.frame_11)
        self.det_button.setGeometry(QtCore.QRect(0, -20, 300, 320))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.det_button.sizePolicy().hasHeightForWidth())
        self.det_button.setSizePolicy(sizePolicy)
        self.det_button.setMinimumSize(QtCore.QSize(0, 0))
        self.det_button.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.det_button.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.det_button.setMouseTracking(True)
        self.det_button.setStyleSheet("QPushButton {\n"
"background-image: url(:/icon/image/choice/检测模型.png);\n"
"border: none;\n"
"}\n"
"QPushButton:hover {\n"
"background-image: url(:/icon/image/choice/检测模型hover.png);\n"
"border: none;\n"
"}")
        self.det_button.setText("")
        self.det_button.setObjectName("det_button")
        self.label_11 = QtWidgets.QLabel(self.frame_11)
        self.label_11.setEnabled(False)
        self.label_11.setGeometry(QtCore.QRect(30, 230, 241, 31))
        self.label_11.setMinimumSize(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC")
        font.setPointSize(11)
        font.setBold(True)
        font.setWeight(75)
        self.label_11.setFont(font)
        self.label_11.setMouseTracking(True)
        self.label_11.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_11.setAlignment(QtCore.Qt.AlignCenter)
        self.label_11.setWordWrap(True)
        self.label_11.setObjectName("label_11")
        self.label_15 = QtWidgets.QLabel(self.frame_11)
        self.label_15.setEnabled(False)
        self.label_15.setGeometry(QtCore.QRect(30, 250, 241, 31))
        self.label_15.setMinimumSize(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.label_15.setFont(font)
        self.label_15.setMouseTracking(True)
        self.label_15.setStyleSheet("color: #8D98AA ;")
        self.label_15.setAlignment(QtCore.Qt.AlignCenter)
        self.label_15.setWordWrap(True)
        self.label_15.setObjectName("label_15")
        self.horizontalLayout_6.addWidget(self.frame_11)
        self.frame_12 = QtWidgets.QFrame(self.frame_5)
        self.frame_12.setMinimumSize(QtCore.QSize(0, 0))
        self.frame_12.setMaximumSize(QtCore.QSize(300, 300))
        self.frame_12.setMouseTracking(True)
        self.frame_12.setStyleSheet("background-repeat: norepeat;\n"
"background-position: 0% 0%;\n"
"border:none;\n"
"border-radius: 0px 0px;")
        self.frame_12.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_12.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_12.setObjectName("frame_12")
        self.seg_button = QtWidgets.QPushButton(self.frame_12)
        self.seg_button.setGeometry(QtCore.QRect(0, -20, 300, 320))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.seg_button.sizePolicy().hasHeightForWidth())
        self.seg_button.setSizePolicy(sizePolicy)
        self.seg_button.setMinimumSize(QtCore.QSize(0, 0))
        self.seg_button.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.seg_button.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.seg_button.setMouseTracking(True)
        self.seg_button.setStyleSheet("QPushButton {\n"
"background-image: url(:/icon/image/choice/分割模型.png);\n"
"border: none;\n"
"}\n"
"QPushButton:hover {\n"
"background-image: url(:/icon/image/choice/分割模型hover.png);\n"
"border: none;\n"
"}")
        self.seg_button.setText("")
        self.seg_button.setObjectName("seg_button")
        self.label_16 = QtWidgets.QLabel(self.frame_12)
        self.label_16.setGeometry(QtCore.QRect(30, 250, 241, 31))
        self.label_16.setMinimumSize(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.label_16.setFont(font)
        self.label_16.setMouseTracking(True)
        self.label_16.setStyleSheet("color: #8D98AA ;")
        self.label_16.setAlignment(QtCore.Qt.AlignCenter)
        self.label_16.setWordWrap(True)
        self.label_16.setObjectName("label_16")
        self.label_12 = QtWidgets.QLabel(self.frame_12)
        self.label_12.setGeometry(QtCore.QRect(30, 230, 241, 31))
        self.label_12.setMinimumSize(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC")
        font.setPointSize(11)
        font.setBold(True)
        font.setWeight(75)
        self.label_12.setFont(font)
        self.label_12.setMouseTracking(True)
        self.label_12.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_12.setAlignment(QtCore.Qt.AlignCenter)
        self.label_12.setWordWrap(True)
        self.label_12.setObjectName("label_12")
        self.horizontalLayout_6.addWidget(self.frame_12)
        self.frame_13 = QtWidgets.QFrame(self.frame_5)
        self.frame_13.setMinimumSize(QtCore.QSize(0, 0))
        self.frame_13.setMaximumSize(QtCore.QSize(300, 300))
        self.frame_13.setMouseTracking(True)
        self.frame_13.setStyleSheet("background-repeat: norepeat;\n"
"background-position: 0% 0%;\n"
"border: none;\n"
"border-radius: 0px 0px;")
        self.frame_13.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_13.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_13.setObjectName("frame_13")
        self.point_button = QtWidgets.QPushButton(self.frame_13)
        self.point_button.setGeometry(QtCore.QRect(0, -20, 300, 320))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.point_button.sizePolicy().hasHeightForWidth())
        self.point_button.setSizePolicy(sizePolicy)
        self.point_button.setMinimumSize(QtCore.QSize(0, 0))
        self.point_button.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.point_button.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.point_button.setMouseTracking(True)
        self.point_button.setStyleSheet("QPushButton {\n"
"background-image: url(:/icon/image/choice/关键点模型.png);\n"
"border: none;\n"
"}\n"
"QPushButton:hover {\n"
"background-image: url(:/icon/image/choice/关键点模型hover.png);\n"
"border: none;\n"
"}")
        self.point_button.setText("")
        self.point_button.setObjectName("point_button")
        self.label_17 = QtWidgets.QLabel(self.frame_13)
        self.label_17.setGeometry(QtCore.QRect(28, 250, 241, 31))
        self.label_17.setMinimumSize(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.label_17.setFont(font)
        self.label_17.setMouseTracking(True)
        self.label_17.setStyleSheet("color: #8D98AA ;")
        self.label_17.setAlignment(QtCore.Qt.AlignCenter)
        self.label_17.setWordWrap(True)
        self.label_17.setObjectName("label_17")
        self.label_13 = QtWidgets.QLabel(self.frame_13)
        self.label_13.setGeometry(QtCore.QRect(30, 230, 241, 31))
        self.label_13.setMinimumSize(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC")
        font.setPointSize(11)
        font.setBold(True)
        font.setWeight(75)
        self.label_13.setFont(font)
        self.label_13.setMouseTracking(True)
        self.label_13.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_13.setAlignment(QtCore.Qt.AlignCenter)
        self.label_13.setWordWrap(True)
        self.label_13.setObjectName("label_13")
        self.horizontalLayout_6.addWidget(self.frame_13)
        self.verticalLayout.addWidget(self.frame_5)
        self.frame_4 = QtWidgets.QFrame(self.frame)
        self.frame_4.setMinimumSize(QtCore.QSize(0, 0))
        self.frame_4.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.frame_4.setMouseTracking(True)
        self.frame_4.setStyleSheet("")
        self.frame_4.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.verticalLayout.addWidget(self.frame_4)
        self.verticalLayout_3.addWidget(self.frame)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.title_label_2.setText(_translate("Form", "  |  "))
        self.title_label.setText(_translate("Form", "模型适配工具"))
        self.label.setText(_translate("Form", "欢迎使用"))
        self.label_5.setText(_translate("Form", "模型适配工具"))
        self.label_6.setText(_translate("Form", "         模型适配工具是一款集成数据标注、模型迁移学习、模型打包为一体的开发者工具套件。降低开发者"))
        self.label_8.setText(_translate("Form", "在模型开发过程中对AI专业知识/深度学习框架学习的要求，并极大降低开发难度及复杂程度。"))
        self.label_7.setText(_translate("Form", "请选择您要进行的任务"))
        self.label_9.setText(_translate("Form", "分类模型"))
        self.label_14.setText(_translate("Form", "Image Classification"))
        self.label_11.setText(_translate("Form", "检测模型"))
        self.label_15.setText(_translate("Form", "Object Detection"))
        self.label_16.setText(_translate("Form", "Object Segmentation"))
        self.label_12.setText(_translate("Form", "分割模型"))
        self.label_17.setText(_translate("Form", "Keypoints Recognition"))
        self.label_13.setText(_translate("Form", "关键点模型"))
