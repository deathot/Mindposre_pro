# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'classindex.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form_body(object):
    def setupUi(self, Form_body):
        Form_body.setObjectName("Form_body")
        Form_body.resize(656, 352)
        self.verticalLayout = QtWidgets.QVBoxLayout(Form_body)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame_bg = QtWidgets.QFrame(Form_body)
        self.frame_bg.setMinimumSize(QtCore.QSize(656, 352))
        self.frame_bg.setMaximumSize(QtCore.QSize(656, 352))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.frame_bg.setFont(font)
        self.frame_bg.setStyleSheet("* {\n"
"border-radius: 5px 5px;\n"
"}\n"
"#frame_bg {\n"
"background: #FEFEFE;\n"
"margin: 5px;\n"
"border:1px solid #DFE5EF;\n"
"}\n"
"")
        self.frame_bg.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_bg.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_bg.setObjectName("frame_bg")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.frame_bg)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 9)
        self.verticalLayout_2.setSpacing(0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.frame_header = QtWidgets.QFrame(self.frame_bg)
        self.frame_header.setMinimumSize(QtCore.QSize(0, 50))
        self.frame_header.setMaximumSize(QtCore.QSize(16777215, 50))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.frame_header.setFont(font)
        self.frame_header.setStyleSheet("background: #FEFEFE;\n"
"color: #000000;\n"
"padding: 0px 5px;\n"
"border-bottom: 1px solid #DFE5EF;")
        self.frame_header.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_header.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_header.setObjectName("frame_header")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.frame_header)
        self.horizontalLayout_5.setContentsMargins(9, 0, 0, 0)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_title = QtWidgets.QLabel(self.frame_header)
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC")
        font.setPointSize(11)
        font.setBold(True)
        font.setWeight(75)
        self.label_title.setFont(font)
        self.label_title.setObjectName("label_title")
        self.horizontalLayout_5.addWidget(self.label_title)
        self.hide_button = QtWidgets.QPushButton(self.frame_header)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.hide_button.sizePolicy().hasHeightForWidth())
        self.hide_button.setSizePolicy(sizePolicy)
        self.hide_button.setMinimumSize(QtCore.QSize(0, 30))
        self.hide_button.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.hide_button.setFont(font)
        self.hide_button.setStyleSheet("QPushButton{\n"
"border-style:none;\n"
"background-image: url(:/icon/image/new_minimize.svg);\n"
"background-position: center;\n"
"background-repeat: norepeat;    \n"
"}\n"
"QPushButton:hover{\n"
"    background-color: #F4F6FA;\n"
"background-image: url(:/icon/image/new_minimize_hover.svg);\n"
"background-repeat: norepeat;\n"
"background-position: center;\n"
"}\n"
"QPushButton:pressed{\n"
"    background-color: #DFE5EF;\n"
"    color:#8D98AA;\n"
"}")
        self.hide_button.setText("")
        self.hide_button.setIconSize(QtCore.QSize(16, 16))
        self.hide_button.setObjectName("hide_button")
        self.horizontalLayout_5.addWidget(self.hide_button)
        self.close_button = QtWidgets.QPushButton(self.frame_header)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.close_button.sizePolicy().hasHeightForWidth())
        self.close_button.setSizePolicy(sizePolicy)
        self.close_button.setMinimumSize(QtCore.QSize(0, 30))
        self.close_button.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.close_button.setFont(font)
        self.close_button.setStyleSheet("QPushButton{\n"
"border-style:none;\n"
"background-image: url(:/icon/image/new_close.svg);\n"
"background-position: center;\n"
"background-repeat: norepeat;    \n"
"}\n"
"QPushButton:hover{\n"
"    background-color: #F4F6FA;\n"
"background-image: url(:/icon/image/new_close_hover.svg);\n"
"background-repeat: norepeat;\n"
"background-position: center;\n"
"}\n"
"QPushButton:pressed{\n"
"    background-color: #DFE5EF;\n"
"    color:#8D98AA;\n"
"}")
        self.close_button.setText("")
        self.close_button.setObjectName("close_button")
        self.horizontalLayout_5.addWidget(self.close_button)
        self.verticalLayout_2.addWidget(self.frame_header, 0, QtCore.Qt.AlignTop)
        self.frame_form = QtWidgets.QFrame(self.frame_bg)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
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
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.frame_form)
        self.verticalLayout_3.setContentsMargins(9, 0, -1, 0)
        self.verticalLayout_3.setSpacing(6)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.frame_5 = QtWidgets.QFrame(self.frame_form)
        self.frame_5.setMinimumSize(QtCore.QSize(0, 0))
        self.frame_5.setMaximumSize(QtCore.QSize(16777215, 20))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.frame_5.setFont(font)
        self.frame_5.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_5.setObjectName("frame_5")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.frame_5)
        self.horizontalLayout_4.setContentsMargins(9, 0, 0, 0)
        self.horizontalLayout_4.setSpacing(0)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.rb2 = QtWidgets.QRadioButton(self.frame_5)
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        self.rb2.setFont(font)
        self.rb2.setStyleSheet("QRadioButton {color:#4E5865;}\n"
"QRadioButton::indicator{\n"
"    width:12px;\n"
"    height:12px;\n"
"    border-radius:7px;\n"
"    border-style: solid;\n"
"    border-width: 1px;\n"
"    border-color: #8D98AA;\n"
"}\n"
"\n"
"QRadioButton::indicator:checked {\n"
"    background-color:#0077FF;\n"
"    border-color: #0077FF;\n"
"}\n"
"QRadioButton::indicator:unchecked {\n"
"    background-color:rgb(255, 255, 255);\n"
"}")
        self.rb2.setObjectName("rb2")
        self.horizontalLayout_4.addWidget(self.rb2)
        self.verticalLayout_3.addWidget(self.frame_5)
        self.frame_4 = QtWidgets.QFrame(self.frame_form)
        self.frame_4.setMinimumSize(QtCore.QSize(0, 0))
        self.frame_4.setMaximumSize(QtCore.QSize(16777215, 20))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.frame_4.setFont(font)
        self.frame_4.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.frame_4)
        self.horizontalLayout.setContentsMargins(9, 0, 0, 0)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.rb1 = QtWidgets.QRadioButton(self.frame_4)
        self.rb1.setMinimumSize(QtCore.QSize(0, 0))
        self.rb1.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        self.rb1.setFont(font)
        self.rb1.setStyleSheet("QRadioButton {color:#4E5865;}\n"
"QRadioButton::indicator{\n"
"    width:12px;\n"
"    height:12px;\n"
"    border-radius:7px;\n"
"    border-style: solid;\n"
"    border-width: 1px;\n"
"    border-color: #8D98AA;\n"
"}\n"
"QRadioButton::indicator:checked {\n"
"    background-color:#0077FF;\n"
"    border-color: #0077FF;\n"
"}\n"
"QRadioButton::indicator:unchecked {\n"
"    background-color:rgb(255, 255, 255);\n"
"}\n"
"")
        self.rb1.setObjectName("rb1")
        self.horizontalLayout.addWidget(self.rb1)
        self.verticalLayout_3.addWidget(self.frame_4, 0, QtCore.Qt.AlignLeft)
        self.transform = QtWidgets.QFrame(self.frame_form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.transform.sizePolicy().hasHeightForWidth())
        self.transform.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.transform.setFont(font)
        self.transform.setStyleSheet("color: #4E5865;")
        self.transform.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.transform.setFrameShadow(QtWidgets.QFrame.Raised)
        self.transform.setObjectName("transform")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout(self.transform)
        self.horizontalLayout_7.setContentsMargins(36, 0, -1, 0)
        self.horizontalLayout_7.setSpacing(6)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.ds_label = QtWidgets.QLabel(self.transform)
        self.ds_label.setMinimumSize(QtCore.QSize(80, 0))
        self.ds_label.setMaximumSize(QtCore.QSize(80, 16777215))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        self.ds_label.setFont(font)
        self.ds_label.setText("图片文件夹")
        self.ds_label.setObjectName("ds_label")
        self.horizontalLayout_7.addWidget(self.ds_label)
        self.dataset = QtWidgets.QFrame(self.transform)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.dataset.sizePolicy().hasHeightForWidth())
        self.dataset.setSizePolicy(sizePolicy)
        self.dataset.setMinimumSize(QtCore.QSize(0, 38))
        self.dataset.setMaximumSize(QtCore.QSize(16777215, 38))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.dataset.setFont(font)
        self.dataset.setStyleSheet("#dataset {\n"
"border:1px solid #C3CEDF;\n"
"border-radius: 2px;\n"
"} ")
        self.dataset.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.dataset.setFrameShadow(QtWidgets.QFrame.Raised)
        self.dataset.setObjectName("dataset")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout(self.dataset)
        self.horizontalLayout_8.setContentsMargins(9, 0, 9, 0)
        self.horizontalLayout_8.setSpacing(0)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.pic_line = QtWidgets.QLineEdit(self.dataset)
        self.pic_line.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pic_line.sizePolicy().hasHeightForWidth())
        self.pic_line.setSizePolicy(sizePolicy)
        self.pic_line.setMinimumSize(QtCore.QSize(0, 30))
        self.pic_line.setMaximumSize(QtCore.QSize(16777215, 30))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        self.pic_line.setFont(font)
        self.pic_line.setToolTip("")
        self.pic_line.setStatusTip("")
        self.pic_line.setStyleSheet("border: none;\n"
"background: #FEFEFE;\n"
"padding: 0px 5px;\n"
"color:#8D98AA;")
        self.pic_line.setText("")
        self.pic_line.setReadOnly(True)
        self.pic_line.setPlaceholderText("点击选择图片文件夹路径")
        self.pic_line.setObjectName("pic_line")
        self.horizontalLayout_8.addWidget(self.pic_line)
        self.pic_button = QtWidgets.QPushButton(self.dataset)
        self.pic_button.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pic_button.sizePolicy().hasHeightForWidth())
        self.pic_button.setSizePolicy(sizePolicy)
        self.pic_button.setMinimumSize(QtCore.QSize(30, 30))
        self.pic_button.setMaximumSize(QtCore.QSize(30, 30))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        self.pic_button.setFont(font)
        self.pic_button.setToolTip("Press to select dataset path.")
        self.pic_button.setStyleSheet("QPushButton{\n"
"    background: #FEFEFE;\n"
"    color: #ffffff;\n"
"    border-radius: 2px 2px;\n"
"}\n"
"QPushButton:hover{\n"
"    background: #FEFEFE;\n"
"}\n"
"QPushButton:pressed{\n"
"    background: #FEFEFE;\n"
"}")
        self.pic_button.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icon/image/new_more.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pic_button.setIcon(icon)
        self.pic_button.setObjectName("pic_button")
        self.horizontalLayout_8.addWidget(self.pic_button)
        self.horizontalLayout_7.addWidget(self.dataset)
        self.verticalLayout_3.addWidget(self.transform)
        self.frame_2 = QtWidgets.QFrame(self.frame_form)
        self.frame_2.setMinimumSize(QtCore.QSize(0, 40))
        self.frame_2.setMaximumSize(QtCore.QSize(16777215, 40))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.frame_2.setFont(font)
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.frame_2)
        self.horizontalLayout_2.setContentsMargins(36, 0, 9, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.flags = QtWidgets.QLabel(self.frame_2)
        self.flags.setMinimumSize(QtCore.QSize(80, 0))
        self.flags.setMaximumSize(QtCore.QSize(80, 16777215))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        self.flags.setFont(font)
        self.flags.setStyleSheet("color: #4E5865;")
        self.flags.setObjectName("flags")
        self.horizontalLayout_2.addWidget(self.flags)
        self.dataset_2 = QtWidgets.QFrame(self.frame_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.dataset_2.sizePolicy().hasHeightForWidth())
        self.dataset_2.setSizePolicy(sizePolicy)
        self.dataset_2.setMinimumSize(QtCore.QSize(0, 38))
        self.dataset_2.setMaximumSize(QtCore.QSize(16777215, 38))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.dataset_2.setFont(font)
        self.dataset_2.setStyleSheet("#dataset_2 {\n"
"border:1px solid #C3CEDF;\n"
"border-radius: 2px;\n"
"} ")
        self.dataset_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.dataset_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.dataset_2.setObjectName("dataset_2")
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout(self.dataset_2)
        self.horizontalLayout_9.setContentsMargins(9, 0, 9, 0)
        self.horizontalLayout_9.setSpacing(0)
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.flags_line = QtWidgets.QLineEdit(self.dataset_2)
        self.flags_line.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.flags_line.sizePolicy().hasHeightForWidth())
        self.flags_line.setSizePolicy(sizePolicy)
        self.flags_line.setMinimumSize(QtCore.QSize(0, 30))
        self.flags_line.setMaximumSize(QtCore.QSize(16777215, 30))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        self.flags_line.setFont(font)
        self.flags_line.setToolTip("")
        self.flags_line.setStatusTip("")
        self.flags_line.setStyleSheet("border: none;\n"
"background: #FEFEFE;\n"
"padding: 0px 5px;\n"
"color:#8D98AA;\n"
"")
        self.flags_line.setText("")
        self.flags_line.setReadOnly(False)
        self.flags_line.setPlaceholderText("填入标签并用逗号隔开，例如dog,cat")
        self.flags_line.setObjectName("flags_line")
        self.horizontalLayout_9.addWidget(self.flags_line)
        self.horizontalLayout_2.addWidget(self.dataset_2)
        self.verticalLayout_3.addWidget(self.frame_2)
        self.frame_7 = QtWidgets.QFrame(self.frame_form)
        self.frame_7.setMinimumSize(QtCore.QSize(0, 32))
        self.frame_7.setMaximumSize(QtCore.QSize(16777215, 32))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.frame_7.setFont(font)
        self.frame_7.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_7.setObjectName("frame_7")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.frame_7)
        self.horizontalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.ok_button = QtWidgets.QPushButton(self.frame_7)
        self.ok_button.setEnabled(False)
        self.ok_button.setMinimumSize(QtCore.QSize(90, 31))
        self.ok_button.setMaximumSize(QtCore.QSize(90, 31))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        self.ok_button.setFont(font)
        self.ok_button.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.ok_button.setStyleSheet("QPushButton{\n"
"    background: #0077FF;\n"
"    color: #ffffff;\n"
"    border-radius: 3px 3px;\n"
"}\n"
"QPushButton:hover{\n"
"    background: #52A3FF;\n"
"}\n"
"QPushButton:pressed{\n"
"    background: #0077FF;\n"
"}\n"
"QPushButton:disabled{\n"
"    background: #B8D9FF;\n"
"}\n"
"")
        self.ok_button.setObjectName("ok_button")
        self.horizontalLayout_6.addWidget(self.ok_button)
        self.verticalLayout_3.addWidget(self.frame_7)
        self.verticalLayout_2.addWidget(self.frame_form)
        self.verticalLayout.addWidget(self.frame_bg)

        self.retranslateUi(Form_body)
        QtCore.QMetaObject.connectSlotsByName(Form_body)

    def retranslateUi(self, Form_body):
        _translate = QtCore.QCoreApplication.translate
        Form_body.setWindowTitle(_translate("Form_body", "选择数据集标签"))
        self.label_title.setText(_translate("Form_body", "选择数据集标签"))
        self.rb2.setText(_translate("Form_body", "已有数据集"))
        self.rb1.setText(_translate("Form_body", "制作数据集"))
        self.flags.setText(_translate("Form_body", "标签"))
        self.ok_button.setText(_translate("Form_body", "确认"))
