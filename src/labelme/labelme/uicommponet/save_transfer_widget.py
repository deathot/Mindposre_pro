# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'save_transfer_widget.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(300, 50)
        Form.setMinimumSize(QtCore.QSize(300, 50))
        Form.setMaximumSize(QtCore.QSize(300, 50))
        Form.setStyleSheet("")
        self.verticalLayout = QtWidgets.QVBoxLayout(Form)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame = QtWidgets.QFrame(Form)
        self.frame.setStyleSheet("background-color: rgb(244, 246, 250);\n"
"border: none;\n"
"")
        self.frame.setObjectName("frame")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.frame)
        self.horizontalLayout.setContentsMargins(9, 0, 9, 0)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setMaximumSize(QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Light")
        font.setPointSize(9)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.auto_save = QtWidgets.QPushButton(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.auto_save.sizePolicy().hasHeightForWidth())
        self.auto_save.setSizePolicy(sizePolicy)
        self.auto_save.setMinimumSize(QtCore.QSize(50, 0))
        self.auto_save.setMaximumSize(QtCore.QSize(50, 16777215))
        self.auto_save.setSizeIncrement(QtCore.QSize(0, 0))
        self.auto_save.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icon/image/auto_save_open.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.auto_save.setIcon(icon)
        self.auto_save.setIconSize(QtCore.QSize(32, 32))
        self.auto_save.setObjectName("auto_save")
        self.horizontalLayout.addWidget(self.auto_save)
        self.transfer = QtWidgets.QPushButton(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.transfer.sizePolicy().hasHeightForWidth())
        self.transfer.setSizePolicy(sizePolicy)
        self.transfer.setMinimumSize(QtCore.QSize(90, 35))
        self.transfer.setMaximumSize(QtCore.QSize(90, 35))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(9)
        self.transfer.setFont(font)
        self.transfer.setStyleSheet("QPushButton{\n"
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
        self.transfer.setObjectName("transfer")
        self.horizontalLayout.addWidget(self.transfer, 0, QtCore.Qt.AlignRight)
        self.verticalLayout.addWidget(self.frame)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label.setText(_translate("Form", "自动保存标注    "))
        self.transfer.setText(_translate("Form", "一键迁移"))
