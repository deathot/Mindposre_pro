# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'global_widget.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(300, 62)
        Form.setMinimumSize(QtCore.QSize(300, 62))
        Form.setMaximumSize(QtCore.QSize(300, 62))
        Form.setStyleSheet("")
        self.verticalLayout = QtWidgets.QVBoxLayout(Form)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame = QtWidgets.QFrame(Form)
        self.frame.setMinimumSize(QtCore.QSize(0, 62))
        self.frame.setStyleSheet("background-color: #EBEFF6;\n"
"border-bottom: 2px solid #DFE5EF;\n"
"")
        self.frame.setObjectName("frame")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.frame)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.open_folder = QtWidgets.QToolButton(self.frame)
        self.open_folder.setMinimumSize(QtCore.QSize(60, 60))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Light")
        font.setPointSize(9)
        self.open_folder.setFont(font)
        self.open_folder.setStyleSheet("QToolButton{\n"
"border-style:none;\n"
"}\n"
"QToolButton:hover{\n"
"    background-color: #C3CEDF;\n"
"}\n"
"QToolButton:pressed{\n"
"    background-color: #C3CEDF;\n"
"}")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icon/image/open_dir.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.open_folder.setIcon(icon)
        self.open_folder.setIconSize(QtCore.QSize(24, 24))
        self.open_folder.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.open_folder.setObjectName("open_folder")
        self.horizontalLayout.addWidget(self.open_folder, 0, QtCore.Qt.AlignTop)
        self.open_pic = QtWidgets.QToolButton(self.frame)
        self.open_pic.setMinimumSize(QtCore.QSize(60, 60))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Light")
        font.setPointSize(9)
        self.open_pic.setFont(font)
        self.open_pic.setStyleSheet("QToolButton{\n"
"border-style:none;\n"
"}\n"
"QToolButton:hover{\n"
"    background-color: #C3CEDF;\n"
"}\n"
"QToolButton:pressed{\n"
"    background-color: #C3CEDF;\n"
"}")
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(":/icon/image/open.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.open_pic.setIcon(icon1)
        self.open_pic.setIconSize(QtCore.QSize(24, 24))
        self.open_pic.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.open_pic.setObjectName("open_pic")
        self.horizontalLayout.addWidget(self.open_pic, 0, QtCore.Qt.AlignTop)
        self.save_file = QtWidgets.QToolButton(self.frame)
        self.save_file.setMinimumSize(QtCore.QSize(60, 60))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Light")
        font.setPointSize(9)
        self.save_file.setFont(font)
        self.save_file.setStyleSheet("QToolButton{\n"
"border-style:none;\n"
"}\n"
"QToolButton:hover{\n"
"    background-color: #C3CEDF;\n"
"}\n"
"QToolButton:pressed{\n"
"    background-color: #C3CEDF;\n"
"}")
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(":/icon/image/save.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.save_file.setIcon(icon2)
        self.save_file.setIconSize(QtCore.QSize(24, 24))
        self.save_file.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.save_file.setObjectName("save_file")
        self.horizontalLayout.addWidget(self.save_file, 0, QtCore.Qt.AlignTop)
        self.delete_file = QtWidgets.QToolButton(self.frame)
        self.delete_file.setMinimumSize(QtCore.QSize(60, 60))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Light")
        font.setPointSize(9)
        self.delete_file.setFont(font)
        self.delete_file.setStyleSheet("QToolButton{\n"
"border-style:none;\n"
"}\n"
"QToolButton:hover{\n"
"    background-color: #C3CEDF;\n"
"}\n"
"QToolButton:pressed{\n"
"    background-color: #C3CEDF;\n"
"}")
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap(":/icon/image/delete_file.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.delete_file.setIcon(icon3)
        self.delete_file.setIconSize(QtCore.QSize(24, 24))
        self.delete_file.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.delete_file.setObjectName("delete_file")
        self.horizontalLayout.addWidget(self.delete_file, 0, QtCore.Qt.AlignTop)
        self.verticalLayout.addWidget(self.frame)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.open_folder.setText(_translate("Form", "打开目录"))
        self.open_pic.setText(_translate("Form", "打开图片"))
        self.save_file.setText(_translate("Form", "保存文件"))
        self.delete_file.setText(_translate("Form", "删除文件"))
