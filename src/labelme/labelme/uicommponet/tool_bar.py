# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'tool_bar.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(965, 50)
        Form.setMinimumSize(QtCore.QSize(0, 50))
        Form.setMaximumSize(QtCore.QSize(16777215, 50))
        Form.setStyleSheet("")
        self.verticalLayout = QtWidgets.QVBoxLayout(Form)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame = QtWidgets.QFrame(Form)
        self.frame.setEnabled(True)
        self.frame.setMinimumSize(QtCore.QSize(0, 50))
        self.frame.setMaximumSize(QtCore.QSize(16777215, 50))
        self.frame.setStyleSheet("background-color: #F4F6FA;\n"
"border-bottom: 1px solid #DFE5EF;\n"
"border-top: 1px solid #DFE5EF;\n"
"")
        self.frame.setObjectName("frame")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.frame)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.frame_3 = QtWidgets.QFrame(self.frame)
        self.frame_3.setMinimumSize(QtCore.QSize(400, 50))
        self.frame_3.setMaximumSize(QtCore.QSize(400, 16777215))
        self.frame_3.setStyleSheet("border-style:none;")
        self.frame_3.setObjectName("frame_3")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.frame_3)
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_3.setSpacing(0)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_3 = QtWidgets.QLabel(self.frame_3)
        self.label_3.setEnabled(False)
        self.label_3.setMinimumSize(QtCore.QSize(300, 0))
        self.label_3.setMaximumSize(QtCore.QSize(300, 16777215))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(12)
        self.label_3.setFont(font)
        self.label_3.setStyleSheet("color:#4E5865;\n"
"background: #EBEFF6;\n"
"border-right: 1px solid #DFE5EF")
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_3.addWidget(self.label_3)
        self.label_4 = QtWidgets.QLabel(self.frame_3)
        self.label_4.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_4.sizePolicy().hasHeightForWidth())
        self.label_4.setSizePolicy(sizePolicy)
        self.label_4.setMinimumSize(QtCore.QSize(100, 0))
        self.label_4.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(11)
        self.label_4.setFont(font)
        self.label_4.setStyleSheet("color:#4E5865;\n"
"border-style:none;\n"
"margin: 10px 10px;")
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_3.addWidget(self.label_4)
        self.horizontalLayout.addWidget(self.frame_3)
        self.frame_2 = QtWidgets.QFrame(self.frame)
        self.frame_2.setMinimumSize(QtCore.QSize(0, 50))
        self.frame_2.setStyleSheet("border-style:none;")
        self.frame_2.setObjectName("frame_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.frame_2)
        self.horizontalLayout_2.setContentsMargins(12, 0, 0, 0)
        self.horizontalLayout_2.setSpacing(15)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.rectangle = QtWidgets.QPushButton(self.frame_2)
        self.rectangle.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.rectangle.sizePolicy().hasHeightForWidth())
        self.rectangle.setSizePolicy(sizePolicy)
        self.rectangle.setMinimumSize(QtCore.QSize(32, 32))
        self.rectangle.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.rectangle.setStyleSheet("QPushButton{\n"
"border-style:none;\n"
"background-image: url(:/tool_button/tool_button/rectangle.svg);\n"
"background-position: center;\n"
"background-repeat: norepeat;    \n"
"}\n"
"QPushButton:hover{\n"
"background-color: #DFE5EF;\n"
"background-image: url(:/tool_button/tool_button/rectangle.svg);\n"
"background-repeat: norepeat;\n"
"background-position: center;\n"
"}\n"
"QPushButton:pressed{\n"
"    background-image: url(:/tool_button/tool_button/rectangle_press.svg);\n"
"}\n"
"QPushButton:!enabled{\n"
"    background-image: url(:/tool_button/tool_button/rectangle_disable.svg);\n"
"}\n"
"QPushButton:checked{\n"
"    background-image: url(:/tool_button/tool_button/rectangle_active.svg);\n"
"}")
        self.rectangle.setText("")
        self.rectangle.setCheckable(True)
        self.rectangle.setChecked(False)
        self.rectangle.setObjectName("rectangle")
        self.horizontalLayout_2.addWidget(self.rectangle)
        self.polygon = QtWidgets.QPushButton(self.frame_2)
        self.polygon.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.polygon.sizePolicy().hasHeightForWidth())
        self.polygon.setSizePolicy(sizePolicy)
        self.polygon.setMinimumSize(QtCore.QSize(32, 32))
        self.polygon.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.polygon.setStyleSheet("QPushButton{\n"
"border-style:none;\n"
"background-image: url(:/tool_button/tool_button/polygon.svg);\n"
"background-position: center;\n"
"background-repeat: norepeat;    \n"
"}\n"
"QPushButton:hover{\n"
"background-color: #DFE5EF;\n"
"background-image: url(:/tool_button/tool_button/polygon.svg);\n"
"background-repeat: norepeat;\n"
"background-position: center;\n"
"}\n"
"QPushButton:pressed{\n"
"    background-image: url(:/tool_button/tool_button/polygon_press.svg);\n"
"}\n"
"QPushButton:!enabled{\n"
"    background-image: url(:/tool_button/tool_button/polygon_disable.svg);\n"
"}\n"
"QPushButton:checked{\n"
"    background-image: url(:/tool_button/tool_button/polygon_active.svg);\n"
"}")
        self.polygon.setText("")
        self.polygon.setCheckable(True)
        self.polygon.setChecked(False)
        self.polygon.setObjectName("polygon")
        self.horizontalLayout_2.addWidget(self.polygon)
        self.point = QtWidgets.QPushButton(self.frame_2)
        self.point.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.point.sizePolicy().hasHeightForWidth())
        self.point.setSizePolicy(sizePolicy)
        self.point.setMinimumSize(QtCore.QSize(32, 32))
        self.point.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.point.setStyleSheet("QPushButton{\n"
"border-style:none;\n"
"background-image: url(:/tool_button/tool_button/point.svg);\n"
"background-position: center;\n"
"background-repeat: norepeat;    \n"
"}\n"
"QPushButton:hover{\n"
"background-color: #DFE5EF;\n"
"background-image: url(:/tool_button/tool_button/point.svg);\n"
"background-repeat: norepeat;\n"
"background-position: center;\n"
"}\n"
"QPushButton:pressed{\n"
"    background-image: url(:/tool_button/tool_button/point_press.svg);\n"
"}\n"
"QPushButton:!enabled{\n"
"    background-image: url(:/tool_button/tool_button/point_disable.svg);\n"
"}\n"
"QPushButton:checked{\n"
"    background-image: url(:/tool_button/tool_button/point_active.svg);\n"
"}")
        self.point.setText("")
        self.point.setCheckable(True)
        self.point.setChecked(False)
        self.point.setObjectName("point")
        self.horizontalLayout_2.addWidget(self.point)
        self.edit = QtWidgets.QPushButton(self.frame_2)
        self.edit.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.edit.sizePolicy().hasHeightForWidth())
        self.edit.setSizePolicy(sizePolicy)
        self.edit.setMinimumSize(QtCore.QSize(32, 32))
        self.edit.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.edit.setStyleSheet("QPushButton{\n"
"border-style:none;\n"
"background-image: url(:/tool_button/tool_button/edit.svg);\n"
"background-position: center;\n"
"background-repeat: norepeat;    \n"
"}\n"
"QPushButton:hover{\n"
"background-color: #DFE5EF;\n"
"background-image: url(:/tool_button/tool_button/edit.svg);\n"
"background-repeat: norepeat;\n"
"background-position: center;\n"
"}\n"
"QPushButton:pressed{\n"
"    background-image: url(:/tool_button/tool_button/edit_press.svg);\n"
"}\n"
"QPushButton:!enabled{\n"
"    background-image: url(:/tool_button/tool_button/edit_disable.svg);\n"
"}\n"
"QPushButton:checked{\n"
"    background-image: url(:/tool_button/tool_button/edit_active.svg);\n"
"}")
        self.edit.setText("")
        self.edit.setCheckable(True)
        self.edit.setChecked(False)
        self.edit.setObjectName("edit")
        self.horizontalLayout_2.addWidget(self.edit)
        self.copy = QtWidgets.QPushButton(self.frame_2)
        self.copy.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.copy.sizePolicy().hasHeightForWidth())
        self.copy.setSizePolicy(sizePolicy)
        self.copy.setMinimumSize(QtCore.QSize(32, 32))
        self.copy.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.copy.setStyleSheet("QPushButton{\n"
"border-style:none;\n"
"background-image: url(:/tool_button/tool_button/copy.svg);\n"
"background-position: center;\n"
"background-repeat: norepeat;    \n"
"}\n"
"QPushButton:hover{\n"
"background-color: #DFE5EF;\n"
"background-image: url(:/tool_button/tool_button/copy.svg);\n"
"background-repeat: norepeat;\n"
"background-position: center;\n"
"}\n"
"QPushButton:pressed{\n"
"    background-image: url(:/tool_button/tool_button/copy_press.svg);\n"
"}\n"
"QPushButton:!enabled{\n"
"    background-image: url(:/tool_button/tool_button/copy_disable.svg);\n"
"}")
        self.copy.setText("")
        self.copy.setCheckable(True)
        self.copy.setChecked(False)
        self.copy.setObjectName("copy")
        self.horizontalLayout_2.addWidget(self.copy)
        self.delete_2 = QtWidgets.QPushButton(self.frame_2)
        self.delete_2.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.delete_2.sizePolicy().hasHeightForWidth())
        self.delete_2.setSizePolicy(sizePolicy)
        self.delete_2.setMinimumSize(QtCore.QSize(32, 32))
        self.delete_2.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.delete_2.setStyleSheet("QPushButton{\n"
"border-style:none;\n"
"background-image: url(:/tool_button/tool_button/delete.svg);\n"
"background-position: center;\n"
"background-repeat: norepeat;    \n"
"}\n"
"QPushButton:hover{\n"
"background-color: #DFE5EF;\n"
"background-image: url(:/tool_button/tool_button/delete.svg);\n"
"background-repeat: norepeat;\n"
"background-position: center;\n"
"}\n"
"QPushButton:pressed{\n"
"    background-image: url(:/tool_button/tool_button/delete_press.svg);\n"
"}\n"
"QPushButton:!enabled{\n"
"    background-image: url(:/tool_button/tool_button/delete_disable.svg);\n"
"}")
        self.delete_2.setText("")
        self.delete_2.setCheckable(True)
        self.delete_2.setChecked(False)
        self.delete_2.setObjectName("delete_2")
        self.horizontalLayout_2.addWidget(self.delete_2)
        self.color = QtWidgets.QPushButton(self.frame_2)
        self.color.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.color.sizePolicy().hasHeightForWidth())
        self.color.setSizePolicy(sizePolicy)
        self.color.setMinimumSize(QtCore.QSize(32, 32))
        self.color.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.color.setStyleSheet("QPushButton{\n"
"border-style:none;\n"
"background-image: url(:/tool_button/tool_button/color.svg);\n"
"background-position: center;\n"
"background-repeat: norepeat;    \n"
"}\n"
"QPushButton:hover{\n"
"background-color: #DFE5EF;\n"
"background-image: url(:/tool_button/tool_button/color.svg);\n"
"background-repeat: norepeat;\n"
"background-position: center;\n"
"}\n"
"QPushButton:pressed{\n"
"    background-image: url(:/tool_button/tool_button/color_press.svg);\n"
"}\n"
"QPushButton:!enabled{\n"
"    background-image: url(:/tool_button/tool_button/color_disable.svg);\n"
"}")
        self.color.setText("")
        self.color.setCheckable(True)
        self.color.setChecked(False)
        self.color.setObjectName("color")
        self.horizontalLayout_2.addWidget(self.color)
        self.undo = QtWidgets.QPushButton(self.frame_2)
        self.undo.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.undo.sizePolicy().hasHeightForWidth())
        self.undo.setSizePolicy(sizePolicy)
        self.undo.setMinimumSize(QtCore.QSize(32, 32))
        self.undo.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.undo.setStyleSheet("QPushButton{\n"
"border-style:none;\n"
"background-image: url(:/tool_button/tool_button/undo.svg);\n"
"background-position: center;\n"
"background-repeat: norepeat;    \n"
"}\n"
"QPushButton:hover{\n"
"background-color: #DFE5EF;\n"
"background-image: url(:/tool_button/tool_button/undo.svg);\n"
"background-repeat: norepeat;\n"
"background-position: center;\n"
"}\n"
"QPushButton:pressed{\n"
"    background-image: url(:/tool_button/tool_button/undo_press.svg);\n"
"}\n"
"QPushButton:!enabled{\n"
"    background-image: url(:/tool_button/tool_button/undo_disable.svg);\n"
"}")
        self.undo.setText("")
        self.undo.setCheckable(True)
        self.undo.setChecked(False)
        self.undo.setObjectName("undo")
        self.horizontalLayout_2.addWidget(self.undo)
        self.prev = QtWidgets.QPushButton(self.frame_2)
        self.prev.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.prev.sizePolicy().hasHeightForWidth())
        self.prev.setSizePolicy(sizePolicy)
        self.prev.setMinimumSize(QtCore.QSize(32, 32))
        self.prev.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.prev.setStyleSheet("QPushButton{\n"
"border-style:none;\n"
"background-image: url(:/tool_button/tool_button/prev.svg);\n"
"background-position: center;\n"
"background-repeat: norepeat;    \n"
"}\n"
"QPushButton:hover{\n"
"background-color: #DFE5EF;\n"
"background-image: url(:/tool_button/tool_button/prev.svg);\n"
"background-repeat: norepeat;\n"
"background-position: center;\n"
"}\n"
"QPushButton:pressed{\n"
"    background-image: url(:/tool_button/tool_button/prev_press.svg);\n"
"}\n"
"QPushButton:!enabled{\n"
"    background-image: url(:/tool_button/tool_button/prev_disable.svg);\n"
"}")
        self.prev.setText("")
        self.prev.setCheckable(True)
        self.prev.setChecked(False)
        self.prev.setObjectName("prev")
        self.horizontalLayout_2.addWidget(self.prev)
        self.next = QtWidgets.QPushButton(self.frame_2)
        self.next.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.next.sizePolicy().hasHeightForWidth())
        self.next.setSizePolicy(sizePolicy)
        self.next.setMinimumSize(QtCore.QSize(32, 32))
        self.next.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.next.setStyleSheet("QPushButton{\n"
"border-style:none;\n"
"background-image: url(:/tool_button/tool_button/next.svg);\n"
"background-position: center;\n"
"background-repeat: norepeat;    \n"
"}\n"
"QPushButton:hover{\n"
"background-color: #DFE5EF;\n"
"background-image: url(:/tool_button/tool_button/next.svg);\n"
"background-repeat: norepeat;\n"
"background-position: center;\n"
"}\n"
"QPushButton:pressed{\n"
"    background-image: url(:/tool_button/tool_button/next_press.svg);\n"
"}\n"
"QPushButton:!enabled{\n"
"    background-image: url(:/tool_button/tool_button/next_disable.svg);\n"
"}")
        self.next.setText("")
        self.next.setCheckable(True)
        self.next.setChecked(False)
        self.next.setObjectName("next")
        self.horizontalLayout_2.addWidget(self.next)
        self.zoom = QtWidgets.QSpinBox(self.frame_2)
        self.zoom.setEnabled(False)
        self.zoom.setMinimumSize(QtCore.QSize(60, 24))
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(10)
        self.zoom.setFont(font)
        self.zoom.setStyleSheet("color:#8D98AA;\n"
"background: #F4F6FA;\n"
"border: 1px solid #8D98AA;")
        self.zoom.setAlignment(QtCore.Qt.AlignCenter)
        self.zoom.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.zoom.setPrefix("")
        self.zoom.setMinimum(1)
        self.zoom.setMaximum(1000)
        self.zoom.setProperty("value", 100)
        self.zoom.setObjectName("zoom")
        self.horizontalLayout_2.addWidget(self.zoom)
        self.horizontalLayout.addWidget(self.frame_2, 0, QtCore.Qt.AlignLeft)
        self.verticalLayout.addWidget(self.frame)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label_3.setText(_translate("Form", "全局操作"))
        self.label_4.setText(Form.model_no)
        self.rectangle.setToolTip(_translate("Form", "绘制矩形"))
        self.polygon.setToolTip(_translate("Form", "绘制多边形"))
        self.point.setToolTip(_translate("Form", "绘制控制点"))
        self.edit.setToolTip(_translate("Form", "编辑多边形"))
        self.copy.setToolTip(_translate("Form", "生成副本"))
        self.delete_2.setToolTip(_translate("Form", "删除多边形"))
        self.color.setToolTip(_translate("Form", "亮度与对比度"))
        self.undo.setToolTip(_translate("Form", "撤销上一次操作"))
        self.prev.setToolTip(_translate("Form", "上一张"))
        self.next.setToolTip(_translate("Form", "下一张"))
        self.zoom.setToolTip(_translate("Form", "显示比例"))
        self.zoom.setSuffix(_translate("Form", " %"))
