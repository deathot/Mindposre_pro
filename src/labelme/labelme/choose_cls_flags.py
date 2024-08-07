import os

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QRegExp, Qt
from PyQt5.QtGui import QRegExpValidator, QCursor, QColor
from PyQt5.QtWidgets import QWidget

from src.labelme.labelme.run_cmd_utils import is_contains_chinese, check_path
from src.labelme.labelme.uicommponet.classindex import Ui_Form_body
from src.labelme.labelme.uicommponet.note import Notes

path = os.path.dirname(os.path.abspath(__file__))

ret = None


class ClsFlags(QWidget, Ui_Form_body):
    def __init__(self, model_task_win, model_no, version_data, parent=None):
        super(ClsFlags, self).__init__(parent)
        self.version_data = version_data
        self.model_task_win = model_task_win
        self.model_no = model_no
        self.last_pic_path = "C:/"
        self.pic_path = None

        self.m_flag = None
        self.m_Position = None

        # 隐藏头部
        self.setWindowFlag(Qt.FramelessWindowHint)
        # 隐藏边框
        self.setAttribute(Qt.WA_TranslucentBackground)
        shadow_effect = QtWidgets.QGraphicsDropShadowEffect(self)
        shadow_effect.setOffset(5, 5)
        shadow_effect.setColor(QColor(0, 0, 0, 26))
        shadow_effect.setBlurRadius(15)
        self.setupUi(self)
        self.frame_bg.setGraphicsEffect(shadow_effect)
        self.button_bind_event_and_validator()
        self.component_change_text()
        self.close_button.clicked.connect(self.close)
        self.hide_button.clicked.connect(self.showMinimized)

    def dataMsg(self):
        self.pic_path = QtWidgets.QFileDialog.getExistingDirectory(None, self.tr("Choose Folder"), self.last_pic_path)
        if self.pic_path:
            p = check_path(self.pic_path)
            if is_contains_chinese(self.pic_path):
                self.notes = Notes(self.tr("ERROR"), self.tr("The path does not support Chinese"), "ERROR")
                self.notes.show()
            elif p != "":
                self.notes = Notes(self.tr("ERROR"), self.tr("The path contains illegal character ") + f"'{p}'", "ERROR")
                self.notes.show()
            else:
                self.last_pic_path = self.pic_path
                self.pic_line.setText(self.pic_path)
                self.pic_line.setToolTip(self.pic_path)
        self.pic_path = self.pic_line.text()

    def change_button1(self):
        if not self.rb2.isChecked():
            self.flags_line.setEnabled(True)
            self.pic_button.setEnabled(True)
            self.rb1.setChecked(True)
            self.ok_button.setEnabled(True)
            return
        if self.rb2.isChecked():
            self.flags_line.setEnabled(True)
            self.pic_button.setEnabled(True)
            self.rb2.setChecked(False)
            self.ok_button.setEnabled(True)
        else:
            self.flags_line.setEnabled(False)
            self.pic_button.setEnabled(False)
            self.ok_button.setEnabled(False)

    def change_button2(self):
        if not self.rb1.isChecked():
            self.flags_line.setEnabled(False)
            self.pic_button.setEnabled(False)
            self.rb2.setChecked(True)
            self.ok_button.setEnabled(True)
            return
        if self.rb1.isChecked():
            self.flags_line.setEnabled(False)
            self.pic_button.setEnabled(False)
            self.rb1.setChecked(False)
            self.ok_button.setEnabled(True)
        else:
            self.flags_line.setEnabled(False)
            self.pic_button.setEnabled(False)
            self.ok_button.setEnabled(False)

    def make_dataset(self):
        self.close()
        flags_path = os.path.join(os.path.dirname(self.pic_path), "flags.txt")
        self.model_task_win.main(self.model_no, self.version_data, filename=self.pic_path, flags=flags_path)

    def write_flags(self):
        if not os.path.exists(self.pic_path):
            self.notes = Notes(self.tr("ERROR"), self.tr("Folder path is invalid."), "ERROR")
            self.notes.close_button.clicked.connect(self.notes.close)
            self.notes.button.clicked.connect(self.notes.close)
            self.notes.show()
            return
        if not self.flags_line.text():
            self.notes = Notes(self.tr("ERROR"), self.tr("Flags is empty."), "ERROR")
            self.notes.close_button.clicked.connect(self.notes.close)
            self.notes.button.clicked.connect(self.notes.close)
            self.notes.show()
            return
        f = open(os.path.join(os.path.dirname(self.pic_path), "flags.txt"), mode='w')
        flags = self.flags_line.text().replace(' ', '').strip().strip(',').split(',')
        for flag in flags:
            f.write(flag + '\n')
        self.notes = Notes(self.tr("INFO"),
                           self.tr("flags.txt has been created at the path ") + os.path.dirname(self.pic_path), "INFO")
        self.notes.close_button.clicked.connect(self.make_dataset)
        self.notes.button.clicked.connect(self.make_dataset)
        self.notes.show()
        # return True

    def show_labelme_main(self):
        if self.rb1.isChecked():
            self.write_flags()
        if self.rb2.isChecked():
            self.close()
            self.model_task_win.main(self.model_no, self.version_data)

    def button_bind_event_and_validator(self):
        self.rb1.clicked.connect(self.change_button1)
        self.rb2.clicked.connect(self.change_button2)
        self.ok_button.clicked.connect(self.show_labelme_main)
        reg = QRegExp(r'^([a-zA-Z0-9]+[,]{1})*$')
        validator = QRegExpValidator()
        validator.setRegExp(reg)
        self.flags_line.setValidator(validator)
        self.pic_button.clicked.connect(self.dataMsg)
        self.pic_path = self.pic_line.text()

    def component_change_text(self):
        self.label_title.setText(self.tr('Choose Classification Flags'))
        self.ds_label.setText(self.tr("Folder:"))
        self.rb1.setText(self.tr("Make dataset"))
        self.rb2.setText(self.tr("I have dataset already"))
        self.ok_button.setText(self.tr("OK"))
        self.flags_line.setPlaceholderText(self.tr("Enter flags separated by commas like dog,cat"))
        self.pic_line.setPlaceholderText(self.tr("Please choose folder with pictures to make dataset."))
        self.flags.setText(self.tr("Flags:"))

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and self.isMaximized() == False:
            self.m_flag = True
            self.m_Position = event.globalPos() - self.pos()  # 获取鼠标相对窗口的位置
            event.accept()
            self.setCursor(QCursor(QtCore.Qt.OpenHandCursor))  # 更改鼠标图标

    def mouseMoveEvent(self, mouse_event):
        if QtCore.Qt.LeftButton and self.m_flag:
            self.move(mouse_event.globalPos() - self.m_Position)  # 更改窗口位置
            mouse_event.accept()

    def mouseReleaseEvent(self, mouse_event):
        self.m_flag = False
        self.setCursor(QCursor(QtCore.Qt.ArrowCursor))
