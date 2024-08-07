import os.path
import shutil

from PyQt5.QtGui import QDoubleValidator, QIntValidator, QColor
from PyQt5.QtWidgets import QWidget
from qtpy import QtWidgets
from PyQt5 import QtCore, QtGui

from src.labelme.labelme.convert_model.keypoint.convert_thread import ConvertThread
from src.labelme.labelme.run_cmd_utils import is_contains_chinese, check_path
from src.labelme.labelme.convert_model.keypoint.component.keypoint import Ui_frame_body
from src.labelme.labelme.uicommponet.note import Notes
from src.labelme.labelme.uicommponet.yes_cancel_note import YesCancelNote


class ConvertModel(QWidget, Ui_frame_body):

    def __init__(self, model_no):
        super(ConvertModel, self).__init__()
        self.model_no = model_no
        self.ds_data = None
        self.ratio_data = None
        self.test_data = None
        self.epoch_data = None
        self.batch_data = None
        self.pkg_data = None
        self.output_data = None
        self.ret = ""
        self.last_out_path = "C:\\"
        self.last_ds_path = "C:\\"
        self.m_flag = None
        self.m_Position = None
        self.thread = None
        # 隐藏头部
        self.setWindowFlag(QtCore.Qt.FramelessWindowHint)
        # 隐藏边框
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        shadow_effect = QtWidgets.QGraphicsDropShadowEffect(self)
        shadow_effect.setOffset(5, 5)
        shadow_effect.setColor(QColor(0, 0, 0, 26))
        shadow_effect.setBlurRadius(15)
        self.setupUi(self)
        self.frame_bg.setGraphicsEffect(shadow_effect)
        self.translate()
        self.setWindowTitle(self.tr("Keypoint"))

        # 数据转换参数
        self.ds_button.clicked.connect(self.dsMsg)

        self.pDoubleVal = QDoubleValidator(self)
        self.pDoubleVal.setRange(0, 1)
        self.pDoubleVal.setDecimals(1)
        self.pDoubleVal.setNotation(QDoubleValidator.StandardNotation)
        self.split_line.setText("0.3")
        self.split_line.setValidator(self.pDoubleVal)

        # 训练参数
        self.epoch_line_val = QIntValidator(self)
        self.epoch_line_val.setRange(1, 1000)
        self.epoch_line.setText("100")
        self.epoch_line.setValidator(self.epoch_line_val)

        self.batch_line_val = QIntValidator(self)
        self.batch_line_val.setRange(1, 100)
        self.batch_line.setText("12")
        self.batch_line.setValidator(self.batch_line_val)

        # 打包参数
        self.output_button.clicked.connect(self.outputMsg)

        # 早停参数 默认值 以及校验器
        self.earlystp_panel.criterion_frame.threshold_lineedit.setText("0.99")
        self.threshold_validator = QDoubleValidator()
        self.threshold_validator.setRange(0.0, 1.0, 10)
        self.earlystp_panel.criterion_frame.threshold_lineedit.setValidator(
            self.threshold_validator)

        self.earlystp_panel.criterion_frame.tolerance_lineedit.setText("10")
        self.tolerance_validator = QIntValidator(0, 1000)
        self.earlystp_panel.criterion_frame.tolerance_lineedit.setValidator(
            self.threshold_validator)
        # 启动
        self.start_button.clicked.connect(self.pkg_start)

        self.close_button.clicked.connect(self.closeMsg)
        self.hide_button.clicked.connect(self.showMinimized)

    def translate(self):
        self.label_title.setText(self.tr("Keypoint"))
        self.ds_label.setText(self.tr("Dataset Path"))
        self.split_label.setText(self.tr("Dataset Split"))
        self.epoch.setText(self.tr("Epoch"))
        self.batch.setText(self.tr("Batch"))
        self.pkg_label.setText(self.tr("Output Path"))
        self.start_button.setText(self.tr("Transfer"))
        self.ds_button.setToolTip(self.tr("Press to select dataset path."))
        self.output_button.setToolTip(self.tr("Press to select transfer output path."))
        self.start_button.setToolTip(self.tr("Transfer"))
        self.ds_line.setPlaceholderText(self.tr("Press to select dataset path.(No special characters)"))
        self.split_line.setPlaceholderText(self.tr("Enter split ratio"))
        self.epoch_line.setPlaceholderText(self.tr("Epoch"))
        self.batch_line.setPlaceholderText(self.tr("Batch"))
        self.output_line.setPlaceholderText(self.tr("Press to select transfer output path.(No special characters)"))

    def closeMsg(self):
        if self.thread:
            self.close_notes = YesCancelNote(self.tr("WARNING"), self.tr("You are terminating the process. Are you sure to quit?"))
            self.close_notes.yes.clicked.connect(self.thread_terminate)
            self.close_notes.show()
        else:
            self.close()

    def dsMsg(self):
        self.ds_path = QtWidgets.QFileDialog.getExistingDirectory(None, self.tr("Choose Folder"), self.last_ds_path)
        if self.ds_path:
            p = check_path(self.ds_path)
            if is_contains_chinese(self.ds_path):
                self.notes = Notes(self.tr("ERROR"), self.tr("The path does not support Chinese"), "ERROR")
                self.notes.show()
            elif p != "":
                self.notes = Notes(self.tr("ERROR"), self.tr("The path contains illegal character ") + f"'{p}'", "ERROR")
                self.notes.show()
            else:
                self.ds_path = self.ds_path.replace('/', '\\')
                self.last_ds_path = self.ds_path
                self.ds_line.setText(self.ds_path)
                self.ds_line.setToolTip(self.ds_path)

    def outputMsg(self):
        if self.last_ds_path != "C:\\":
            self.last_out_path = os.path.dirname(self.last_ds_path)
        self.output_path = QtWidgets.QFileDialog.getExistingDirectory(None, self.tr("Choose Folder"), self.last_out_path)
        if self.output_path:
            p = check_path(self.output_path)
            if is_contains_chinese(self.output_path):
                self.notes = Notes(self.tr("ERROR"), self.tr("The path does not support Chinese"), "ERROR")
                self.notes.show()
            elif p != "":
                self.notes = Notes(self.tr("ERROR"), self.tr("The path contains illegal character ") + f"'{p}'", "ERROR")
                self.notes.show()
            else:
                self.output_path = self.output_path.replace('/', '\\')
                self.last_out_path = self.output_path
                self.output_path = os.path.join(self.output_path, "output")
                self.output_line.setText(self.output_path)
                self.output_line.setToolTip(self.output_path)

    def recovery_ui(self):
        if self.thread:
            self.thread.terminate()
            self.thread = None
        self.ds_button.setEnabled(True)
        self.output_button.setEnabled(True)
        self.split_line.setEnabled(True)
        self.epoch_line.setEnabled(True)
        self.batch_line.setEnabled(True)
        self.output_line.setEnabled(True)
        self.start_button.setStyleSheet('''
            QPushButton {
                background: #0077FF;
                color: #fefefe;
                border-radius: 3px 3px;
            }
            QPushButton:hover{
                background: #b8d9ff;
                color: #fefefe;
            }
        ''')
        self.start_button.setText(self.tr("Transfer"))
        self.msg_label.setText("")
        self.set_process(0)

    def thread_terminate(self):
        self.recovery_ui()
        if os.path.exists(self.output_data):
            shutil.rmtree(self.output_data)

    def threadEnd(self):
        self.recovery_ui()
        if "INFO" in self.ret:
            self.set_process(100)
            self.notes = Notes(self.tr("INFO"), self.tr("Transfer Successful! Please upload package ") + self.ret.split('INFO: ')[1] + self.tr(" to the development board."), "INFO")
            self.notes.show()
            self.set_process(0)
            self.hide()
        else:
            self.set_process(0)
            self.notes = Notes(self.tr("ERROR"), self.ret, "ERROR")
            self.notes.show()
            if os.path.exists(self.output_data):
                shutil.rmtree(self.output_data)

    def set_ret(self, msg):
        self.ret = self.tr(msg)

    def set_status(self, msg):
        self.msg_label.setText(msg)

    def set_process(self, value):
        self.progressBar.setValue(value)
        self.progressBar.update()
        self.progress_label.setText(f"{value}%")

    def pkg_start(self):
        if self.start_button.text() == self.tr('Transfer'):
            self.ds_data = self.ds_line.text()
            self.ratio_data = str(self.split_line.text())
            self.epoch_data = str(self.epoch_line.text())
            self.batch_data = str(self.batch_line.text())
            self.output_data = self.output_line.text()

            # 是否使用早停
            self.earlystp_enabled = self.earlystp_panel.is_enabled_frame.chkbox.isChecked()

            # 早停所需的参数（todo: 加try-catch?)
            self.earlystp_threshold_fl = float(self.earlystp_panel.criterion_frame.threshold_lineedit.text())
            self.earlystp_tolerance_int = int(self.earlystp_panel.criterion_frame.tolerance_lineedit.text())

            if self.ds_data and self.ratio_data and self.epoch_data and self.batch_data and self.output_data:
                if float(self.ratio_data) <= 0 or float(self.ratio_data) >= 1:
                    self.notes = Notes(self.tr("ERROR"), self.tr("split_ratio out of range (0,1)!"), "ERROR")
                    self.notes.show()
                    return
                if int(self.epoch_data) <= 0:
                    self.notes = Notes(self.tr("ERROR"), self.tr("epochs should bigger than 0!"), "ERROR")
                    self.notes.show()
                    return
                if int(self.batch_data) <= 0:
                    self.notes = Notes(self.tr("ERROR"), self.tr("batch size should bigger than 0!"), "ERROR")
                    self.notes.show()
                    return
                if is_contains_chinese(self.output_data):
                    self.notes = Notes(self.tr("ERROR"), self.tr("The path does not support Chinese"), "ERROR")
                    self.notes.show()
                    return
                p = check_path(self.output_data)
                if p != "":
                    self.notes = Notes(self.tr("ERROR"), self.tr("The path contains illegal character ") + f"'{p}'",
                                       "ERROR")
                    self.notes.show()
                    return
                if os.path.exists(self.output_data):
                    self.notes = Notes(self.tr("ERROR"), self.tr("output already exist!"), "ERROR")
                    self.notes.show()
                    return
                else:
                    os.makedirs(self.output_data)

                # 早停参数的取值范围有Validator校验不了的，这里手工校验
                if self.earlystp_threshold_fl <= 0:
                    self.notes = Notes(self.tr("ERROR"), "不能输入0", "ERROR")
                    self.notes.show()
                    return
                if self.earlystp_threshold_fl >= 1:
                    self.notes = Notes(self.tr("ERROR"), "不能大于等于1", "ERROR")
                    self.notes.show()
                    return
                if self.earlystp_tolerance_int >= int(self.epoch_data):
                    self.notes = Notes(self.tr("ERROR"), "不能超过迭代次数", "ERROR")
                    self.notes.show()
                    return

                self.ds_button.setEnabled(False)
                self.output_button.setEnabled(False)
                self.split_line.setEnabled(False)
                self.epoch_line.setEnabled(False)
                self.batch_line.setEnabled(False)
                self.output_line.setEnabled(False)
                self.start_button.setText(self.tr("Untransfer"))
                self.start_button.setStyleSheet('''
                    QPushButton {
                        background: #fefefe;
                        color: #000000;
                        border: 1px solid #C3CEDF;
                    }
                    QPushButton:hover{
                        background: #b8d9ff;
                        color: #fefefe;
                    }
                ''')
                self.thread = ConvertThread(self.ds_data, self.ratio_data,
                                            int(self.epoch_data), int(self.batch_data), self.output_data,
                                            self.earlystp_enabled,
                                            self.earlystp_threshold_fl, self.earlystp_tolerance_int)
                self.thread.ret_value.connect(self.set_ret)
                self.thread.status_value.connect(self.set_status)
                self.thread.thread_end.connect(self.threadEnd)
                self.thread.process_value.connect(self.set_process)
                self.thread.start()
            else:
                self.notes = Notes(self.tr("ERROR"), self.tr("Parameters are not configured completely!"), "ERROR")
                self.notes.show()
        else:
            self.closeMsg()

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