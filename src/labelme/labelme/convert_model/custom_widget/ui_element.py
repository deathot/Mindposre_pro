from PyQt5 import QtCore, QtGui
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QVBoxLayout, QLabel, QLineEdit, QSizePolicy
from PyQt5.QtCore import Qt
from src.labelme.labelme.uicommponet import image_rc

class IsEnabledFrame(QtWidgets.QFrame):
    def __init__(self):
        super().__init__()
        self.setMaximumHeight(22)
        self.layout = QHBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        # self.setStyleSheet("border: 1px solid black")
        self.chkbox = QtWidgets.QCheckBox("使用早停策略")
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC")
        font.setPointSize(9)
        self.chkbox.setFont(font)
        self.chkbox.setFixedWidth(90)
        # self.chkbox.setStyleSheet("border: 1px solid black")
        self.chkbox.setSizePolicy(QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding))
        self.info_btn = QtWidgets.QPushButton()
        self.info_btn.setToolTip("当满足下列任一个条件时，训练将自动停止。推荐使用默认参数。")
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(":/icon/help.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.info_btn.setIcon(icon1)
        self.info_btn.setIconSize(QtCore.QSize(16, 16))
        self.layout.addWidget(self.chkbox, Qt.AlignBottom)
        self.layout.addWidget(self.info_btn)
        self.layout.addSpacing(614)


class CriterionFrame(QtWidgets.QFrame):
    def __init__(self, metric: str) -> None:
        super().__init__()
        self.setMaximumHeight(34)
        self.layout = QHBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)

        self.threshold_label = QLabel(f"{metric}达到")
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC")
        font.setBold(False)
        font.setPointSize(9)
        self.threshold_label.setFont(font)
        self.threshold_label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        self.threshold_label.setFixedSize(100, 34)
        self.threshold_lineedit = QLineEdit()
        self.threshold_lineedit.setFixedSize(116, 34)
        self.threshold_lineedit.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        self.threshold_lineedit.setStyleSheet("border:1px solid #C3CEDF; border-radius: 2px;")
        self.threshold_lineedit.setToolTip("取值范围(0, 1)")

        self.tolerance_label = QLabel(f"{metric}连续迭代不上升次数")
        self.tolerance_label.setFont(font)
        self.tolerance_label.setFixedSize(150, 34)
        self.tolerance_label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        # self.tolerance_label.setStyleSheet("border: 1px solid black;")
        self.tolerance_lineedit = QLineEdit()
        self.tolerance_lineedit.setToolTip(f"当{metric}达到该值时停止训练")
        self.tolerance_lineedit.setFixedSize(130, 34)
        self.tolerance_lineedit.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        self.tolerance_lineedit.setStyleSheet("border:1px solid #C3CEDF; border-radius: 2px;")
        self.tolerance_lineedit.setToolTip("取值范围(0, 迭代次数]")
        self.layout.addWidget(self.threshold_label)

        self.layout.addWidget(self.threshold_lineedit)

        self.layout.addSpacing(111)
        self.layout.addWidget(self.tolerance_label)
        self.layout.addWidget(self.tolerance_lineedit)

class EarlyStpPanel(QtWidgets.QFrame):
    def __init__(self, metric: str):
        super().__init__()
        self.setMaximumHeight(54)
        self.is_enabled_frame = IsEnabledFrame()
        self.criterion_frame = CriterionFrame(metric)
        self.criterion_frame.threshold_lineedit.setEnabled(False)
        self.criterion_frame.tolerance_lineedit.setEnabled(False)

        def toggle_editable():
            self.criterion_frame.threshold_lineedit.setEnabled(\
                not self.criterion_frame.threshold_lineedit.isEnabled())
            self.criterion_frame.tolerance_lineedit.setEnabled(\
                not self.criterion_frame.tolerance_lineedit.isEnabled())

        self.is_enabled_frame.chkbox.stateChanged.connect(toggle_editable)
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setContentsMargins(9, 0, 9, 0)
        self.layout.setSpacing(3)
        self.layout.addWidget(self.is_enabled_frame)
        self.layout.addWidget(self.criterion_frame)