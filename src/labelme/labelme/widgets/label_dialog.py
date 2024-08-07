import re

from PyQt5.QtCore import Qt, QSize
from qtpy import QT_VERSION
from qtpy import QtCore
from qtpy import QtGui
from qtpy import QtWidgets

from src.labelme.labelme.logger import logger
import src.labelme.labelme.utils


QT5 = QT_VERSION[0] == "5"


# TODO(unknown):
# - Calculate optimal position so as not to go out of screen area.


class LabelQLineEdit(QtWidgets.QLineEdit):
    def setListWidget(self, list_widget):
        self.list_widget = list_widget

    def keyPressEvent(self, e):
        if e.key() in [QtCore.Qt.Key_Up, QtCore.Qt.Key_Down]:
            self.list_widget.keyPressEvent(e)
        else:
            super(LabelQLineEdit, self).keyPressEvent(e)


class LabelDialog(QtWidgets.QDialog):
    def __init__(
        self,
        text="Enter object label",
        parent=None,
        labels=None,
        sort_labels=True,
        show_text_field=True,
        completion="startswith",
        fit_to_content=None,
        flags=None,
    ):
        self.m_flag = None
        self.m_Position = None
        self.Margins = 5
        if fit_to_content is None:
            fit_to_content = {"row": False, "column": True}
        self._fit_to_content = fit_to_content

        super(LabelDialog, self).__init__(parent)
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(9)
        font.setBold(False)
        font.setWeight(50)
        self.setMouseTracking(True)
        self.setWindowFlag(Qt.FramelessWindowHint)
        # self.setContentsMargins(12, 0, 12, 0)
        self.edit = LabelQLineEdit()
        self.edit.setStyleSheet("width:240px;\nheight:32px;\nbackground-color: #FEFEFE;\nborder: 1px solid #C3CEDF;\nborder-radius: 2px;")
        self.edit.setFont(font)
        self.edit.setPlaceholderText(self.tr("Enter object label"))
        self.edit.setValidator(src.labelme.labelme.utils.labelValidator())
        self.edit.editingFinished.connect(self.postProcess)
        if flags:
            self.edit.textChanged.connect(self.updateFlags)
        self.edit_group_id = QtWidgets.QLineEdit()
        self.edit_group_id.setStyleSheet("width:100px;\nheight:32px;\nbackground-color: #FEFEFE;\nborder: 1px solid #C3CEDF;\nborder-radius: 2px;")
        self.edit_group_id.setFont(font)
        self.edit_group_id.setPlaceholderText(self.tr("Group ID"))
        self.edit_group_id.setValidator(
            QtGui.QRegExpValidator(QtCore.QRegExp(r"\d*"), None)
        )
        layout = QtWidgets.QVBoxLayout()
        # title
        title_layout = QtWidgets.QHBoxLayout()
        title_label = QtWidgets.QLabel(self.tr("Add Label"))
        titel_font = QtGui.QFont()
        titel_font.setFamily("HarmonyOS Sans SC Medium")
        titel_font.setPointSize(12)
        titel_font.setBold(False)
        titel_font.setWeight(50)
        title_label.setFont(titel_font)
        title_label.setFixedHeight(36)
        title_label.setStyleSheet("border-style: none")
        title_button = QtWidgets.QPushButton()
        title_button.clicked.connect(self.close)
        title_button.setFixedSize(32, 32)
        title_button_qss = '''
            QPushButton{
                border-style:none;
            }
            QPushButton:hover{
                background-color: #DFE5EF;
            }
            QPushButton:pressed{
                background-color: #C3CEDF;
                color:#8D98AA;
            }
        '''
        title_button.setStyleSheet(title_button_qss)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icon/image/new_close.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        title_button.setIcon(icon)
        title_button.setIconSize(QSize(32, 32))
        title_layout.addWidget(title_label)
        title_layout.addWidget(title_button)
        layout.addLayout(title_layout)
        if show_text_field:
            layout_edit = QtWidgets.QHBoxLayout()
            layout_edit.addWidget(self.edit)
            layout_edit.addWidget(self.edit_group_id)
            layout.addLayout(layout_edit)
        # buttons
        layout_ok_cancel = QtWidgets.QHBoxLayout()
        ok_button = QtWidgets.QPushButton(self.tr("OK"))
        cancle_button = QtWidgets.QPushButton(self.tr("Cancel"))
        ok_qss = '''
            QPushButton{
                background-color: #0077FF;
                color: #ffffff;
                border: 1px solid #C3CEDF;
                border-radius: 2px;
            }
            QPushButton:hover{
                background-color: #52A3FF;
            }
            QPushButton:pressed{
                background-color: #0077FF;
            }
        '''
        cancle_qss = '''
            QPushButton{
                background-color: #FEFEFE;
                color: #4E5865;
                border: 1px solid #C3CEDF;
                border-radius: 2px;
            }
            QPushButton:hover{
                background-color: #FEFEFE;
                color: #52A3FF;
                border: 1px solid #52A3FF;
            }
            QPushButton:pressed{
                background-color: #FEFEFE;
                color: #4E5865;
                border: 1px solid #C3CEDF;
            }
        '''
        ok_button.setFixedSize(80, 32)
        cancle_button.setFixedSize(80, 32)
        ok_button.setStyleSheet(ok_qss)
        cancle_button.setStyleSheet(cancle_qss)
        ok_button.setFont(font)
        cancle_button.setFont(font)
        ok_button.clicked.connect(self.validate)
        cancle_button.clicked.connect(self.reject)
        layout_ok_cancel.addWidget(ok_button)
        layout_ok_cancel.addWidget(cancle_button)
        # label_list
        self.labelList = QtWidgets.QListWidget()
        self.labelList.setFont(font)
        labellist_qss = '''
            QListWidget
            {
                background: #FEFEFE;
                color: #000000;
            }
            QListWidget::Item
            {
                height: 32px;
                background: #FEFEFE;
                color: #000000;
            }
            QListWidget::Item:hover
            {
                height: 32px;
                background: #52A3FF;
                color: #000000;
            }
            QListWidget::item:selected
            {
                height: 32px;
                background: #0077FF;
                color: #FFFFFF;
            }
        '''
        self.labelList.setStyleSheet(labellist_qss)
        self.setStyleSheet("background: #FEFEFE;\nborder: 1px solid #C3CEDF")
        if self._fit_to_content["row"]:
            self.labelList.setHorizontalScrollBarPolicy(
                QtCore.Qt.ScrollBarAlwaysOff
            )
        if self._fit_to_content["column"]:
            self.labelList.setVerticalScrollBarPolicy(
                QtCore.Qt.ScrollBarAlwaysOff
            )
        self._sort_labels = sort_labels
        if labels:
            self.labelList.addItems(labels)
        if self._sort_labels:
            self.labelList.sortItems()
        else:
            self.labelList.setDragDropMode(
                QtWidgets.QAbstractItemView.InternalMove
            )
        self.labelList.currentItemChanged.connect(self.labelSelected)
        self.labelList.itemDoubleClicked.connect(self.labelDoubleClicked)
        self.edit.setListWidget(self.labelList)
        layout.addWidget(self.labelList)
        layout.addLayout(layout_ok_cancel)
        # label_flags
        if flags is None:
            flags = {}
        self._flags = flags
        self.flagsLayout = QtWidgets.QVBoxLayout()
        self.resetFlags()
        layout.addItem(self.flagsLayout)
        self.edit.textChanged.connect(self.updateFlags)
        self.setLayout(layout)
        # completion
        completer = QtWidgets.QCompleter()
        if not QT5 and completion != "startswith":
            logger.warn(
                "completion other than 'startswith' is only "
                "supported with Qt5. Using 'startswith'"
            )
            completion = "startswith"
        if completion == "startswith":
            completer.setCompletionMode(QtWidgets.QCompleter.InlineCompletion)
            # Default settings.
            # completer.setFilterMode(QtCore.Qt.MatchStartsWith)
        elif completion == "contains":
            completer.setCompletionMode(QtWidgets.QCompleter.PopupCompletion)
            completer.setFilterMode(QtCore.Qt.MatchContains)
        else:
            raise ValueError("Unsupported completion: {}".format(completion))
        completer.setModel(self.labelList.model())
        self.edit.setCompleter(completer)

    def addLabelHistory(self, label):
        if self.labelList.findItems(label, QtCore.Qt.MatchExactly):
            return
        self.labelList.addItem(label)
        if self._sort_labels:
            self.labelList.sortItems()

    def labelSelected(self, item):
        self.edit.setText(item.text())

    def validate(self):
        text = self.edit.text()
        if hasattr(text, "strip"):
            text = text.strip()
        else:
            text = text.trimmed()
        if text:
            self.accept()

    def labelDoubleClicked(self, item):
        self.validate()

    def postProcess(self):
        text = self.edit.text()
        if hasattr(text, "strip"):
            text = text.strip()
        else:
            text = text.trimmed()
        self.edit.setText(text)

    def updateFlags(self, label_new):
        # keep state of shared flags
        flags_old = self.getFlags()

        flags_new = {}
        for pattern, keys in self._flags.items():
            if re.match(pattern, label_new):
                for key in keys:
                    flags_new[key] = flags_old.get(key, False)
        self.setFlags(flags_new)

    def deleteFlags(self):
        for i in reversed(range(self.flagsLayout.count())):
            item = self.flagsLayout.itemAt(i).widget()
            self.flagsLayout.removeWidget(item)
            item.setParent(None)

    def resetFlags(self, label=""):
        flags = {}
        for pattern, keys in self._flags.items():
            if re.match(pattern, label):
                for key in keys:
                    flags[key] = False
        self.setFlags(flags)

    def setFlags(self, flags):
        self.deleteFlags()
        for key in flags:
            item = QtWidgets.QCheckBox(key, self)
            item.setChecked(flags[key])
            self.flagsLayout.addWidget(item)
            item.show()

    def getFlags(self):
        flags = {}
        for i in range(self.flagsLayout.count()):
            item = self.flagsLayout.itemAt(i).widget()
            flags[item.text()] = item.isChecked()
        return flags

    def getGroupId(self):
        group_id = self.edit_group_id.text()
        if group_id:
            return int(group_id)
        return None

    def popUp(self, text=None, move=True, flags=None, group_id=None):
        if self._fit_to_content["row"]:
            self.labelList.setMinimumHeight(
                self.labelList.sizeHintForRow(0) * self.labelList.count() + 2
            )
        if self._fit_to_content["column"]:
            self.labelList.setMinimumWidth(
                self.labelList.sizeHintForColumn(0) + 2
            )
        # if text is None, the previous label in self.edit is kept
        if text is None:
            text = self.edit.text()
        if flags:
            self.setFlags(flags)
        else:
            self.resetFlags(text)
        self.edit.setText(text)
        self.edit.setSelection(0, len(text))
        if group_id is None:
            self.edit_group_id.clear()
        else:
            self.edit_group_id.setText(str(group_id))
        items = self.labelList.findItems(text, QtCore.Qt.MatchFixedString)
        if items:
            if len(items) != 1:
                logger.warning("Label list has duplicate '{}'".format(text))
            self.labelList.setCurrentItem(items[0])
            row = self.labelList.row(items[0])
            self.edit.completer().setCurrentRow(row)
        self.edit.setFocus(QtCore.Qt.PopupFocusReason)
        if self.exec_():
            return self.edit.text(), self.getFlags(), self.getGroupId()
        else:
            return None, None, None

    def move(self, pos):
        if self.windowState() == Qt.WindowMaximized or self.windowState() == Qt.WindowFullScreen:
            return
        super(LabelDialog, self).move(pos)

    def mousePressEvent(self, event):
        """鼠标点击事件"""
        super(LabelDialog, self).mousePressEvent(event)
        if event.button() == Qt.LeftButton:
            # 位置变化
            self.m_flag = True
            self.m_Position = event.globalPos() - self.pos()
            self.setCursor(QtGui.QCursor(Qt.OpenHandCursor))

    def mouseReleaseEvent(self, event):
        '''鼠标弹起事件'''
        super(LabelDialog, self).mouseReleaseEvent(event)
        self.m_flag = False
        self.setCursor(Qt.ArrowCursor)

    def mouseMoveEvent(self, event):
        """鼠标移动事件"""
        super(LabelDialog, self).mouseMoveEvent(event)
        self.setMouseTracking(True)
        if Qt.LeftButton and self.m_flag:
            self.move(event.globalPos() - self.m_Position)  # 更改窗口位置
            return