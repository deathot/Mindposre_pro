from qtpy import QtCore, QtGui
from qtpy import QtWidgets


class ToolBar(QtWidgets.QToolBar):
    def __init__(self, title):
        super(ToolBar, self).__init__(title)
        layout = self.layout()
        m = (0, 0, 0, 0)
        layout.setSpacing(0)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setMouseTracking(True)
        self.setContentsMargins(0, 0, 0, 0)
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.FramelessWindowHint)

    def addAction(self, action):
        if isinstance(action, QtWidgets.QWidgetAction):
            return super(ToolBar, self).addAction(action)
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(9)
        font.setBold(False)
        font.setWeight(50)
        if action.text() == self.tr('Global Operation'):
            btn = QtWidgets.QPushButton()
            btn.setFont(font)
            btn.setText(action.text())
            btn.setMinimumSize(300, 50)
            btn.setStyleSheet("QPushButton {color:#4E5865;\nbackground: #EBEFF6;\nborder-right: 1px solid #DFE5EF}")
        elif action.text() in [self.tr('Classification'), self.tr('Detection'), self.tr('Segmentation'), self.tr('Keypoint')]:
            btn = QtWidgets.QPushButton()
            btn.setFont(font)
            btn.setText(action.text())
            btn.setMinimumSize(100, 50)
            btn.setStyleSheet("QPushButton {color:#4E5865;\nbackground: #F4F6FA;\nborder: 1px solid #C3CEDF;\nmargin: 10px 10px;}")
        else:
            btn = QtWidgets.QToolButton()
            btn.setMinimumSize(50, 50)
            btn.setStyleSheet(f"QToolButton {'{' + 'color:#0077FF;background: #F4F6FA;border: none;' + '}'}"
                              f"QToolButton:hover {'{' + 'background-color: #DFE5EF;border: 10px solid #F4F6FA;' + '}'}"
                              f"QToolButton:pressed {'{' + 'background-color: #C3CEDF;border: 10px solid #F4F6FA;' + '}'}")
            btn.setDefaultAction(action)
            btn.setToolButtonStyle(QtCore.Qt.ToolButtonIconOnly)

        self.addWidget(btn)
        self.setStyleSheet("width: 0 px;\nheight: 0 px;\nmargin: 0 px;\npadding: 0px;\nbackground: #F4F6FA;\nborder: none")

        # center align
        for i in range(self.layout().count()):
            if isinstance(
                self.layout().itemAt(i).widget(), QtWidgets.QToolButton
            ):
                self.layout().itemAt(i).setAlignment(QtCore.Qt.AlignCenter)
