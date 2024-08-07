from qtpy import QtCore
from qtpy import QtGui
from qtpy import QtWidgets


class ZoomWidget(QtWidgets.QSpinBox):
    def __init__(self, value=100):
        super(ZoomWidget, self).__init__()
        self.setStyleSheet("color:#BFC7D7;\nbackground: #F4F6FA;\nborder: 1px solid #C3CEDF;\nfont-family:'HarmonyOS Sans SC Medium'")
        self.setMinimumHeight(20)
        self.setMinimumWidth(60)
        self.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.setRange(1, 1000)
        self.setSuffix(" %")
        self.setValue(value)
        self.setToolTip("Zoom Level")
        self.setStatusTip(self.toolTip())
        self.setAlignment(QtCore.Qt.AlignCenter)

    def minimumSizeHint(self):
        height = super(ZoomWidget, self).minimumSizeHint().height()
        fm = QtGui.QFontMetrics(self.font())
        width = fm.width(str(self.maximum()))
        return QtCore.QSize(width, height)
