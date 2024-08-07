# -*- encoding: utf-8 -*-

from qtpy.QtCore import Qt
from qtpy import QtWidgets, QtGui

from .escapable_qlist_widget import EscapableQListWidget


class UniqueLabelQListWidget(EscapableQListWidget):
    def mousePressEvent(self, event):
        super(UniqueLabelQListWidget, self).mousePressEvent(event)
        if not self.indexAt(event.pos()).isValid():
            self.clearSelection()

    def findItemsByLabel(self, label):
        items = []
        for row in range(self.count()):
            item = self.item(row)
            if item.data(Qt.UserRole) == label:
                items.append(item)
        return items

    def createItemFromLabel(self, label):
        item = QtWidgets.QListWidgetItem()
        item.setData(Qt.UserRole, label)
        return item

    def setItemLabel(self, item, label, color=None):
        qlabel = QtWidgets.QLabel()
        if color is None:
            qlabel.setText("{}".format(label))
        else:
            qlabel.setText(
                '{} <font color="#{:02x}{:02x}{:02x}">●</font>'.format(
                    label, *color
                )
            )
        qlabel.setContentsMargins(9, 0, 0, 0)
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(9)
        qlabel.setFont(font)

        self.setItemWidget(item, qlabel)
