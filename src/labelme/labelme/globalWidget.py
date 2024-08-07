from PyQt5 import QtWidgets

from src.labelme.labelme.uicommponet.global_widget import Ui_Form as Global_Ui_Form
from src.labelme.labelme.uicommponet.save_transfer_widget import Ui_Form as SaveTransfer_Ui_Form
from src.labelme.labelme.uicommponet.tool_bar import Ui_Form as ToolBar_Ui_Form


class GlobalWidget(QtWidgets.QWidget, Global_Ui_Form):
    def __init__(self):
        super(GlobalWidget, self).__init__()
        self.setupUi(self)
        self.open_folder.setText(self.tr("Open Folder"))
        self.open_pic.setText(self.tr("Open Pic"))
        self.save_file.setText(self.tr("Save"))
        self.delete_file.setText(self.tr("Delete"))


class SaveTransferWidget(QtWidgets.QWidget, SaveTransfer_Ui_Form):
    def __init__(self):
        super(SaveTransferWidget, self).__init__()
        self.setupUi(self)
        self.label.setText(self.tr("Autosave"))
        self.transfer.setText(self.tr("Transfer"))

class ToolBarWidget(QtWidgets.QWidget, ToolBar_Ui_Form):
    def __init__(self, model_no):
        super(ToolBarWidget, self).__init__()
        self.model_no = model_no
        self.setupUi(self)
        self.label_3.setText(self.tr("Global operation"))
