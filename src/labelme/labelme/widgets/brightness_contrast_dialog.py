import PIL.Image
import PIL.ImageEnhance
import PyQt5
from PyQt5.QtCore import QRect
from PyQt5.QtGui import QColor, QCursor
from qtpy.QtCore import Qt
from qtpy import QtGui
from qtpy import QtWidgets

from .. import utils


class BrightnessContrastDialog(QtWidgets.QDialog):
    def __init__(self, img, callback, parent=None):
        super(BrightnessContrastDialog, self).__init__(parent)
        self.setModal(True)
        self.setWindowFlag(Qt.FramelessWindowHint)
        self.setWindowFlags(Qt.Popup)
        # self.setWindowTitle("Brightness/Contrast")
        self.setStyleSheet(
            '''
            background: #FFFFFE;
            '''
        )
        self.slider_brightness = self._create_slider()
        self.slider_contrast = self._create_slider()

        formLayout = QtWidgets.QFormLayout()
        formLayout.addRow(self.tr("Brightness"), self.slider_brightness)
        formLayout.addRow(self.tr("Contrast"), self.slider_contrast)
        self.setLayout(formLayout)
        self.move(QtGui.QCursor.pos() + PyQt5.QtCore.QPoint(15, 15))
        assert isinstance(img, PIL.Image.Image)
        self.img = img
        self.callback = callback

    def onNewValue(self, value):
        brightness = self.slider_brightness.value() / 50.0
        contrast = self.slider_contrast.value() / 50.0

        img = self.img
        img = PIL.ImageEnhance.Brightness(img).enhance(brightness)
        img = PIL.ImageEnhance.Contrast(img).enhance(contrast)

        img_data = utils.img_pil_to_data(img)
        qimage = QtGui.QImage.fromData(img_data)
        self.callback(qimage)

    def _create_slider(self):
        slider = QtWidgets.QSlider(Qt.Horizontal)
        slider.setStyleSheet(
            '''
            QSlider::groove:horizontal {
            border: 0px solid #bbb;
            }
            QSlider::sub-page:horizontal {
            background: #0077FF;
            border-radius: 2px;
            margin-top:8px;
            margin-bottom:8px;
            }
            QSlider::add-page:horizontal {
            background: #E4EAF1;
            border-radius: 2px;
            margin-top:9px;
            margin-bottom:9px;
            }
            QSlider::handle:horizontal {
            background: #0077FF;
            width: 8px;
            border: 3px solid #FFFFFF;
            border-radius: 5px; 
            margin-top:4px;
            margin-bottom:4px;
            }
            QSlider::handle:horizontal:hover {
            background: #0077FF;
            width: 8px;
            border: 3px solid #FFFFFF;
            border-radius: 5px; 
            margin-top:4px;
            margin-bottom:4px;
            }
            '''
        )
        slider.setRange(0, 150)
        slider.setValue(50)
        slider.valueChanged.connect(self.onNewValue)
        return slider
