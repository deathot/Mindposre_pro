# -*- coding: utf-8 -*-

import functools
import json
import math
import os
import os.path as osp
import re
import shutil
import webbrowser

import imgviz
import natsort
from PyQt5.QtCore import QSize
from PyQt5.QtWidgets import QSizePolicy, QDockWidget
from qtpy import QtCore
from qtpy.QtCore import Qt
from qtpy import QtGui
from qtpy import QtWidgets

from src.labelme.labelme import __appname__
from src.labelme.labelme import PY2

from . import utils
from src.labelme.labelme.config import get_config
from src.labelme.labelme.label_file import LabelFile
from src.labelme.labelme.label_file import LabelFileError
from src.labelme.labelme.logger import logger
from src.labelme.labelme.shape import Shape
from src.labelme.labelme.widgets import BrightnessContrastDialog
from src.labelme.labelme.widgets import Canvas
from src.labelme.labelme.widgets import LabelDialog
from src.labelme.labelme.widgets import LabelListWidget
from src.labelme.labelme.widgets import LabelListWidgetItem
from src.labelme.labelme.widgets import ToolBar
from src.labelme.labelme.widgets import UniqueLabelQListWidget
from src.labelme.labelme.widgets import ZoomWidget

# FIXME
# - [medium] Set max zoom value to something big enough for FitWidth/Window

# TODO(unknown):
# - Zoom is too "steppy".
from src.labelme.labelme.convert_model.detection.convert_button import ConvertModel as det_conv_win
from src.labelme.labelme.convert_model.classification.convert_button import ConvertModel as cls_conv_win
from src.labelme.labelme.convert_model.keypoint.convert_button import ConvertModel as key_conv_win
from src.labelme.labelme.convert_model.segmentation.convert_button import ConvertModel as seg_conv_win
from src.labelme.labelme.globalWidget import GlobalWidget, SaveTransferWidget, ToolBarWidget

from .run_cmd_utils import is_contains_chinese
from src.labelme.labelme.uicommponet.note import Notes
from .uicommponet.yes_cancel_note import YesCancelNote

LABEL_COLORMAP = imgviz.label_colormap()


class MainWindow(QtWidgets.QMainWindow):

    FIT_WINDOW, FIT_WIDTH, MANUAL_ZOOM = 0, 1, 2

    def __init__(
        self,
        config=None,
        filename=None,
        output=None,
        output_file=None,
        output_dir=None,
        version_data=None,
        model_no=None,
        model_task_win=None,
        labelme_title=None,
        final_win=None,
    ):
        # super(MainWindow, self).__init__()
        self.convert_win = None
        self.version_data = version_data
        self.model_no = model_no
        self.model_task_win = model_task_win
        self.labelme_title = labelme_title
        self.final_win = final_win
        self.config = config
        self.filename = filename
        self.output_dir = output_dir
        self.output_file = output_file
        if output is not None:
            logger.warning(
                "argument output is deprecated, use output_file instead"
            )
            if output_file is None:
                output_file = output

        # see labelme/config/default_config.yaml for valid configuration
        if config is None:
            config = get_config()
        self._config = config

        # set default shape colors
        Shape.line_color = QtGui.QColor(*self._config["shape"]["line_color"])
        Shape.fill_color = QtGui.QColor(*self._config["shape"]["fill_color"])
        Shape.select_line_color = QtGui.QColor(
            *self._config["shape"]["select_line_color"]
        )
        Shape.select_fill_color = QtGui.QColor(
            *self._config["shape"]["select_fill_color"]
        )
        Shape.vertex_fill_color = QtGui.QColor(
            *self._config["shape"]["vertex_fill_color"]
        )
        Shape.hvertex_fill_color = QtGui.QColor(
            *self._config["shape"]["hvertex_fill_color"]
        )

        # Set point size from config file
        Shape.point_size = self._config["shape"]["point_size"]

        super(MainWindow, self).__init__()
        self.setMouseTracking(True)
        self.setStyleSheet("QMainWindow {background-color: #FEFEFE;}"
                           "QMainWindow::separator {width: 0px;height: 0px;margin: 0px;padding: 0px;}")
        self.setContextMenuPolicy(Qt.NoContextMenu)
        # self.setContentsMargins(0,0,0,0)
        # self.setMinimumHeight(1677721)
        # # 隐藏头部
        # self.setWindowFlag(Qt.FramelessWindowHint)
        # # 隐藏边框
        # self.setAttribute(Qt.WA_TranslucentBackground)

        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(9)
        font.setBold(False)
        font.setWeight(50)
        self.setFont(font)
        # self.setWindowTitle(self.tr("Data Annotation"))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.statusBar().setStyleSheet("color:#4E5865;\nborder: 1px solid #DFE5EF;\nbackground: #F4F6FA;")
        self.statusBar().setMouseTracking(True)

        # Whether we need to save or not.
        self.dirty = False

        self._noSelectionSlot = False

        self._copied_shapes = None

        list_qss = '''
                   QListWidget { color:#4E5865; background-color: #f4f6fa; border: none;padding-left: 5px;}
                   QListWidget::Item { height: 24px; background: #f4f6fa; color: #000000; }
                   QListWidget::Item:hover { height: 24px; background: #52A3FF; color: #000000; }
                   QListWidget::Item:selected { height: 24px; background: #0077FF; color: #FFFFFF; }
                   QListWidget::indicator{width: 18px; height:18px}
                   QListWidget::indicator:checked{image:url(:/icon/image/check_yes.svg)}
                   QListWidget::indicator:checked:hover{image:url(:/icon/image/check_hover.svg)}
                   QListWidget::indicator:unchecked{image:url(:/icon/image/check_no.svg)}
                   '''
        vertical_qss = """
                    QScrollBar:vertical { border: none; border-radius: 4px; width: 8px; background-color: #DFE5EF; margin: 2px 0 2px 0; }
                    QScrollBar::handle:vertical { border: none; border-radius: 4px; background-color:  #C3CEDF; height: 60px; margin: 0px 0px 0px 0px; }
                    """
        horizontal_qss = """
                        QScrollBar:horizontal { border: none; border-radius: 4px; height: 8px; background-color: #DFE5EF; margin: 0 2px 0 2px;}
                        QScrollBar::handle:horizontal { border: none; border-radius: 4px; background-color:  #C3CEDF; width: 60px; margin: 0px 0px 0px 0px;}
                        """

        # Main widgets and related state.
        self.labelDialog = LabelDialog(
            parent=self,
            labels=self._config["labels"],
            sort_labels=self._config["sort_labels"],
            show_text_field=self._config["show_label_text_field"],
            completion=self._config["label_completion"],
            fit_to_content=self._config["fit_to_content"],
            flags=self._config["label_flags"],
        )

        self.labelList = LabelListWidget()
        # self.labelList.setFont(font)
        self.labelList.setStyleSheet(
            "QListView { color:#4E5865; background-color: #f4f6fa; border: none;padding-left: 5px;}"
            "QListView::Item { height: 24px; background: #f4f6fa; color: #000000; }"
            "QListView::Item:hover { height: 24px; background: #52A3FF; color: #000000; }"
            "QListView::Item:selected { height: 24px; background: #0077FF; color: #FFFFFF; }"
            "QListView::indicator{width: 18px; height:18px}"
            "QListView::indicator:checked{image:url(:/icon/image/check_yes.svg)}"
            "QListView::indicator:checked:hover{image:url(:/icon/image/check_hover.svg)}"
            "QListView::indicator:unchecked{image:url(:/icon/image/check_no.svg)}"
        )
        self.labelList.verticalScrollBar().setStyleSheet(vertical_qss)
        self.labelList.horizontalScrollBar().setStyleSheet(horizontal_qss)
        self.lastOpenDir = None

        self.flag_dock = self.flag_widget = None
        self.flag_dock = QtWidgets.QDockWidget(self.tr("Flags"), self)
        self.flag_dock.setStyleSheet("QDockWidget::title {padding-left: 4px;\npadding-top: 4px;color:#00ff00;\nbackground-color: #EBEFF6;\nfont-family:'HarmonyOS Sans SC Medium'}")
        self.flag_dock.setObjectName("Flags")
        self.flag_widget = QtWidgets.QListWidget()
        self.flag_widget.setMouseTracking(True)
        self.flag_widget.setStyleSheet(list_qss)
        self.flag_widget.verticalScrollBar().setStyleSheet(vertical_qss)
        self.flag_widget.horizontalScrollBar().setStyleSheet(horizontal_qss)
        if config["flags"]:
            self.loadFlags({k: False for k in config["flags"]})
        self.flag_dock.setWidget(self.flag_widget)
        self.flag_widget.itemChanged.connect(self.setDirty)

        self.labelList.itemSelectionChanged.connect(self.labelSelectionChanged)
        self.labelList.itemDoubleClicked.connect(self.editLabel)
        self.labelList.itemChanged.connect(self.labelItemChanged)
        self.labelList.itemDropped.connect(self.labelOrderChanged)
        self.labelList.setMouseTracking(True)
        self.shape_dock = QtWidgets.QDockWidget(
            self.tr("Polygon Labels"), self
        )
        self.shape_dock.setStyleSheet("QDockWidget::title {padding-left: 4px;\npadding-top: 4px;color:#4E5865;\nbackground-color: #EBEFF6;\nfont-family:'HarmonyOS Sans SC Medium'}")
        self.shape_dock.setObjectName("Labels")
        self.shape_dock.setWidget(self.labelList)

        self.uniqLabelList = UniqueLabelQListWidget()
        self.uniqLabelList.setToolTip(
            self.tr(
                "Select label to start annotating for it. "
                "Press 'Esc' to deselect."
            )
        )
        if self._config["labels"]:
            for label in self._config["labels"]:
                item = self.uniqLabelList.createItemFromLabel(label)
                self.uniqLabelList.addItem(item)
                # self.uniqLabelList.setSpacing(5)
                rgb = self._get_rgb_by_label(label)
                self.uniqLabelList.setItemLabel(item, label, rgb)
        self.label_dock = QtWidgets.QDockWidget(self.tr("Label List"), self)
        self.label_dock.setStyleSheet("QDockWidget::title {padding-left: 4px;\npadding-top: 4px;color:#4E5865;\nbackground-color: #EBEFF6;\nfont-family:'HarmonyOS Sans SC Medium'}")
        self.label_dock.setObjectName("Label List")
        self.uniqLabelList.setMouseTracking(True)
        self.uniqLabelList.setStyleSheet(list_qss)
        self.uniqLabelList.verticalScrollBar().setStyleSheet(vertical_qss)
        self.uniqLabelList.horizontalScrollBar().setStyleSheet(horizontal_qss)
        self.label_dock.setWidget(self.uniqLabelList)

        self.fileSearch = QtWidgets.QLineEdit()
        self.fileSearch.setTextMargins(10, 0, 0, 0)
        self.fileSearch.setFixedHeight(36)
        self.fileSearch.setContentsMargins(6, 6, 6, 6)
        self.fileSearch.setPlaceholderText(self.tr("Search Filename"))
        self.fileSearch.textChanged.connect(self.fileSearchChanged)
        self.fileSearch.setFont(font)
        self.fileListWidget = QtWidgets.QListWidget()
        self.fileListWidget.setStyleSheet(list_qss)
        self.fileListWidget.verticalScrollBar().setStyleSheet(vertical_qss)
        self.fileListWidget.horizontalScrollBar().setStyleSheet(horizontal_qss)
        self.fileListWidget.itemSelectionChanged.connect(
            self.fileSelectionChanged
        )
        fileListLayout = QtWidgets.QVBoxLayout()
        fileListLayout.setContentsMargins(0, 0, 0, 0)
        fileListLayout.setSpacing(0)
        fileListLayout.addWidget(self.fileSearch)
        fileListLayout.addWidget(self.fileListWidget)
        self.file_dock = QtWidgets.QDockWidget(self.tr("File List"), self)
        self.file_dock.setStyleSheet("QDockWidget::title {padding-left: 4px;\npadding-top: 4px;color:#4E5865;\nbackground-color: #EBEFF6;\nfont-family:'HarmonyOS Sans SC Medium'}")
        self.file_dock.setObjectName("Files")
        fileListWidget = QtWidgets.QWidget()
        fileListWidget.setStyleSheet("color:#4E5865;\nbackground-color: #f4f6fa;\nborder: 1px solid #DFE5EF;\nfont-family:'HarmonyOS Sans SC Medium'")
        self.fileSearch.setStyleSheet("color:#4E5865;\nbackground-color: #f4f6fa;\nborder: 1px solid #C3CEDF;\nborder-radius: 2px;")
        fileListWidget.setLayout(fileListLayout)
        fileListWidget.setMouseTracking(True)
        self.file_dock.setWidget(fileListWidget)

        self.zoomWidget = ZoomWidget()
        self.setAcceptDrops(True)

        self.canvas = self.labelList.canvas = Canvas(
            epsilon=self._config["epsilon"],
            double_click=self._config["canvas"]["double_click"],
            num_backups=self._config["canvas"]["num_backups"],
        )
        self.canvas.zoomRequest.connect(self.zoomRequest)

        scrollArea = QtWidgets.QScrollArea()
        # scrollArea.setStyleSheet("background: #FEFEFE;\nmargin: 0 px;\nborder: none;")
        scrollArea.setStyleSheet("""
        QScrollArea {
            background-color: #FEFEFE;
            margin: 0 px;
            border:none;
        }
        QScrollBar:vertical {
            border: none;
            border-radius: 4px;
            width: 8px;
            background-color: #DFE5EF;
            margin: 2px 0 2px 0;
        }
        QScrollBar::handle:vertical {
            border: none;
            border-radius: 4px;
            background-color:  #C3CEDF;
            height: 60px;
            margin: 0px 0px 0px 0px;
        }
        QScrollBar:horizontal {
            border: none;
            border-radius: 4px;
            height: 8px;
            background-color: #DFE5EF;
            margin: 0 2px 0 2px;
        }
        QScrollBar::handle:horizontal {
            border: none;
            border-radius: 4px;
            background-color:  #C3CEDF;
            width: 60px;
            margin: 0px 0px 0px 0px;
        }
        """)
        scrollArea.setWidget(self.canvas)
        scrollArea.setWidgetResizable(True)
        self.scrollBars = {
            Qt.Vertical: scrollArea.verticalScrollBar(),
            Qt.Horizontal: scrollArea.horizontalScrollBar(),
        }
        self.canvas.scrollRequest.connect(self.scrollRequest)

        self.canvas.newShape.connect(self.newShape)
        self.canvas.shapeMoved.connect(self.setDirty)
        self.canvas.selectionChanged.connect(self.shapeSelectionChanged)
        self.canvas.drawingPolygon.connect(self.toggleDrawingSensitive)

        self.setCentralWidget(scrollArea)

        features = QtWidgets.QDockWidget.DockWidgetFeatures()
        for dock in ["flag_dock", "label_dock", "shape_dock", "file_dock"]:
            if self._config[dock]["closable"]:
                features = features | QtWidgets.QDockWidget.DockWidgetClosable
            if self._config[dock]["floatable"]:
                features = features | QtWidgets.QDockWidget.DockWidgetFloatable
            if self._config[dock]["movable"]:
                features = features | QtWidgets.QDockWidget.DockWidgetMovable
            getattr(self, dock).setFeatures(features)
            if self._config[dock]["show"] is False:
                getattr(self, dock).setVisible(False)

        self.global_dock = QtWidgets.QDockWidget(self.tr("Global Operation"), self)
        self.global_dock.setObjectName("Global Operation")
        # self.global_dock.setStyleSheet("QDockWidget::title {color:#4E5865;\nbackground-color: #EBEFF6;}")
        self.global_widget = GlobalWidget()
        self.global_dock.setWidget(self.global_widget)
        self.global_dock.setTitleBarWidget(QtWidgets.QWidget())
        self.global_widget.open_folder.clicked.connect(self.openDirDialog)
        self.global_widget.open_pic.clicked.connect(self.openFile)
        self.global_widget.save_file.clicked.connect(self.saveFile)
        self.global_widget.delete_file.clicked.connect(self.deleteFile)
        if self.hasLabelFile():
            self.global_widget.delete_file.setEnabled(True)
        else:
            self.global_widget.delete_file.setEnabled(False)
        self.global_widget.save_file.setEnabled(False)
        self.save_transfer_dock = QtWidgets.QDockWidget(self.tr("Transfer"), self)
        self.save_transfer_dock.setObjectName("Transfer")
        self.save_transfer_widget = SaveTransferWidget()
        self.save_transfer_dock.setWidget(self.save_transfer_widget)
        self.save_transfer_widget.auto_save.clicked.connect(self.auto_save_func)
        self.save_transfer_widget.transfer.clicked.connect(self.showConvertModel)
        self.save_transfer_dock.setTitleBarWidget(QtWidgets.QWidget())

        self.tool_bar_dock = QtWidgets.QDockWidget(self.tr("Tool Bar"), self)
        self.tool_bar_dock.setObjectName("Tool Bar")
        self.tool_bar_widget = ToolBarWidget(self.model_no)

        self.tool_bar_widget.rectangle.clicked.connect(lambda: self.toggleDrawMode(False, createMode="rectangle"))
        self.tool_bar_widget.polygon.clicked.connect(lambda: self.toggleDrawMode(False, createMode="polygon"))
        self.tool_bar_widget.point.clicked.connect(lambda: self.toggleDrawMode(False, createMode="point"))
        self.tool_bar_widget.edit.clicked.connect(self.setEditMode)
        self.tool_bar_widget.copy.clicked.connect(self.duplicateSelectedShape)
        self.tool_bar_widget.delete_2.clicked.connect(self.deleteSelectedShape)
        self.tool_bar_widget.color.clicked.connect(self.brightnessContrast)
        self.tool_bar_widget.undo.clicked.connect(self.undoShapeEdit)
        self.tool_bar_widget.prev.clicked.connect(self.openPrevImg)
        self.tool_bar_widget.next.clicked.connect(self.openNextImg)

        self.tool_bar_list = [
            self.tool_bar_widget.rectangle,
            self.tool_bar_widget.polygon,
            self.tool_bar_widget.point,
            self.tool_bar_widget.edit,
            self.tool_bar_widget.copy,
            self.tool_bar_widget.delete_2,
            self.tool_bar_widget.color,
            self.tool_bar_widget.undo,
            self.tool_bar_widget.prev,
            self.tool_bar_widget.next
        ]

        self.tool_bar_dock.setWidget(self.tool_bar_widget)
        self.tool_bar_dock.setTitleBarWidget(QtWidgets.QWidget())
        self.flag_dock.setMouseTracking(True)
        self.label_dock.setMouseTracking(True)
        self.shape_dock.setMouseTracking(True)
        self.file_dock.setMouseTracking(True)

        self.flag_dock.setFeatures(QDockWidget.NoDockWidgetFeatures)
        self.label_dock.setFeatures(QDockWidget.NoDockWidgetFeatures)
        self.shape_dock.setFeatures(QDockWidget.NoDockWidgetFeatures)
        self.file_dock.setFeatures(QDockWidget.NoDockWidgetFeatures)
        self.save_transfer_dock.setFeatures(QDockWidget.NoDockWidgetFeatures)
        self.global_dock.setFeatures(QDockWidget.NoDockWidgetFeatures)
        self.tool_bar_dock.setFeatures(QDockWidget.NoDockWidgetFeatures)

        self.addDockWidget(Qt.RightDockWidgetArea, self.flag_dock)
        self.addDockWidget(Qt.RightDockWidgetArea, self.label_dock)
        self.addDockWidget(Qt.RightDockWidgetArea, self.shape_dock)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.global_dock)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.file_dock)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.save_transfer_dock)
        self.addDockWidget(Qt.TopDockWidgetArea, self.tool_bar_dock)

        # Actions
        action = functools.partial(utils.newAction, self)
        shortcuts = self._config["shortcuts"]
        quit = action(
            self.tr("Quit"),
            self.closeLabelme,
            shortcuts["quit"],
            icon=None,
            tip=self.tr("Quit application"),
        )
        global_title = action(
            self.tr('Global Operation'),
        )
        model_name = action(
            model_no,
        )
        open_ = action(
            self.tr("Open"),
            self.openFile,
            shortcuts["open"],
            icon=None,
            tip=self.tr("Open image or label file"),
        )
        opendir = action(
            self.tr("Open Dir"),
            self.openDirDialog,
            shortcuts["open_dir"],
            icon=None,
            tip=self.tr("Open Dir"),
        )
        openNextImg = action(
            self.tr("Next Image"),
            self.openNextImg,
            shortcuts["open_next"],
            icon=None,
            tip=self.tr("Open next (hold Ctl+Shift to copy labels)"),
            enabled=False,
        )
        openPrevImg = action(
            self.tr("Prev Image"),
            self.openPrevImg,
            shortcuts["open_prev"],
            icon=None,
            tip=self.tr("Open prev (hold Ctl+Shift to copy labels)"),
            enabled=False,
        )
        save = action(
            self.tr("Save"),
            self.saveFile,
            shortcuts["save"],
            icon=None,
            tip=self.tr("Save labels to file"),
            enabled=False,
        )
        saveAs = action(
            self.tr("Save As"),
            self.saveFileAs,
            shortcuts["save_as"],
            icon=None,
            tip=self.tr("Save labels to a different file"),
            enabled=False,
        )

        deleteFile = action(
            self.tr("Delete File"),
            self.deleteFile,
            shortcuts["delete_file"],
            icon=None,
            tip=self.tr("Delete current label file"),
            enabled=False,
        )

        changeOutputDir = action(
            self.tr("&Change Output Dir"),
            slot=self.changeOutputDirDialog,
            shortcut=shortcuts["save_to"],
            icon=None,
            tip=self.tr("Change where annotations are loaded/saved"),
        )

        saveAuto = action(
            text=self.tr("Save Automatically"),
            slot=lambda x: self.actions.saveAuto.setChecked(x),
            icon="save",
            tip=self.tr("Save automatically"),
            checkable=True,
            enabled=True,
        )
        saveAuto.setChecked(True)

        saveWithImageData = action(
            text=self.tr("Save With Image Data"),
            slot=self.enableSaveImageWithData,
            tip=self.tr("Save image data in label file"),
            checkable=True,
            checked=self._config["store_data"],
        )

        close = action(
            self.tr("Close"),
            self.closeFile,
            shortcuts["close"],
            icon=None,
            tip="Close current file",
            enabled=False,
        )

        toggle_keep_prev_mode = action(
            self.tr("Keep Previous Annotation"),
            self.toggleKeepPrevMode,
            shortcuts["toggle_keep_prev_mode"],
            icon=None,
            tip=self.tr('Toggle "keep pevious annotation" mode'),
            checkable=True,
        )
        toggle_keep_prev_mode.setChecked(self._config["keep_prev"])

        createMode = action(
            self.tr("Create Polygons"),
            lambda: self.toggleDrawMode(False, createMode="polygon"),
            shortcuts["create_polygon"],
            icon=None,
            tip=self.tr("Start drawing polygons"),
            enabled=False,
        )
        createRectangleMode = action(
            self.tr("Create Rectangle"),
            lambda: self.toggleDrawMode(False, createMode="rectangle"),
            shortcuts["create_rectangle"],
            icon=None,
            tip=self.tr("Start drawing rectangles"),
            enabled=False,
        )
        createCircleMode = action(
            self.tr("Create Circle"),
            lambda: self.toggleDrawMode(False, createMode="circle"),
            shortcuts["create_circle"],
            icon=None,
            tip=self.tr("Start drawing circles"),
            enabled=False,
        )
        createLineMode = action(
            self.tr("Create Line"),
            lambda: self.toggleDrawMode(False, createMode="line"),
            shortcuts["create_line"],
            icon=None,
            tip=self.tr("Start drawing lines"),
            enabled=False,
        )
        createPointMode = action(
            self.tr("Create Point"),
            lambda: self.toggleDrawMode(False, createMode="point"),
            shortcuts["create_point"],
            icon=None,
            tip=self.tr("Start drawing points"),
            enabled=False,
        )
        createLineStripMode = action(
            self.tr("Create LineStrip"),
            lambda: self.toggleDrawMode(False, createMode="linestrip"),
            shortcuts["create_linestrip"],
            icon=None,
            tip=self.tr("Start drawing linestrip. Ctrl+LeftClick ends creation."),
            enabled=False,
        )
        editMode = action(
            self.tr("Edit Polygons"),
            self.setEditMode,
            shortcuts["edit_polygon"],
            icon=None,
            tip=self.tr("Move and edit the selected polygons"),
            enabled=False,
        )

        delete = action(
            self.tr("Delete Polygons"),
            self.deleteSelectedShape,
            shortcuts["delete_polygon"],
            icon=None,
            tip=self.tr("Delete the selected polygons"),
            enabled=False,
        )
        deleteLabel = action(
            self.tr("Delete Label"),
            self.delLabel,
            shortcut=None,
            icon=None,
            tip=None,
            enabled=True,
        )
        duplicate = action(
            self.tr("Duplicate Polygons"),
            self.duplicateSelectedShape,
            shortcuts["duplicate_polygon"],
            "copy",
            self.tr("Create a duplicate of the selected polygons"),
            enabled=False,
        )
        copy = action(
            self.tr("Copy Polygons"),
            self.copySelectedShape,
            shortcuts["copy_polygon"],
            icon=None,
            tip=self.tr("Copy selected polygons to clipboard"),
            enabled=False,
        )
        paste = action(
            self.tr("Paste Polygons"),
            self.pasteSelectedShape,
            shortcuts["paste_polygon"],
            icon=None,
            tip=self.tr("Paste copied polygons"),
            enabled=False,
        )
        undoLastPoint = action(
            self.tr("Undo last point"),
            self.canvas.undoLastPoint,
            shortcuts["undo_last_point"],
            icon=None,
            tip=self.tr("Undo last drawn point"),
            enabled=False,
        )
        removePoint = action(
            text=self.tr("Remove Selected Point"),
            slot=self.removeSelectedPoint,
            shortcut=shortcuts["remove_selected_point"],
            icon=None,
            tip=self.tr("Remove selected point from polygon"),
            enabled=False,
        )

        undo = action(
            self.tr("Undo"),
            self.undoShapeEdit,
            shortcuts["undo"],
            icon=None,
            tip=self.tr("Undo last add and edit of shape"),
            enabled=False,
        )

        hideAll = action(
            self.tr("Hide Polygons"),
            functools.partial(self.togglePolygons, False),
            icon=None,
            tip=self.tr("Hide all polygons"),
            enabled=False,
        )
        showAll = action(
            self.tr("Show Polygons"),
            functools.partial(self.togglePolygons, True),
            icon=None,
            tip=self.tr("Show all polygons"),
            enabled=False,
        )

        convert_model = action(
            self.tr("Transfer"),
            self.convertModel,
            icon=None,
            tip=self.tr("Push to Package"),
        )

        help = action(
            self.tr("Tutorial"),
            self.tutorial,
            icon=None,
            tip=self.tr("Show tutorial page"),
        )

        zoom = QtWidgets.QWidgetAction(self)
        zoom.setDefaultWidget(self.zoomWidget)
        self.zoomWidget.setWhatsThis(
            str(
                self.tr(
                    "Zoom in or out of the image. Also accessible with "
                    "{} and {} from the canvas."
                )
            ).format(
                utils.fmtShortcut(
                    "{},{}".format(shortcuts["zoom_in"], shortcuts["zoom_out"])
                ),
                utils.fmtShortcut(self.tr("Ctrl+Wheel")),
            )
        )
        self.zoomWidget.setEnabled(False)

        zoomIn = action(
            self.tr("Zoom In"),
            functools.partial(self.addZoom, 1.1),
            shortcuts["zoom_in"],
            icon=None,
            tip=self.tr("Increase zoom level"),
            enabled=False,
        )
        zoomOut = action(
            self.tr("Zoom Out"),
            functools.partial(self.addZoom, 0.9),
            shortcuts["zoom_out"],
            icon=None,
            tip=self.tr("Decrease zoom level"),
            enabled=False,
        )
        zoomOrg = action(
            self.tr("Original size"),
            functools.partial(self.setZoom, 100),
            shortcuts["zoom_to_original"],
            icon=None,
            tip=self.tr("Zoom to original size"),
            enabled=False,
        )
        keepPrevScale = action(
            self.tr("&Keep Previous Scale"),
            self.enableKeepPrevScale,
            tip=self.tr("Keep previous zoom scale"),
            checkable=True,
            checked=self._config["keep_prev_scale"],
            enabled=True,
        )
        fitWindow = action(
            self.tr("Fit Window"),
            self.setFitWindow,
            shortcuts["fit_window"],
            icon=None,
            tip=self.tr("Zoom follows window size"),
            checkable=True,
            enabled=False,
        )
        fitWidth = action(
            self.tr("FitWidth"),
            self.setFitWidth,
            shortcuts["fit_width"],
            icon=None,
            tip=self.tr("Zoom follows window width"),
            checkable=True,
            enabled=False,
        )
        brightnessContrast = action(
            self.tr("Brightness Contrast"),
            self.brightnessContrast,
            None,
            icon=None,
            tip=self.tr("Adjust brightness and contrast"),
            enabled=False,
        )
        # Group zoom controls into a list for easier toggling.
        zoomActions = (
            self.tool_bar_widget.zoom,
            self.zoomWidget,
            zoomIn,
            zoomOut,
            zoomOrg,
            fitWindow,
            fitWidth,
        )
        self.zoomMode = self.FIT_WINDOW
        fitWindow.setChecked(Qt.Checked)
        self.scalers = {
            self.FIT_WINDOW: self.scaleFitWindow,
            self.FIT_WIDTH: self.scaleFitWidth,
            # Set to one to scale to 100% when loading files.
            self.MANUAL_ZOOM: lambda: 1,
        }

        edit = action(
            self.tr("Edit Label"),
            self.editLabel,
            shortcuts["edit_label"],
            icon=None,
            tip=self.tr("Modify the label of the selected polygon"),
            enabled=False,
        )

        fill_drawing = action(
            self.tr("Fill Drawing Polygon"),
            self.canvas.setFillDrawing,
            None,
            "color",
            self.tr("Fill polygon while drawing"),
            checkable=True,
            enabled=True,
        )
        fill_drawing.trigger()

        # Lavel list context menu.
        labelMenu = QtWidgets.QMenu()
        labelMenu.setMouseTracking(True)
        utils.addActions(labelMenu, (edit, delete))
        self.labelList.setContextMenuPolicy(Qt.CustomContextMenu)
        self.labelList.customContextMenuRequested.connect(
            self.popLabelListMenu
        )

        uniqLabelMenu = QtWidgets.QMenu()
        uniqLabelMenu.setMouseTracking(True)
        utils.addActions(uniqLabelMenu, (deleteLabel,))
        self.uniqLabelList.setContextMenuPolicy(Qt.CustomContextMenu)
        self.uniqLabelList.customContextMenuRequested.connect(self.popUniqLabelListMenu)

        self.actions = utils.struct(
            saveAuto=saveAuto,
            saveWithImageData=saveWithImageData,
            changeOutputDir=changeOutputDir,
            save=save,
            saveAs=saveAs,
            open=open_,
            close=close,
            deleteFile=deleteFile,
            toggleKeepPrevMode=toggle_keep_prev_mode,
            delete=delete,
            deleteLabel=deleteLabel,
            edit=edit,
            duplicate=duplicate,
            copy=copy,
            paste=paste,
            undoLastPoint=undoLastPoint,
            undo=undo,
            removePoint=removePoint,
            createMode=createMode,
            editMode=editMode,
            createRectangleMode=createRectangleMode,
            createCircleMode=createCircleMode,
            createLineMode=createLineMode,
            createPointMode=createPointMode,
            createLineStripMode=createLineStripMode,
            zoom=zoom,
            zoomIn=zoomIn,
            zoomOut=zoomOut,
            zoomOrg=zoomOrg,
            keepPrevScale=keepPrevScale,
            fitWindow=fitWindow,
            fitWidth=fitWidth,
            brightnessContrast=brightnessContrast,
            zoomActions=zoomActions,
            openNextImg=openNextImg,
            openPrevImg=openPrevImg,
            fileMenuActions=(open_, opendir, save, saveAs, close, quit),
            tool=(),
            # XXX: need to add some actions here to activate the shortcut
            editMenu=(
                edit,
                copy,
                paste,
                delete,
                None,
                undo,
                undoLastPoint,
            ),
            views=(
                hideAll,
                showAll,
                zoomIn,
                zoomOut,
                zoomOrg,
                fitWindow,
                fitWidth,
                brightnessContrast,
            ),
            # menu shown at right click
            menu=(
                createRectangleMode,
                createMode,
                createPointMode,
                editMode,
                edit,
                copy,
                paste,
                delete,
                undo,
                undoLastPoint,
            ),
            onLoadActive=(
                self.tool_bar_widget.polygon,
                self.tool_bar_widget.rectangle,
                self.tool_bar_widget.point,
                self.tool_bar_widget.edit,
                self.tool_bar_widget.color,
                close,
                createMode,
                createRectangleMode,
                createCircleMode,
                createLineMode,
                createPointMode,
                createLineStripMode,
                editMode,
                brightnessContrast,
            ),
            onShapesPresent=(saveAs, hideAll, showAll),
        )

        self.canvas.vertexSelected.connect(self.actions.removePoint.setEnabled)
        # self.tools = self.toolbar(self.tr("Tools"))
        self.menus = utils.struct(
            file=self.menu(self.tr("File")),
            edit=self.menu(self.tr("Edit")),
            view=self.menu(self.tr("View")),
            train=self.menu(self.tr("Model Tools")),
            help=self.menu(self.tr("Help")),
            recentFiles=QtWidgets.QMenu(self.tr("Open Recent")),
            labelList=labelMenu,
            uniqLabelList=uniqLabelMenu,
        )
        self.menus.recentFiles.setFont(font)
        menus_qss = """
            QMenu {
                background-color: #FEFEFE;
            }
            QMenu::item {
                color: #000000;
                background: #FEFEFE;
            }
            QMenu::item:selected {
                color: #ffffff;
                background-color: #0077ff;    
            }
            QMenu::item:!enabled {
                color:#BFC7D7;
                background-color:#FEFEFE;
            }
        """
        self.menus.file.setStyleSheet(menus_qss)
        self.menus.edit.setStyleSheet(menus_qss)
        self.menus.view.setStyleSheet(menus_qss)
        self.menus.train.setStyleSheet(menus_qss)
        self.menus.help.setStyleSheet(menus_qss)
        self.menus.recentFiles.setStyleSheet(menus_qss)
        self.menus.labelList.setStyleSheet(menus_qss)

        utils.addActions(
            self.menus.file,
            (
                open_,
                openNextImg,
                openPrevImg,
                opendir,
                self.menus.recentFiles,
                save,
                close,
                deleteFile,
                None,
                quit,
            ),
        )
        utils.addActions(
            self.menus.train,
            (
                convert_model,
            ),
        )
        utils.addActions(self.menus.help, (help,))
        utils.addActions(
            self.menus.view,
            (
                hideAll,
                showAll,
                None,
                zoomIn,
                zoomOut,
                zoomOrg,
                None,
                fitWindow,
                fitWidth,
                None,
                brightnessContrast,
            ),
        )

        self.menus.file.aboutToShow.connect(self.updateFileMenu)

        # Custom context menu for the canvas widget:
        utils.addActions(self.canvas.menus[0], self.actions.menu)
        utils.addActions(
            self.canvas.menus[1],
            (
                action("粘贴", self.copyShape),
                action("移动", self.moveShape),
            ),
        )
        self.canvas.menus[0].setStyleSheet("QMenu {background-color: #FEFEFE}"
                                           "QMenu::item {color: #000000;\nbackground: #FEFEFE;}"
                                           "QMenu::item:selected {color: #ffffff;\nbackground-color: #0077ff}"
                                           "QMenu::item:!enabled {color:#777777;\nbackground-color:#FEFEFE;}")
        self.canvas.menus[1].setStyleSheet("QMenu {background-color: #FEFEFE}"
                                           "QMenu::item {color: #000000;\nbackground: #FEFEFE;}"
                                           "QMenu::item:selected {color: #ffffff;\nbackground-color: #0077ff}"
                                           "QMenu::item:!enabled {color:#777777;\nbackground-color:#FEFEFE;}")

        # Menu buttons on Top
        self.actions.tool = (
            global_title,
            model_name,
            createMode,
            createRectangleMode,
            createPointMode,
            editMode,
            duplicate,
            delete,
            brightnessContrast,
            undo,
            openPrevImg,
            openNextImg,
            zoom,
        )
        self.statusBar().show()

        if output_file is not None and self._config["auto_save"]:
            logger.warn(
                "If `auto_save` argument is True, `output_file` argument "
                "is ignored and output filename is automatically "
                "set as IMAGE_BASENAME.json."
            )
        self.output_file = output_file
        self.output_dir = output_dir

        # Application state.
        self.image = QtGui.QImage()
        self.imagePath = None
        self.recentFiles = []
        self.maxRecent = 7
        self.otherData = None
        self.zoom_level = 100
        self.fit_window = False
        self.zoom_values = {}  # key=filename, value=(zoom_mode, zoom_value)
        self.brightnessContrast_values = {}
        self.scroll_values = {
            Qt.Horizontal: {},
            Qt.Vertical: {},
        }  # key=filename, value=scroll_value

        if filename is not None and osp.isdir(filename):
            self.importDirImages(filename, load=False)
        else:
            self.filename = filename

        if config["file_search"]:
            self.fileSearch.setText(config["file_search"])
            self.fileSearchChanged()

        # XXX: Could be completely declarative.
        # Restore application settings.
        self.settings = QtCore.QSettings("labelme", "labelme")
        self.recentFiles = self.settings.value("recentFiles", []) or []
        # size = self.settings.value("window/size", QtCore.QSize(600, 500))
        # position = self.settings.value("window/position", QtCore.QPoint(0, 0))
        # state = self.settings.value("window/state", QtCore.QByteArray())
        # self.resize(size)
        # self.move(position)
        # or simply:
        # self.restoreGeometry(settings['window/geometry']
        # self.restoreState(state)

        # Populate the File menu dynamically.
        self.updateFileMenu()
        # Since loading the file may take some time,
        # make sure it runs in the background.
        if self.filename is not None:
            self.queueEvent(functools.partial(self.loadFile, self.filename))

        # Callbacks:
        self.zoomWidget.valueChanged.connect(self.paintCanvas)
        self.tool_bar_widget.zoom.valueChanged.connect(self.paintCanvas)

        self.populateModeActions()

        # self.firstStart = True
        # if self.firstStart:
        #    QWhatsThis.enterWhatsThisMode()

    def auto_save_func(self):
        icon = QtGui.QIcon()
        if self.actions.saveAuto.isChecked():
            icon.addPixmap(QtGui.QPixmap(":/icon/image/auto_save_close.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.actions.saveAuto.setChecked(False)
        else:
            icon.addPixmap(QtGui.QPixmap(":/icon/image/auto_save_open.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.actions.saveAuto.setChecked(True)
        self.save_transfer_widget.auto_save.setIcon(icon)

    def showConvertModel(self):
        # 检查窗口是否被重复创建，防止覆盖原来的窗口
        if self.convert_win is None:
            if self.model_no == self.tr("Detection"):
                self.convert_win = det_conv_win(model_no=self.model_no)
            elif self.model_no == self.tr("Classification"):
                self.convert_win = cls_conv_win(model_no=self.model_no)
            elif self.model_no == self.tr("Keypoint"):
                self.convert_win = key_conv_win(model_no=self.model_no)
            elif self.model_no == self.tr("Segmentation"):
                self.convert_win = seg_conv_win(model_no=self.model_no)
        self.convert_win.showNormal()
        self.convert_win.raise_()
        self.convert_win.show()

    def menu(self, title, actions=None):
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(9)
        font.setBold(False)
        font.setWeight(50)
        menubar = self.menuBar()
        menubar.setFont(font)
        menubar.setFixedHeight(44)
        menubar.setMouseTracking(True)
        menu = menubar.addMenu(title)
        menu.setMouseTracking(True)
        if actions:
            utils.addActions(menu, actions)
        return menu

    def toolbar(self, title, actions=None):
        toolbar = ToolBar(title)
        toolbar.setIconSize(QSize(16, 16))
        toolbar.setObjectName("%sToolBar" % title)
        # toolbar.setOrientation(Qt.Vertical)
        toolbar.setToolButtonStyle(Qt.ToolButtonIconOnly)
        toolbar.setMovable(False)
        if actions:
            utils.addActions(toolbar, actions)
        self.addToolBar(Qt.TopToolBarArea, toolbar)
        toolbar.setAllowedAreas(Qt.NoToolBarArea)
        return toolbar

    # Support Functions

    def noShapes(self):
        return not len(self.labelList)

    def populateModeActions(self):
        tool, menu = self.actions.tool, self.actions.menu
        # self.tools.clear()
        # utils.addActions(self.tools, tool)
        self.canvas.menus[0].clear()
        utils.addActions(self.canvas.menus[0], menu)
        self.menus.edit.clear()
        actions = (
            self.actions.createRectangleMode,
            self.actions.createMode,
            self.actions.createPointMode,
            self.actions.editMode,
        )
        utils.addActions(self.menus.edit, actions + self.actions.editMenu)

    def setDirty(self):
        # Even if we autosave the file, we keep the ability to undo
        self.actions.undo.setEnabled(self.canvas.isShapeRestorable)
        self.tool_bar_widget.undo.setEnabled(self.canvas.isShapeRestorable)

        if self._config["auto_save"] or self.actions.saveAuto.isChecked():
            label_file = osp.splitext(self.imagePath)[0] + ".json"
            if self.output_dir:
                label_file_without_path = osp.basename(label_file)
                label_file = osp.join(self.output_dir, label_file_without_path)
            self.saveLabels(label_file)
            return
        self.dirty = True
        self.actions.save.setEnabled(True)
        self.global_widget.save_file.setEnabled(True)
        title = __appname__
        if self.filename is not None:
            title = "{} - {}*".format(title, self.filename)
        self.setWindowTitle(self.tr(title))

    def setClean(self):
        self.dirty = False
        self.actions.save.setEnabled(False)
        self.global_widget.save_file.setEnabled(False)
        if self.model_no == self.tr("Classification"):
            pass
        elif self.model_no == self.tr("Detection"):
            self.actions.createRectangleMode.setEnabled(True)
        elif self.model_no == self.tr("Segmentation"):
            self.actions.createMode.setEnabled(True)
        elif self.model_no == self.tr("Keypoint"):
            self.actions.createMode.setEnabled(True)
            self.actions.createPointMode.setEnabled(True)
        title = __appname__
        if self.filename is not None:
            title = "{} - {}".format(title, self.filename)
        self.setWindowTitle(self.tr(title))

        if self.hasLabelFile():
            self.actions.deleteFile.setEnabled(True)
            self.global_widget.delete_file.setEnabled(True)
        else:
            self.actions.deleteFile.setEnabled(False)
            self.global_widget.delete_file.setEnabled(False)

    def toggleActions(self, value=True):
        """Enable/Disable widgets which depend on an opened image."""
        for z in self.actions.zoomActions:
            z.setEnabled(value)
        for action in self.actions.onLoadActive:
            action.setEnabled(value)
        self.closeMode()

    def queueEvent(self, function):
        QtCore.QTimer.singleShot(0, function)

    def status(self, message, delay=5000):
        self.statusBar().showMessage(message)

    def resetState(self):
        self.labelList.clear()
        self.filename = None
        self.imagePath = None
        self.imageData = None
        self.labelFile = None
        self.otherData = None
        self.canvas.resetState()

    def currentItem(self):
        items = self.labelList.selectedItems()
        if items:
            return items[0]
        return None

    def addRecentFile(self, filename):
        filename = self.filename
        if filename in self.recentFiles:
            self.recentFiles.remove(filename)
        elif len(self.recentFiles) >= self.maxRecent:
            self.recentFiles.pop()
        self.recentFiles.insert(0, filename)
        if len(self.imageList) > 1:
            self.tool_bar_widget.prev.setEnabled(True)
            self.tool_bar_widget.next.setEnabled(True)
            self.actions.openNextImg.setEnabled(True)
            self.actions.openPrevImg.setEnabled(True)

    # Callbacks

    def undoShapeEdit(self):
        self.canvas.restoreShape()
        self.labelList.clear()
        self.loadShapes(self.canvas.shapes)
        self.actions.undo.setEnabled(self.canvas.isShapeRestorable)
        self.tool_bar_widget.undo.setEnabled(self.canvas.isShapeRestorable)
        if self.actions.saveAuto.isChecked():
            self.saveFile()

    def tutorial(self):
        help = self.version_data.get("help")
        webbrowser.open(help)

    def convertModel(self):
        self.showConvertModel()

    def toggleDrawingSensitive(self, drawing=True):
        """Toggle drawing sensitive.

        In the middle of drawing, toggling between modes should be disabled.
        """
        self.actions.editMode.setEnabled(not drawing)
        self.actions.undoLastPoint.setEnabled(drawing)
        self.actions.undo.setEnabled(not drawing)
        self.tool_bar_widget.undo.setEnabled(not drawing)
        self.actions.delete.setEnabled(not drawing)

    def choose_one(self, choose):
        for button in self.tool_bar_list:
            if choose == button:
                choose.setChecked(True)

    def close_other(self, choose):
        for button in self.tool_bar_list:
            if choose != button:
                button.setChecked(False)

    def toggleDrawMode(self, edit=True, createMode="polygon"):
        self.canvas.setEditing(edit)
        self.canvas.createMode = createMode
        if edit:
            self.labelSelectionChanged()
            self.choose_one(self.tool_bar_widget.edit)
            if self.tool_bar_widget.edit.isChecked():
                self.close_other(self.tool_bar_widget.edit)
                self.actions.createMode.setEnabled(True)
                self.actions.createRectangleMode.setEnabled(True)
                self.actions.createCircleMode.setEnabled(True)
                self.actions.createLineMode.setEnabled(True)
                self.actions.createPointMode.setEnabled(True)
                self.actions.createLineStripMode.setEnabled(True)
            else:
                self.tool_bar_widget.edit.setChecked(False)
                self.canvas.setEditing(True)
        else:
            if createMode == "polygon":
                self.choose_one(self.tool_bar_widget.polygon)
                if self.tool_bar_widget.polygon.isChecked():
                    self.close_other(self.tool_bar_widget.polygon)
                    self.actions.createMode.setEnabled(False)
                    self.actions.createRectangleMode.setEnabled(True)
                    self.actions.createCircleMode.setEnabled(True)
                    self.actions.createLineMode.setEnabled(True)
                    self.actions.createPointMode.setEnabled(True)
                    self.actions.createLineStripMode.setEnabled(True)
                else:
                    self.tool_bar_widget.polygon.setChecked(False)
                    self.canvas.setEditing(True)
            elif createMode == "rectangle":
                self.choose_one(self.tool_bar_widget.rectangle)
                if self.tool_bar_widget.rectangle.isChecked():
                    self.close_other(self.tool_bar_widget.rectangle)
                    self.actions.createMode.setEnabled(True)
                    self.actions.createRectangleMode.setEnabled(False)
                    self.actions.createCircleMode.setEnabled(True)
                    self.actions.createLineMode.setEnabled(True)
                    self.actions.createPointMode.setEnabled(True)
                    self.actions.createLineStripMode.setEnabled(True)
                else:
                    self.tool_bar_widget.rectangle.setChecked(False)
                    self.canvas.setEditing(True)
            elif createMode == "line":
                self.actions.createMode.setEnabled(True)
                self.actions.createRectangleMode.setEnabled(True)
                self.actions.createCircleMode.setEnabled(True)
                self.actions.createLineMode.setEnabled(False)
                self.actions.createPointMode.setEnabled(True)
                self.actions.createLineStripMode.setEnabled(True)
            elif createMode == "point":
                self.choose_one(self.tool_bar_widget.point)
                if self.tool_bar_widget.point.isChecked():
                    self.close_other(self.tool_bar_widget.point)
                    self.actions.createMode.setEnabled(True)
                    self.actions.createRectangleMode.setEnabled(True)
                    self.actions.createCircleMode.setEnabled(True)
                    self.actions.createLineMode.setEnabled(True)
                    self.actions.createPointMode.setEnabled(False)
                    self.actions.createLineStripMode.setEnabled(True)
                else:
                    self.tool_bar_widget.point.setChecked(False)
                    self.canvas.setEditing(True)
            elif createMode == "circle":
                self.actions.createMode.setEnabled(True)
                self.actions.createRectangleMode.setEnabled(True)
                self.actions.createCircleMode.setEnabled(False)
                self.actions.createLineMode.setEnabled(True)
                self.actions.createPointMode.setEnabled(True)
                self.actions.createLineStripMode.setEnabled(True)
            elif createMode == "linestrip":
                self.actions.createMode.setEnabled(True)
                self.actions.createRectangleMode.setEnabled(True)
                self.actions.createCircleMode.setEnabled(True)
                self.actions.createLineMode.setEnabled(True)
                self.actions.createPointMode.setEnabled(True)
                self.actions.createLineStripMode.setEnabled(False)
            else:
                raise ValueError("Unsupported createMode: %s" % createMode)
        self.actions.editMode.setEnabled(not edit)
        self.closeMode()

    def closeMode(self):
        if self.model_no == self.tr("Classification"):
            self.actions.createMode.setEnabled(False)
            self.actions.createRectangleMode.setEnabled(False)
            self.actions.createPointMode.setEnabled(False)
            self.actions.editMode.setEnabled(False)
            self.tool_bar_widget.rectangle.setEnabled(False)
            self.tool_bar_widget.polygon.setEnabled(False)
            self.tool_bar_widget.point.setEnabled(False)
            self.tool_bar_widget.edit.setEnabled(False)
        elif self.model_no == self.tr("Detection"):
            self.actions.createMode.setEnabled(False)
            self.actions.createPointMode.setEnabled(False)
            self.tool_bar_widget.polygon.setEnabled(False)
            self.tool_bar_widget.point.setEnabled(False)
        elif self.model_no == self.tr("Segmentation"):
            self.actions.createRectangleMode.setEnabled(False)
            self.actions.createPointMode.setEnabled(False)
            self.tool_bar_widget.rectangle.setEnabled(False)
            self.tool_bar_widget.point.setEnabled(False)
        elif self.model_no == self.tr("Keypoint"):
            self.actions.createRectangleMode.setEnabled(False)
            self.tool_bar_widget.rectangle.setEnabled(False)

    def setEditMode(self):
        self.toggleDrawMode(True)

    def updateFileMenu(self):
        current = self.filename

        def exists(filename):
            return osp.exists(str(filename))

        menu = self.menus.recentFiles
        menu.clear()
        files = [f for f in self.recentFiles if f != current and exists(f)]
        for i, f in enumerate(files):
            icon = utils.newIcon("labels")
            action = QtWidgets.QAction(
                icon, "&%d %s" % (i + 1, QtCore.QFileInfo(f).fileName()), self
            )
            action.triggered.connect(functools.partial(self.loadRecent, f))
            menu.addAction(action)

    def popLabelListMenu(self, point):
        self.menus.labelList.exec_(self.labelList.mapToGlobal(point))

    def validateLabel(self, label):
        # no validation
        if self._config["validate_label"] is None:
            return True

        for i in range(self.uniqLabelList.count()):
            label_i = self.uniqLabelList.item(i).data(Qt.UserRole)
            if self._config["validate_label"] in ["exact"]:
                if label_i == label:
                    return True
        return False

    def editLabel(self, item=None):
        if item and not isinstance(item, LabelListWidgetItem):
            raise TypeError("item must be LabelListWidgetItem type")

        if not self.canvas.editing():
            return
        if not item:
            item = self.currentItem()
        if item is None:
            return
        shape = item.shape()
        if shape is None:
            return
        text, flags, group_id = self.labelDialog.popUp(
            text=shape.label,
            flags=shape.flags,
            group_id=shape.group_id,
        )
        if text is None:
            return
        if not self.validateLabel(text):
            self.errorMessage(
                self.tr("Invalid label"),
                self.tr("Invalid label '{}' with validation type '{}'").format(
                    text, self._config["validate_label"]
                ),
            )
            return
        shape.label = text
        shape.flags = flags
        shape.group_id = group_id

        if shape.group_id is None:
            item.setText(
                '{}'.format(
                    shape.label
                )
            )
        else:
            item.setText("{} ({})".format(shape.label, shape.group_id))
        self.setDirty()
        if not self.uniqLabelList.findItemsByLabel(shape.label):
            self.labelList.clear()
            item = QtWidgets.QListWidgetItem()
            item.setData(Qt.UserRole, shape.label)
            self.uniqLabelList.addItem(item)
            rgb = self._get_rgb_by_label(shape.label)
            self.uniqLabelList.setItemLabel(item, shape.label, rgb)
            self.loadShapes(self.canvas.shapes)
            self.labelList.selectItem(self.labelList.findItemByShape(shape))
        else:
            self._update_shape_color(shape)

    def fileSearchChanged(self):
        self.importDirImages(
            self.lastOpenDir,
            pattern=self.fileSearch.text(),
            load=False,
        )

    def fileSelectionChanged(self):
        def _fileSelectionChanged():
            items = self.fileListWidget.selectedItems()
            if not items:
                return
            item = items[0]
            self.fileListWidget.setToolTip(item.text())
            currIndex = self.imageList.index(str(item.text()))
            if currIndex < len(self.imageList):
                filename = self.imageList[currIndex]
                if filename:
                    self.loadFile(filename)
            self.canvas.deSelectShape()
        def _save():
            self.saveFile()
            _fileSelectionChanged()
        def _discard():
            _fileSelectionChanged()
        def _close():
            self.fileListWidget.setCurrentRow(self.imageList.index(self.filename))
            self.save_notes.close()
        if not self.dirty:
            _fileSelectionChanged()
        else:
            msg = self.tr('Save annotations to "{}" before closing?').format(
                self.filename
            )
            self.save_notes = YesCancelNote(self.tr("WARNING"), msg)
            self.save_notes.yes.clicked.connect(_save)
            self.save_notes.cancel.clicked.connect(_discard)
            self.save_notes.close_button.clicked.connect(_close)
            self.save_notes.show()

    # React to canvas signals.
    def shapeSelectionChanged(self, selected_shapes):
        self._noSelectionSlot = True
        for shape in self.canvas.selectedShapes:
            shape.selected = False
        self.labelList.clearSelection()
        self.canvas.selectedShapes = selected_shapes
        for shape in self.canvas.selectedShapes:
            shape.selected = True
            item = self.labelList.findItemByShape(shape)
            self.labelList.selectItem(item)
            self.labelList.scrollToItem(item)
        self._noSelectionSlot = False
        n_selected = len(selected_shapes)
        self.actions.delete.setEnabled(n_selected)
        self.actions.duplicate.setEnabled(n_selected)
        self.actions.copy.setEnabled(n_selected)
        self.tool_bar_widget.copy.setEnabled(n_selected)
        self.tool_bar_widget.delete_2.setEnabled(n_selected)
        self.actions.edit.setEnabled(n_selected == 1)

    def addLabel(self, shape):
        if shape.group_id is None:
            text = shape.label
        else:
            text = "{} ({})".format(shape.label, shape.group_id)
        label_list_item = LabelListWidgetItem(text, shape)
        self.labelList.addItem(label_list_item)
        if not self.uniqLabelList.findItemsByLabel(shape.label):
            item = self.uniqLabelList.createItemFromLabel(shape.label)
            self.uniqLabelList.addItem(item)
            rgb = self._get_rgb_by_label(shape.label)
            self.uniqLabelList.setItemLabel(item, shape.label, rgb)
        self.labelDialog.addLabelHistory(shape.label)
        for action in self.actions.onShapesPresent:
            action.setEnabled(True)

        self._update_shape_color(shape)
        label_list_item.setText(
            '{}'.format(
                text
            )
        )

    def _update_shape_color(self, shape):
        r, g, b = self._get_rgb_by_label(shape.label)
        shape.line_color = QtGui.QColor(r, g, b)
        shape.vertex_fill_color = QtGui.QColor(r, g, b)
        shape.hvertex_fill_color = QtGui.QColor(255, 255, 255)
        shape.fill_color = QtGui.QColor(r, g, b, 128)
        shape.select_line_color = QtGui.QColor(255, 255, 255)
        shape.select_fill_color = QtGui.QColor(r, g, b, 155)

    def _get_rgb_by_label(self, label):
        if self._config["shape_color"] == "auto":
            item = self.uniqLabelList.findItemsByLabel(label)[0]
            label_id = self.uniqLabelList.indexFromItem(item).row() + 1
            label_id += self._config["shift_auto_shape_color"]
            return LABEL_COLORMAP[label_id % len(LABEL_COLORMAP)]
        elif (
            self._config["shape_color"] == "manual"
            and self._config["label_colors"]
            and label in self._config["label_colors"]
        ):
            return self._config["label_colors"][label]
        elif self._config["default_shape_color"]:
            return self._config["default_shape_color"]
        return (0, 255, 0)

    def remLabels(self, shapes):
        for shape in shapes:
            item = self.labelList.findItemByShape(shape)
            self.labelList.removeItem(item)

    def loadShapes(self, shapes, replace=True):
        self._noSelectionSlot = True
        for shape in shapes:
            self.addLabel(shape)
        self.labelList.clearSelection()
        self._noSelectionSlot = False
        self.canvas.loadShapes(shapes, replace=replace)

    def loadLabels(self, shapes):
        s = []
        for shape in shapes:
            label = shape["label"]
            points = shape["points"]
            shape_type = shape["shape_type"]
            flags = shape["flags"]
            group_id = shape["group_id"]
            other_data = shape["other_data"]

            if not points:
                # skip point-empty shape
                continue

            shape = Shape(
                label=label,
                shape_type=shape_type,
                group_id=group_id,
            )
            for x, y in points:
                shape.addPoint(QtCore.QPointF(x, y))
            shape.close()

            default_flags = {}
            if self._config["label_flags"]:
                for pattern, keys in self._config["label_flags"].items():
                    if re.match(pattern, label):
                        for key in keys:
                            default_flags[key] = False
            shape.flags = default_flags
            shape.flags.update(flags)
            shape.other_data = other_data

            s.append(shape)
        self.loadShapes(s)

    def loadFlags(self, flags):
        self.flag_widget.clear()
        for key, flag in flags.items():
            item = QtWidgets.QListWidgetItem(key)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Checked if flag else Qt.Unchecked)
            self.flag_widget.addItem(item)

    def saveLabels(self, filename):
        lf = LabelFile()

        def format_shape(s):
            data = s.other_data.copy()
            data.update(
                dict(
                    label=s.label.encode("utf-8") if PY2 else s.label,
                    points=[(p.x(), p.y()) for p in s.points],
                    group_id=s.group_id,
                    shape_type=s.shape_type,
                    flags=s.flags,
                )
            )
            return data

        shapes = [format_shape(item.shape()) for item in self.labelList]
        flags = {}
        for i in range(self.flag_widget.count()):
            item = self.flag_widget.item(i)
            key = item.text()
            flag = item.checkState() == Qt.Checked
            flags[key] = flag
        try:
            imagePath = osp.relpath(self.imagePath, osp.dirname(filename))
            imageData = self.imageData if self._config["store_data"] else None
            if osp.dirname(filename) and not osp.exists(osp.dirname(filename)):
                os.makedirs(osp.dirname(filename))
            lf.save(
                filename=filename,
                shapes=shapes,
                imagePath=imagePath,
                imageData=imageData,
                imageHeight=self.image.height(),
                imageWidth=self.image.width(),
                otherData=self.otherData,
                flags=flags,
            )
            self.labelFile = lf
            items = self.fileListWidget.findItems(
                self.filename, Qt.MatchExactly
            )
            if len(items) > 0:
                if len(items) != 1:
                    raise RuntimeError("There are duplicate files.")
                items[0].setCheckState(Qt.Checked)
            # disable allows next and previous image to proceed
            # self.filename = filename
            return True
        except LabelFileError as e:
            self.errorMessage(
                self.tr("Error saving label data"), self.tr("<b>%s</b>") % e
            )
            return False

    def duplicateSelectedShape(self):
        added_shapes = self.canvas.duplicateSelectedShapes()
        self.labelList.clearSelection()
        for shape in added_shapes:
            self.addLabel(shape)
        self.setDirty()

    def pasteSelectedShape(self):
        self.loadShapes(self._copied_shapes, replace=False)
        self.setDirty()
        self.actions.paste.setEnabled(False)

    def copySelectedShape(self):
        self._copied_shapes = [s.copy() for s in self.canvas.selectedShapes]
        self.actions.paste.setEnabled(len(self._copied_shapes) > 0)

    def labelSelectionChanged(self):
        if self._noSelectionSlot:
            return
        if self.canvas.editing():
            selected_shapes = []
            for item in self.labelList.selectedItems():
                selected_shapes.append(item.shape())
            if selected_shapes:
                self.canvas.selectShapes(selected_shapes)
            else:
                self.canvas.deSelectShape()

    def labelItemChanged(self, item):
        shape = item.shape()
        font = QtGui.QFont()
        font.setFamily("HarmonyOS Sans SC Medium")
        font.setPointSize(9)
        item.setFont(font)
        self.canvas.setShapeVisible(shape, item.checkState() == Qt.Checked)

    def labelOrderChanged(self):
        self.setDirty()
        self.canvas.loadShapes([item.shape() for item in self.labelList])

    # Callback functions:

    def newShape(self):
        """Pop-up and give focus to the label editor.

        position MUST be in global coordinates.
        """
        items = self.uniqLabelList.selectedItems()
        text = None
        if items:
            text = items[0].data(Qt.UserRole)
        flags = {}
        group_id = None
        if self._config["display_label_popup"] or not text:
            previous_text = self.labelDialog.edit.text()
            text, flags, group_id = self.labelDialog.popUp(text)
            if not text:
                self.labelDialog.edit.setText(previous_text)

        if text and not self.validateLabel(text):
            self.errorMessage(
                self.tr("Invalid label"),
                self.tr("Invalid label '{}' with validation type '{}'").format(
                    text, self._config["validate_label"]
                ),
            )
            text = ""
        if text:
            self.labelList.clearSelection()
            shape = self.canvas.setLastLabel(text, flags)
            shape.group_id = group_id
            self.addLabel(shape)
            self.actions.editMode.setEnabled(True)
            self.actions.undoLastPoint.setEnabled(False)
            self.actions.undo.setEnabled(True)
            self.tool_bar_widget.undo.setEnabled(True)
            self.setDirty()
        else:
            self.canvas.undoLastLine()
            self.canvas.shapesBackups.pop()

    def scrollRequest(self, delta, orientation):
        units = -delta * 0.1  # natural scroll
        bar = self.scrollBars[orientation]
        value = bar.value() + bar.singleStep() * units
        self.setScroll(orientation, value)

    def setScroll(self, orientation, value):
        self.scrollBars[orientation].setValue(value)
        self.scroll_values[orientation][self.filename] = value

    def setZoom(self, value):
        self.actions.fitWidth.setChecked(False)
        self.actions.fitWindow.setChecked(False)
        self.zoomMode = self.MANUAL_ZOOM
        self.zoomWidget.setValue(value)
        self.tool_bar_widget.zoom.setValue(value)
        self.zoom_values[self.filename] = (self.zoomMode, value)

    def addZoom(self, increment=1.1):
        zoom_value = self.tool_bar_widget.zoom.value() * increment
        if increment > 1:
            zoom_value = math.ceil(zoom_value)
        else:
            zoom_value = math.floor(zoom_value)
        self.setZoom(zoom_value)

    def zoomRequest(self, delta, pos):
        canvas_width_old = self.canvas.width()
        units = 1.1
        if delta < 0:
            units = 0.9
        self.addZoom(units)

        canvas_width_new = self.canvas.width()
        if canvas_width_old != canvas_width_new:
            canvas_scale_factor = canvas_width_new / canvas_width_old

            x_shift = round(pos.x() * canvas_scale_factor) - pos.x()
            y_shift = round(pos.y() * canvas_scale_factor) - pos.y()

            self.setScroll(
                Qt.Horizontal,
                self.scrollBars[Qt.Horizontal].value() + x_shift,
            )
            self.setScroll(
                Qt.Vertical,
                self.scrollBars[Qt.Vertical].value() + y_shift,
            )

    def setFitWindow(self, value=True):
        if value:
            self.actions.fitWidth.setChecked(False)
        self.zoomMode = self.FIT_WINDOW if value else self.MANUAL_ZOOM
        self.adjustScale()

    def setFitWidth(self, value=True):
        if value:
            self.actions.fitWindow.setChecked(False)
        self.zoomMode = self.FIT_WIDTH if value else self.MANUAL_ZOOM
        self.adjustScale()

    def enableKeepPrevScale(self, enabled):
        self._config["keep_prev_scale"] = enabled
        self.actions.keepPrevScale.setChecked(enabled)

    def onNewBrightnessContrast(self, qimage):
        self.canvas.loadPixmap(
            QtGui.QPixmap.fromImage(qimage), clear_shapes=False
        )

    def brightnessContrast(self, value):
        self.choose_one(self.tool_bar_widget.color)
        dialog = BrightnessContrastDialog(
            utils.img_data_to_pil(self.imageData),
            self.onNewBrightnessContrast,
            parent=self,
        )
        brightness, contrast = self.brightnessContrast_values.get(
            self.filename, (None, None)
        )
        if brightness is not None:
            dialog.slider_brightness.setValue(brightness)
        if contrast is not None:
            dialog.slider_contrast.setValue(contrast)
        dialog.exec_()

        brightness = dialog.slider_brightness.value()
        contrast = dialog.slider_contrast.value()
        self.brightnessContrast_values[self.filename] = (brightness, contrast)

    def togglePolygons(self, value):
        for item in self.labelList:
            item.setCheckState(Qt.Checked if value else Qt.Unchecked)

    def loadFile(self, filename=None):
        """Load the specified file, or the last opened file if None."""
        # changing fileListWidget loads file
        if filename in self.imageList and (
            self.fileListWidget.currentRow() != self.imageList.index(filename)
        ):
            self.fileListWidget.setCurrentRow(self.imageList.index(filename))
            self.fileListWidget.repaint()
            return

        self.resetState()
        self.canvas.setEnabled(False)
        if filename is None:
            filename = self.settings.value("filename", "")
        filename = str(filename)
        if not QtCore.QFile.exists(filename):
            self.errorMessage(
                self.tr("Error opening file"),
                self.tr("No such file: <b>%s</b>") % filename,
            )
            return False
        # assumes same name, but json extension
        self.status(
            filename
        )
        label_file = osp.splitext(filename)[0] + ".json"
        if self.output_dir:
            label_file_without_path = osp.basename(label_file)
            label_file = osp.join(self.output_dir, label_file_without_path)
        if QtCore.QFile.exists(label_file) and LabelFile.is_label_file(
            label_file
        ):
            json_file = json.load(open(label_file, "r", encoding="utf-8"))
            imagename = json_file['imagePath']
            if imagename == filename.split('/')[-1]:
                try:
                    self.labelFile = LabelFile(label_file)
                except LabelFileError as e:
                    self.errorMessage(
                        self.tr("Error opening file"),
                        self.tr(
                            "<p><b>%s</b></p>"
                            "<p>Make sure <i>%s</i> is a valid label file."
                        )
                        % (e, label_file),
                    )
                    self.status(self.tr("Error reading %s") % label_file)
                    return False
                self.imageData = self.labelFile.imageData
                self.imagePath = osp.join(
                    osp.dirname(label_file),
                    self.labelFile.imagePath,
                )
                self.otherData = self.labelFile.otherData
            else:
                self.imageData = LabelFile.load_image_file(filename)
                if self.imageData:
                    self.imagePath = filename
                self.labelFile = None
        else:
            self.imageData = LabelFile.load_image_file(filename)
            if self.imageData:
                self.imagePath = filename
            self.labelFile = None
        image = QtGui.QImage.fromData(self.imageData)

        if image.isNull():
            formats = [
                "*.{}".format(fmt.data().decode())
                for fmt in QtGui.QImageReader.supportedImageFormats()
            ]
            self.errorMessage(
                self.tr("Error opening file"),
                self.tr(
                    "<p>Make sure <i>{0}</i> is a valid image file.<br/>"
                    "Supported image formats: {1}</p>"
                ).format(filename, ",".join(formats)),
            )
            self.status(self.tr("Error reading %s") % filename)
            return False
        self.image = image
        self.filename = filename
        if self._config["keep_prev"]:
            prev_shapes = self.canvas.shapes
        self.canvas.loadPixmap(QtGui.QPixmap.fromImage(image))
        flags = {k: False for k in self._config["flags"] or []}
        if self.labelFile:
            self.loadLabels(self.labelFile.shapes)
            if self.labelFile.flags is not None:
                flags.update(self.labelFile.flags)
        self.loadFlags(flags)
        if self._config["keep_prev"] and self.noShapes():
            self.loadShapes(prev_shapes, replace=False)
            self.setDirty()
        else:
            self.setClean()
        self.canvas.setEnabled(True)
        # set zoom values
        is_initial_load = not self.zoom_values
        if self.filename in self.zoom_values:
            self.zoomMode = self.zoom_values[self.filename][0]
            self.setZoom(self.zoom_values[self.filename][1])
        elif is_initial_load or not self._config["keep_prev_scale"]:
            self.adjustScale(initial=True)
        # set scroll values
        for orientation in self.scroll_values:
            if self.filename in self.scroll_values[orientation]:
                self.setScroll(
                    orientation, self.scroll_values[orientation][self.filename]
                )
        # set brightness contrast values
        dialog = BrightnessContrastDialog(
            utils.img_data_to_pil(self.imageData),
            self.onNewBrightnessContrast,
            parent=self,
        )
        brightness, contrast = self.brightnessContrast_values.get(
            self.filename, (None, None)
        )
        if self._config["keep_prev_brightness"] and self.recentFiles:
            brightness, _ = self.brightnessContrast_values.get(
                self.recentFiles[0], (None, None)
            )
        if self._config["keep_prev_contrast"] and self.recentFiles:
            _, contrast = self.brightnessContrast_values.get(
                self.recentFiles[0], (None, None)
            )
        if brightness is not None:
            dialog.slider_brightness.setValue(brightness)
        if contrast is not None:
            dialog.slider_contrast.setValue(contrast)
        self.brightnessContrast_values[self.filename] = (brightness, contrast)
        if brightness is not None or contrast is not None:
            dialog.onNewValue(None)
        self.paintCanvas()
        self.addRecentFile(self.filename)
        self.toggleActions(True)
        self.canvas.setFocus()
        self.status(filename)
        return True

    def resizeEvent(self, event):
        if (
            self.canvas
            and not self.image.isNull()
            and self.zoomMode != self.MANUAL_ZOOM
        ):
            self.adjustScale()
        super(MainWindow, self).resizeEvent(event)

    def paintCanvas(self):
        assert not self.image.isNull(), "cannot paint null image"
        self.canvas.scale = 0.01 * self.tool_bar_widget.zoom.value()
        self.canvas.adjustSize()
        self.canvas.update()

    def adjustScale(self, initial=False):
        value = self.scalers[self.FIT_WINDOW if initial else self.zoomMode]()
        value = int(100 * value)
        self.tool_bar_widget.zoom.setValue(value)
        self.zoomWidget.setValue(value)
        self.zoom_values[self.filename] = (self.zoomMode, value)

    def scaleFitWindow(self):
        """Figure out the size of the pixmap to fit the main widget."""
        e = 2.0  # So that no scrollbars are generated.
        w1 = self.centralWidget().width() - e
        h1 = self.centralWidget().height() - e
        a1 = w1 / h1
        # Calculate a new scale value based on the pixmap's aspect ratio.
        w2 = self.canvas.pixmap.width() - 0.0
        h2 = self.canvas.pixmap.height() - 0.0
        a2 = w2 / h2
        return w1 / w2 if a2 >= a1 else h1 / h2

    def scaleFitWidth(self):
        # The epsilon does not seem to work too well here.
        w = self.centralWidget().width() - 2.0
        return w / self.canvas.pixmap.width()

    def enableSaveImageWithData(self, enabled):
        self._config["store_data"] = enabled
        self.actions.saveWithImageData.setChecked(enabled)

    def closeLabelme(self):
        # flag = True
        # if not self.mayContinue():
        #     flag = False
        def _closeLabelme():
            self.settings.setValue(
                "filename", self.filename if self.filename else ""
            )
            self.settings.setValue("window/size", self.size())
            self.settings.setValue("window/position", self.pos())
            self.settings.setValue("window/state", self.saveState())
            self.settings.setValue("recentFiles", self.recentFiles)

            # ask the use for where to save the labels
            # self.settings.setValue('window/geometry', self.saveGeometry())
            def thread_terminate():
                self.convert_win.thread.terminate()
                self.convert_win.thread = None
                if os.path.exists(self.convert_win.output_data):
                    shutil.rmtree(self.convert_win.output_data)
                self.final_win.close()
                self.model_task_win.show()
                self.close()
                self.convert_win.close()

            if self.convert_win and self.convert_win.thread:
                self.final_win.showMinimized()
                self.convert_win.close_notes = YesCancelNote(self.tr("WARNING"),
                                                             self.tr(
                                                                 "You are terminating the process. Are you sure to quit?"))
                self.convert_win.close_notes.yes.clicked.connect(thread_terminate)
                self.convert_win.close_notes.show()
            else:
                self.labelme_title.close()
                self.final_win.close()
                self.close()
                self.model_task_win.show()
                if self.convert_win:
                    self.convert_win.close()
        def _save():
            self.saveFile()
            self.fileSelectionChanged()
            _closeLabelme()
        def _discard():
            self.loadFile(self.filename)
            _closeLabelme()
        def _close():
            self.save_notes.close()
            self.fileListWidget.setCurrentRow(self.imageList.index(self.filename))
        if not self.dirty:
            _closeLabelme()
        else:
            msg = self.tr('Save annotations to "{}" before closing?').format(
                self.filename
            )
            self.save_notes = YesCancelNote(self.tr("WARNING"), msg)
            self.save_notes.yes.clicked.connect(_save)
            self.save_notes.cancel.clicked.connect(_discard)
            self.save_notes.close_button.clicked.connect(_close)
            self.save_notes.show()


    def dragEnterEvent(self, event):
        extensions = [
            ".%s" % fmt.data().decode().lower()
            for fmt in QtGui.QImageReader.supportedImageFormats()
        ]
        if event.mimeData().hasUrls():
            items = [i.toLocalFile() for i in event.mimeData().urls()]
            if any([i.lower().endswith(tuple(extensions)) for i in items]):
                event.accept()
        else:
            event.ignore()

    def dropEvent(self, event):
        if not self.mayContinue():
            event.ignore()
            return
        items = [i.toLocalFile() for i in event.mimeData().urls()]
        self.importDroppedImageFiles(items)

    # User Dialogs #

    def loadRecent(self, fileName):
        suffix = fileName.split('/')[-1].split('.')[-1].lower()
        externs = ['png', 'jpg', 'jpeg', 'bmp', 'webp']
        for ex in externs:
            if fileName not in self.imageList and (fileName.replace(fileName.split('.')[-1], '') + ex) in self.imageList:
                self.notes = Notes(self.tr("ERROR"),
                                   self.tr("Cannot open a file with the same name in the same directory."), "ERROR")
                self.notes.show()
                return
        if fileName and fileName not in self.imageList:
            if suffix not in externs:
                self.notes = Notes(self.tr("ERROR"), self.tr("It only supports picture of format in jpg/png/jpeg/bmp/webp."), "ERROR")
                self.notes.show()
                return
            label_file = osp.splitext(fileName)[0] + ".json"
            if self.output_dir:
                label_file_without_path = osp.basename(label_file)
                label_file = osp.join(self.output_dir, label_file_without_path)
            item = QtWidgets.QListWidgetItem(fileName)
            item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
            if QtCore.QFile.exists(label_file) and LabelFile.is_label_file(
                    label_file
            ):
                json_file = json.load(open(label_file, "r", encoding="utf-8"))
                imagename = json_file['imagePath']
                if imagename == fileName.split('/')[-1]:
                    item.setCheckState(Qt.Checked)
                else:
                    item.setCheckState(Qt.Unchecked)
            else:
                item.setCheckState(Qt.Unchecked)
            self.fileListWidget.addItem(item)
            self.imageList.append(fileName)
            if len(self.imageList) > 1:
                self.tool_bar_widget.prev.setEnabled(True)
                self.tool_bar_widget.next.setEnabled(True)
        if self.mayContinue():
            self.loadFile(fileName)

    def openPrevImg(self, _value=False):
        keep_prev = self._config["keep_prev"]
        if QtWidgets.QApplication.keyboardModifiers() == (
            Qt.ControlModifier | Qt.ShiftModifier
        ):
            self._config["keep_prev"] = True
        def _openPrevImg():
            if len(self.imageList) <= 0:
                return
            if self.filename is None:
                return
            currIndex = self.imageList.index(self.filename)
            if currIndex - 1 >= 0:
                filename = self.imageList[currIndex - 1]
                if filename:
                    self.loadFile(filename)
            self._config["keep_prev"] = keep_prev
        def _save():
            self.saveFile()
            self.fileSelectionChanged()
            _openPrevImg()
        def _discard():
            self.loadFile(self.filename)
            _openPrevImg()
        def _close():
            self.save_notes.close()
            self.fileListWidget.setCurrentRow(self.imageList.index(self.filename))
        if not self.dirty:
            _openPrevImg()
        else:
            msg = self.tr('Save annotations to "{}" before closing?').format(
                self.filename
            )
            self.save_notes = YesCancelNote(self.tr("WARNING"), msg)
            self.save_notes.yes.clicked.connect(_save)
            self.save_notes.cancel.clicked.connect(_discard)
            self.save_notes.close_button.clicked.connect(_close)
            self.save_notes.show()

    def openNextImg(self, _value=False, load=True):
        keep_prev = self._config["keep_prev"]
        if QtWidgets.QApplication.keyboardModifiers() == (
            Qt.ControlModifier | Qt.ShiftModifier
        ):
            self._config["keep_prev"] = True
        def _openNextImg():
            if len(self.imageList) <= 0:
                return
            filename = None
            if self.filename is None:
                filename = self.imageList[0]
            else:
                currIndex = self.imageList.index(self.filename)
                if currIndex + 1 < len(self.imageList):
                    filename = self.imageList[currIndex + 1]
                else:
                    filename = self.imageList[-1]
            self.filename = filename
            if self.filename and load:
                self.loadFile(self.filename)
            self._config["keep_prev"] = keep_prev
        def _save():
            self.saveFile()
            self.fileSelectionChanged()
            _openNextImg()
        def _discard():
            self.loadFile(self.filename)
            _openNextImg()
        def _close():
            self.save_notes.close()
            self.fileListWidget.setCurrentRow(self.imageList.index(self.filename))
        if not self.dirty:
            _openNextImg()
        else:
            msg = self.tr('Save annotations to "{}" before closing?').format(
                self.filename
            )
            self.save_notes = YesCancelNote(self.tr("WARNING"), msg)
            self.save_notes.yes.clicked.connect(_save)
            self.save_notes.cancel.clicked.connect(_discard)
            self.save_notes.close_button.clicked.connect(_close)
            self.save_notes.show()

    def openFile(self, _value=False, dirpath=None):
        if not self.mayContinue():
            return
        path = osp.dirname(str(self.filename)) if self.filename else "."
        formats = [
            "*.{}".format(fmt.data().decode())
            for fmt in QtGui.QImageReader.supportedImageFormats()
        ]
        defaultOpenPath = dirpath if dirpath else "."
        if self.lastOpenDir and osp.exists(self.lastOpenDir):
            defaultOpenPath = self.lastOpenDir
        else:
            defaultOpenPath = (
                osp.dirname(self.filename) if self.filename else "."
            )
        targetPath = QtWidgets.QFileDialog.getOpenFileName(None, self.tr("%s - Choose Image or Label file"), defaultOpenPath)
        fileName = targetPath[0]
        suffix = fileName.split('/')[-1].split('.')[-1].lower()
        externs = ['png', 'jpg', 'jpeg', 'bmp', 'webp']
        for ex in externs:
            if fileName not in self.imageList and (fileName.replace(fileName.split('.')[-1], '') + ex) in self.imageList:
                self.notes = Notes(self.tr("ERROR"),
                                   self.tr("Cannot open a file with the same name in the same directory."), "ERROR")
                self.notes.show()
                return
        if fileName and fileName not in self.imageList:
            if suffix not in externs:
                self.notes = Notes(self.tr("ERROR"), self.tr("It only supports picture of format in jpg/png/jpeg/bmp/webp."), "ERROR")
                self.notes.show()
                return
            if is_contains_chinese(fileName):
                self.notes = Notes(self.tr("ERROR"), self.tr("The path does not support Chinese"), "ERROR")
                self.notes.show()
                return
            label_file = osp.splitext(fileName)[0] + ".json"
            if self.output_dir:
                label_file_without_path = osp.basename(label_file)
                label_file = osp.join(self.output_dir, label_file_without_path)
            item = QtWidgets.QListWidgetItem(fileName)
            item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
            if QtCore.QFile.exists(label_file) and LabelFile.is_label_file(
                    label_file
            ):
                json_file = json.load(open(label_file, "r", encoding="utf-8"))
                imagename = json_file['imagePath']
                if imagename == fileName.split('/')[-1]:
                    item.setCheckState(Qt.Checked)
                else:
                    item.setCheckState(Qt.Unchecked)
            else:
                item.setCheckState(Qt.Unchecked)
            self.fileListWidget.addItem(item)
            self.imageList.append(fileName)
        if len(self.imageList) > 1:
            self.tool_bar_widget.prev.setEnabled(True)
            self.tool_bar_widget.next.setEnabled(True)
            self.actions.openNextImg.setEnabled(True)
            self.actions.openPrevImg.setEnabled(True)
        if fileName:
            self.loadFile(fileName)

    def changeOutputDirDialog(self, _value=False):
        default_output_dir = self.output_dir
        if default_output_dir is None and self.filename:
            default_output_dir = osp.dirname(self.filename)
        if default_output_dir is None:
            default_output_dir = self.currentPath()

        output_dir = QtWidgets.QFileDialog.getExistingDirectory(
            self,
            self.tr("%s - Save/Load Annotations in Directory") % __appname__,
            default_output_dir,
            QtWidgets.QFileDialog.ShowDirsOnly
            | QtWidgets.QFileDialog.DontResolveSymlinks,
        )
        if is_contains_chinese(output_dir):
            self.notes = Notes(self.tr("ERROR"), self.tr("The path does not support Chinese"), "ERROR")
            self.notes.show()
            return
        else:
            output_dir = str(output_dir)

        if not output_dir:
            return

        self.output_dir = output_dir

        self.statusBar().showMessage(
            self.tr("%s . Annotations will be saved/loaded in %s")
            % ("Change Annotations Dir", self.output_dir)
        )
        self.statusBar().show()

        current_filename = self.filename
        self.importDirImages(self.lastOpenDir, load=False)

        if current_filename in self.imageList:
            # retain currently selected file
            self.fileListWidget.setCurrentRow(
                self.imageList.index(current_filename)
            )
            self.fileListWidget.repaint()

    def saveFile(self, _value=False):
        assert not self.image.isNull(), "cannot save empty image"
        if self.labelFile:
            # DL20180323 - overwrite when in directory
            self._saveFile(self.labelFile.filename)
        elif self.output_file:
            self._saveFile(self.output_file)
            self.close()
        else:
            self._saveFile(self.filename.replace(self.filename.split('.')[-1], 'json'))

    def saveFileAs(self, _value=False):
        assert not self.image.isNull(), "cannot save empty image"
        self._saveFile(self.saveFileDialog())

    def saveFileDialog(self):
        caption = self.tr("%s - Choose File") % __appname__
        filters = self.tr("Label files (*%s)") % LabelFile.suffix
        if self.output_dir:
            dlg = QtWidgets.QFileDialog(
                self, caption, self.output_dir, filters
            )
        else:
            dlg = QtWidgets.QFileDialog(
                self, caption, self.currentPath(), filters
            )
        dlg.setDefaultSuffix(LabelFile.suffix[1:])
        dlg.setAcceptMode(QtWidgets.QFileDialog.AcceptSave)
        dlg.setOption(QtWidgets.QFileDialog.DontConfirmOverwrite, False)
        dlg.setOption(QtWidgets.QFileDialog.DontUseNativeDialog, False)
        basename = osp.basename(osp.splitext(self.filename)[0])
        if self.output_dir:
            default_labelfile_name = osp.join(
                self.output_dir, basename + LabelFile.suffix
            )
        else:
            default_labelfile_name = osp.join(
                self.currentPath(), basename + LabelFile.suffix
            )
        filename = dlg.getSaveFileName(
            self,
            self.tr("Choose File"),
            default_labelfile_name,
            self.tr("Label files (*%s)") % LabelFile.suffix,
        )
        if isinstance(filename, tuple):
            filename, _ = filename
        return filename

    def _saveFile(self, filename):
        if is_contains_chinese(filename):
            self.notes = Notes(self.tr("ERROR"), self.tr("The path does not support Chinese"), "ERROR")
            self.notes.show()
            return
        if filename and self.saveLabels(filename):
            self.addRecentFile(filename)
            self.setClean()

    def closeFile(self, _value=False):
        def _closeFile():
            self.resetState()
            self.setClean()
            self.fileListWidget.takeItem(self.fileListWidget.currentRow())
            # self.canvas.setEnabled(False)
            if self.fileListWidget.count() != 0:
                self.actions.close.setEnabled(True)
                if self.fileListWidget.count() == 1:
                    self.tool_bar_widget.prev.setEnabled(False)
                    self.tool_bar_widget.next.setEnabled(False)
                    self.actions.openNextImg.setEnabled(False)
                    self.actions.openPrevImg.setEnabled(False)
            else:
                self.actions.close.setEnabled(False)
                for button in self.tool_bar_list:
                    button.setEnabled(False)
                    button.setChecked(False)
                for action in self.actions.menu:
                    action.setEnabled(False)
                for action in self.actions.views:
                    action.setEnabled(False)
        def _save():
            self.saveFile()
            self.fileSelectionChanged()
            _closeFile()
        def _discard():
            self.loadFile(self.filename)
            _closeFile()
        if not self.dirty:
            _closeFile()
        else:
            msg = self.tr('Save annotations to "{}" before closing?').format(
                self.filename
            )
            self.save_notes = YesCancelNote(self.tr("WARNING"), msg)
            self.save_notes.yes.clicked.connect(_save)
            self.save_notes.cancel.clicked.connect(_discard)
            self.save_notes.show()

    def getLabelFile(self):
        if self.filename.lower().endswith(".json"):
            label_file = self.filename
        else:
            label_file = osp.splitext(self.filename)[0] + ".json"

        return label_file

    def deleteFile(self):
        def _delete():
            label_file = self.getLabelFile()
            if osp.exists(label_file):
                os.remove(label_file)
                logger.info("Label file is removed: {}".format(label_file))
                item = self.fileListWidget.currentItem()
                item.setCheckState(Qt.Unchecked)
                imageData = LabelFile.load_image_file(self.filename)
                image = QtGui.QImage.fromData(imageData)
                self.canvas.loadPixmap(QtGui.QPixmap.fromImage(image), clear_shapes=True)
                self.labelList.clear()
                self.global_widget.delete_file.setEnabled(False)
                self.actions.deleteFile.setEnabled(False)

        msg = self.tr(
            "You are about to permanently delete this label file, "
            "proceed anyway?"
        )
        self.delete_notes = YesCancelNote(self.tr("WARNING"), msg)
        self.delete_notes.yes.clicked.connect(_delete)
        self.delete_notes.show()

    # Message Dialogs. #
    def hasLabels(self):
        if self.noShapes():
            self.errorMessage(
                "No objects labeled",
                "You must label at least one object to save the file.",
            )
            return False
        return True

    def hasLabelFile(self):
        if self.filename is None:
            return False

        label_file = self.getLabelFile()
        return osp.exists(label_file)

    def mayContinue(self):
        if not self.dirty:
            return True
        def _save():
            self.saveFile()
            self.fileSelectionChanged()
            return True
        def _discard():
            self.loadFile(self.filename)
            return False
        msg = self.tr('Save annotations to "{}" before closing?').format(
            self.filename
        )
        self.save_notes = YesCancelNote(self.tr("WARNING"), msg)
        self.save_notes.yes.clicked.connect(_save)
        self.save_notes.cancel.clicked.connect(_discard)
        self.save_notes.show()

    def errorMessage(self, title, message):
        return QtWidgets.QMessageBox.critical(
            self, title, "<p><b>%s</b></p>%s" % (title, message)
        )

    def currentPath(self):
        return osp.dirname(str(self.filename)) if self.filename else "."

    def toggleKeepPrevMode(self):
        self._config["keep_prev"] = not self._config["keep_prev"]

    def removeSelectedPoint(self):
        self.canvas.removeSelectedPoint()
        self.canvas.update()
        if not self.canvas.hShape.points:
            self.canvas.deleteShape(self.canvas.hShape)
            self.remLabels([self.canvas.hShape])
            self.setDirty()
            if self.noShapes():
                for action in self.actions.onShapesPresent:
                    action.setEnabled(False)

    def delLabel(self):
        self.uniqLabelList.clear()
        self.loadFile(self.filename)

    def deleteSelectedShape(self):
        def ok():
            self.remLabels(self.canvas.deleteSelected())
            self.setDirty()
            label_file = self.getLabelFile()
            if osp.exists(label_file):
                os.remove(label_file)
                item = self.fileListWidget.currentItem()
                item.setCheckState(Qt.Unchecked)
                self.tool_bar_widget.delete_2.setEnabled(False)
                self.tool_bar_widget.copy.setEnabled(False)
                self.tool_bar_widget.undo.setEnabled(False)
                self.global_widget.save_file.setEnabled(False)
                self.global_widget.delete_file.setEnabled(False)
                self.actions.delete.setEnabled(False)
                self.actions.save.setEnabled(False)
                self.actions.deleteFile.setEnabled(False)
                self.actions.copy.setEnabled(False)
                self.actions.edit.setEnabled(False)
                self.actions.undo.setEnabled(False)
                self.dirty = False
            for action in self.actions.onShapesPresent:
                action.setEnabled(False)
        def cancel():
            return
        if len(self.labelList) == 1:
            self.save_notes = YesCancelNote(self.tr("WARNING"), "删除最后一个标注将删除标签文件，是否继续？")
            self.save_notes.yes.clicked.connect(ok)
            self.save_notes.cancel.clicked.connect(cancel)
            self.save_notes.show()
        elif len(self.labelList.selectedItems()) == len(self.labelList):
            self.save_notes = YesCancelNote(self.tr("WARNING"), "删除所有标注将删除标签文件，是否继续？")
            self.save_notes.yes.clicked.connect(ok)
            self.save_notes.cancel.clicked.connect(cancel)
            self.save_notes.show()
        else:
            self.remLabels(self.canvas.deleteSelected())
            self.setDirty()
            self.tool_bar_widget.delete_2.setEnabled(False)
            self.tool_bar_widget.copy.setEnabled(False)
            self.actions.delete.setEnabled(False)
            self.actions.copy.setEnabled(False)
            self.actions.edit.setEnabled(False)
            if self.noShapes():
                for action in self.actions.onShapesPresent:
                    action.setEnabled(False)


    def copyShape(self):
        self.canvas.endMove(copy=True)
        for shape in self.canvas.selectedShapes:
            self.addLabel(shape)
        self.labelList.clearSelection()
        self.setDirty()

    def moveShape(self):
        self.canvas.endMove(copy=False)
        self.setDirty()

    def openDirDialog(self, _value=False, dirpath=None):
        if not self.mayContinue():
            return

        defaultOpenDirPath = dirpath if dirpath else "."
        if self.lastOpenDir and osp.exists(self.lastOpenDir):
            defaultOpenDirPath = self.lastOpenDir
        else:
            defaultOpenDirPath = (
                osp.dirname(self.filename) if self.filename else "."
            )

        targetDirPath = str(
            QtWidgets.QFileDialog.getExistingDirectory(
                self,
                self.tr("%s - Open Directory") % __appname__,
                defaultOpenDirPath,
                QtWidgets.QFileDialog.ShowDirsOnly
                | QtWidgets.QFileDialog.DontResolveSymlinks,
            )
        )
        if is_contains_chinese(targetDirPath):
            self.notes = Notes(self.tr("ERROR"), self.tr("The path does not support Chinese"), "ERROR")
            self.notes.show()
            return
        else:
            self.importDirImages(targetDirPath)

    @property
    def imageList(self):
        lst = []
        for i in range(self.fileListWidget.count()):
            item = self.fileListWidget.item(i)
            lst.append(item.text())
        return lst

    def importDroppedImageFiles(self, imageFiles):
        extensions = [
            ".%s" % fmt.data().decode().lower()
            for fmt in QtGui.QImageReader.supportedImageFormats()
        ]

        self.filename = None
        for file in imageFiles:
            if file in self.imageList or not file.lower().endswith(
                tuple(extensions)
            ):
                continue
            label_file = osp.splitext(file)[0] + ".json"
            if self.output_dir:
                label_file_without_path = osp.basename(label_file)
                label_file = osp.join(self.output_dir, label_file_without_path)
            item = QtWidgets.QListWidgetItem(file)
            item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
            if QtCore.QFile.exists(label_file) and LabelFile.is_label_file(
                label_file
            ):
                json_file = json.load(open(label_file, "r", encoding="utf-8"))
                imagename = json_file['imagePath']
                if imagename == file.split('/')[-1]:
                    item.setCheckState(Qt.Checked)
                else:
                    item.setCheckState(Qt.Unchecked)
            else:
                item.setCheckState(Qt.Unchecked)
            self.fileListWidget.addItem(item)

        if len(self.imageList) > 1:
            self.tool_bar_widget.prev.setEnabled(True)
            self.tool_bar_widget.next.setEnabled(True)
            self.actions.openNextImg.setEnabled(True)
            self.actions.openPrevImg.setEnabled(True)

        self.openNextImg()

    def importDirImages(self, dirpath, pattern=None, load=True):
        listFiles = []
        lenFiles = len(self.scanAllImages(dirpath))
        for file in self.scanAllImages(dirpath):
            file = file.replace(file.split('.')[-1], '')
            if file not in listFiles:
                listFiles.append(file)
        if len(listFiles) != lenFiles:
            self.notes = Notes(self.tr("ERROR"),
                               self.tr("Cannot open a file with the same name in the same directory."), "ERROR")
            self.notes.show()
            return
        self.tool_bar_widget.prev.setEnabled(True)
        self.tool_bar_widget.next.setEnabled(True)
        self.actions.openNextImg.setEnabled(True)
        self.actions.openPrevImg.setEnabled(True)

        if not self.mayContinue() or not dirpath:
            return

        self.lastOpenDir = dirpath
        self.filename = None
        self.fileListWidget.clear()
        for filename in self.scanAllImages(dirpath):
            externs = ['png', 'jpg', 'jpeg', 'bmp', 'webp']
            suffix = filename.split('/')[-1].split('.')[-1].lower()
            if suffix not in externs:
                self.fileListWidget.clear()
                self.notes = Notes(self.tr("ERROR"),
                                   self.tr("It only supports picture of format in jpg/png/jpeg/bmp/webp."), "ERROR")
                self.notes.show()
                return
            if is_contains_chinese(filename):
                self.fileListWidget.clear()
                self.notes = Notes(self.tr("ERROR"), self.tr("The path does not support Chinese"), "ERROR")
                self.notes.show()
                return
            if pattern and pattern not in filename:
                continue
            label_file = osp.splitext(filename)[0] + ".json"
            if self.output_dir:
                label_file_without_path = osp.basename(label_file)
                label_file = osp.join(self.output_dir, label_file_without_path)
            item = QtWidgets.QListWidgetItem(filename)
            item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
            if QtCore.QFile.exists(label_file) and LabelFile.is_label_file(
                label_file
            ):
                json_file = json.load(open(label_file, "r", encoding="utf-8"))
                imagename = json_file['imagePath']
                if imagename == filename.split('/')[-1]:
                    item.setCheckState(Qt.Checked)
                else:
                    item.setCheckState(Qt.Unchecked)
            else:
                item.setCheckState(Qt.Unchecked)
            self.fileListWidget.addItem(item)
        self.openNextImg(load=load)

    def scanAllImages(self, folderPath):
        extensions = [
            ".%s" % fmt.data().decode().lower()
            for fmt in QtGui.QImageReader.supportedImageFormats()
        ]

        images = []
        for root, dirs, files in os.walk(folderPath):
            for file in files:
                if file.lower().endswith(tuple(extensions)):
                    relativePath = osp.join(f"{root}/", file)
                    images.append(relativePath)
        images = natsort.os_sorted(images)
        return images

    def popUniqLabelListMenu(self, point):
        if self.uniqLabelList.indexAt(point).column() > -1:
            self.menus.uniqLabelList.exec_(self.uniqLabelList.mapToGlobal(point))
