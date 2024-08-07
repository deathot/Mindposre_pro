/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Raspberry Pi Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * 2022.08.25-Changed interface display
 *                 Huawei Technologies Co., Ltd.
 *
 */

import QtQuick 2.9
import QtQuick.Window 2.2
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.0
import QtQuick.Controls.Material 2.2
import Qt.labs.platform 1.1
import QtGraphicalEffects 1.13
import "qmlcomponents"

ApplicationWindow {
    id: window
    visible: true
    // 自定义头部
    flags: Qt.Window | Qt.FramelessWindowHint
    width: imageWriter.isEmbeddedMode() ? -1 : 1376
    height: imageWriter.isEmbeddedMode() ? -1 : 768
    minimumWidth: imageWriter.isEmbeddedMode() ? -1 : 1376
    //maximumWidth: imageWriter.isEmbeddedMode() ? -1 : 1376
    minimumHeight: imageWriter.isEmbeddedMode() ? -1 : 768
    //maximumHeight: imageWriter.isEmbeddedMode() ? -1 : 768

    title: ""


    FontLoader {id: roboto;      source: "fonts/Roboto-Regular.ttf"}
    FontLoader {id: harmonyOSSansMedium;  source: "fonts/HarmonyOS_Sans_SC_Medium.ttf"}
    FontLoader {id: harmonyOSSansSCBold;  source: "fonts/HarmonyOS_Sans_SC_Bold.ttf"}
    FontLoader {id: harmonyOSSansSCRegular;  source: "fonts/HarmonyOS_Sans_SC_Regular.ttf"}
    onClosing: {
        if (progressBar.visible) {
            close.accepted = false
            quitpopup.openPopup()
        }
    }

    Shortcut {
        sequence: StandardKey.Quit
        context: Qt.ApplicationShortcut
        onActivated: {
            if (!progressBar.visible) {
                Qt.quit()
            }
        }
    }

    Shortcut {
        sequences: ["Shift+Ctrl+X", "Shift+Meta+X"]
        context: Qt.ApplicationShortcut
        onActivated: {
            optionspopup.openPopup()
        }
    }
    // 背景色
    Rectangle {
        width: window.width
        height: window.height
        color: "#f4f6fa"
    }

    menuBar: Rectangle {
        height: 40
        visible: true
        id: menu
        Rectangle {
            anchors.fill: parent
            color: Qt.rgba(255,255,255,0.85)
            id: rect
        }
        MouseArea {
            anchors.fill: parent
            acceptedButtons: Qt.LeftButton
            onPressed: if(active) window.startSystemMove()
        }
        DropShadow {
            anchors.fill: rect
            horizontalOffset: 3
            verticalOffset: 0
            radius: 8.0
            samples: 16
            color: Qt.rgba(0,0,0,0.1)
            source: rect
        }
        Image {
            id: headerLogo
            source: "icons/AscendLogo.svg"
            sourceSize.width: 72
            sourceSize.height: 30
            anchors.left: parent.left
            anchors.leftMargin: 24
            anchors.verticalCenter: parent.verticalCenter
        }
        Text {
            id: headerTitle
            text: " | " + qsTr("One key Write Image Tool")
            anchors.left: headerLogo.right
            anchors.verticalCenter: parent.verticalCenter
            font.pixelSize: 15
            font.family: harmonyOSSansSCBold.name
        }
        Image {
            id: help
            source: "icons/help.svg"
            sourceSize.width: 15
            sourceSize.height: 15
            anchors.left: parent.left
            anchors.leftMargin: headerLogo.implicitWidth + headerTitle.implicitWidth + 40
            anchors.verticalCenter: parent.verticalCenter
            MouseArea {
                anchors.fill: help
                cursorShape: Qt.PointingHandCursor
                onClicked: {
                    Qt.openUrlExternally(body.feedBackUrl)
                }
            }
        }
        Image {
            id: hide
            source: "icons/hide.svg"
            sourceSize.width: 14
            sourceSize.height: 14
            x:window.width - 60
            y:12
            MouseArea {
                anchors.fill: hide
                cursorShape: Qt.PointingHandCursor
                onClicked: {
                    window.visibility = Window.Minimized
                }
            }
        }
        Image {
            id: zoom
            source: "icons/zoom.svg"
            sourceSize.width: 12
            sourceSize.height: 12
            x: hide.x + hide.width + 18
            y: 12
            visible: false
            MouseArea {
                anchors.fill: zoom
                cursorShape: Qt.PointingHandCursor
                onClicked: {
                    window.visibility === Window.Maximized ? window.visibility = window.showNormal() : window.showMaximized()
                }
            }
        }
        Image {
            id: close
            source: "icons/close.svg"
            sourceSize.width: 14
            sourceSize.height: 14
            x: hide.x + hide.width + 18
            y: 12
            MouseArea {
                anchors.fill: close
                cursorShape: Qt.PointingHandCursor
                onClicked: {
                    window.close()
                }
            }
        }

    }
    ColumnLayout{
        id: body
        property string device: ""
        property bool isA2: true
        property double startDownloadDate: 0
        property string feedBackUrl: ""
        RowLayout{
            id: content
            spacing: 19
            Layout.leftMargin: 24
            Layout.rightMargin: 24
            Layout.topMargin: 24
            Rectangle {
                id: operatebox
                implicitHeight: window.height - 90
                implicitWidth: window.width/3 - 28
                border.color: "#FEFEFE"
                radius: 2
                // tab选项下的横线
                Rectangle {
                    color: "#DFE5EF"
                    width: parent.width - 48
                    y: btbox.y + btbox.height
                    x: 24
                    implicitHeight: 2
                }
                // 保存选择的镜像名称
                ImButton {
                    id: osbutton
                    text: imageWriter.srcFileName() === "" ? qsTr("CHOOSE OPERATION TYPE") : imageWriter.srcFileName()
                    visible: false
                }
                // 烧录时禁止点击
                MouseArea {
                    id: operateboxdisabled
                    visible: false
                    anchors.fill: parent
                    enabled: true
                    z: 99
                }
                ColumnLayout {
                    id: operatecol
                    width: operatebox.width
                    height: operatebox.height
                    spacing: 0
                    Rectangle {
                        width: parent.width - 48
                        height: 112
                        Layout.leftMargin: 24
                        RowLayout {
                            spacing: 8
                            anchors.fill: parent
                            Image {
                                id: one
                                source: "icons/onedark.svg"
                                Layout.preferredHeight: 64
                                Layout.preferredWidth: 36
                                sourceSize.width: 36
                                sourceSize.height: 64
                                verticalAlignment: Image.AlignVCenter
                                Layout.alignment: Qt.AlignBottom
                                Layout.bottomMargin: 24
                            }
                            Text {
                                id: operatetitle
                                text: qsTr("Select Action")
                                color: "#8D98AA"
                                font.pixelSize: 18
                                height: 24
                                Layout.fillWidth: true
                                Layout.alignment: Qt.AlignBottom
                                Layout.bottomMargin: 24
                                font.family: harmonyOSSansSCBold.name
                            }
                        }

                    }
                    RowLayout {
                        id: btbox
                        width: parent.width
                        height: 30
                        spacing: 32
                        Layout.leftMargin: 24
                        Rectangle {
                            height: parent.height
                            width: 56
                            Text{
                                id: onlinebt
                                text: qsTr("Online Image")
                                anchors.centerIn: parent
                                color: swview.currentIndex === 0 ? "#0077FF" : "#4E5865"
                                font.pixelSize: 15
                                font.family: harmonyOSSansMedium.name
                            }
                            MouseArea {
                                anchors.fill: parent
                                cursorShape: Qt.PointingHandCursor
                                onClicked: {
                                    swview.setCurrentIndex(0)
                                    operatebox.handleStatus()

                                }

                            }
                            Rectangle {
                                color: swview.currentIndex === 0 ? "#0077FF" : "#DFE5EF"
                                width: 56
                                y: parent.y + parent.height
                                implicitHeight: 2
                            }

                        }
                        Rectangle {
                            height: parent.height
                            width: 56
                            Text {
                                id: localbt
                                text: qsTr("Local Image")
                                anchors.centerIn: parent
                                color: swview.currentIndex === 1 ? "#0077FF" : "#4E5865"
                                font.pixelSize: 15
                                font.family: harmonyOSSansMedium.name
                            }
                            MouseArea {
                                anchors.fill: parent
                                cursorShape: Qt.PointingHandCursor
                                onClicked: {
                                    swview.setCurrentIndex(1)
                                    operatebox.handleStatus()
                                }
                            }
                            Rectangle {
                                color: swview.currentIndex === 1 ? "#0077FF" : "#DFE5EF"
                                width: 56
                                y: parent.y + parent.height
                                implicitHeight: 2
                            }
                        }
                        Rectangle {
                            height: parent.height
                            width: 56
                            Text {
                                id: readbackbt
                                text: qsTr("Backup")
                                anchors.centerIn: parent
                                color: swview.currentIndex === 2 ? "#0077FF" : "#4E5865"
                                font.pixelSize: 15
                                font.family: harmonyOSSansMedium.name
                            }
                            MouseArea {
                                anchors.fill: parent
                                cursorShape: Qt.PointingHandCursor
                                onClicked: {
                                    swview.setCurrentIndex(2)
                                    operatebox.handleStatus()
                                }
                            }
                            Rectangle {
                                color: swview.currentIndex === 2 ? "#0077FF" : "#DFE5EF"
                                width: 56
                                y: parent.y + parent.height
                                implicitHeight: 2
                            }
                        }
                    }
                    Rectangle {
                        id: onlineview
                        width: parent.width - 48
                        implicitWidth: parent.width - 48
                        Layout.fillHeight: true
                        Layout.leftMargin: 24
                        Layout.topMargin: 24
                        Layout.bottomMargin: 24
                        clip: true
                        SwipeView {
                            id: swview
                            currentIndex: 0
                            interactive: false
                            width: onlineview.width
                            height: onlineview.height
                            Rectangle {
                                width: onlineview.width
                                height:onlineview.height
                                ColumnLayout {
                                    id: onlinebox
                                    visible: false
                                    spacing: 24
                                    Rectangle {
                                        height: 223
                                        width: onlineview.width

                                        ListView {
                                            id: dka2
                                            model: ListModel {
                                                id: dka2list
                                            }
                                            clip: true
                                            property var imagelist:[]
                                            property bool isClick: false
                                            property bool isSetIp: true
                                            currentIndex: -1
                                            delegate: osdelegate
                                            interactive: false
                                            width: parent.width
                                            height: parent.height
                                            implicitWidth: parent.width
                                            implicitHeight: parent.height
                                            Keys.onSpacePressed: {
                                                if (currentIndex != -1)
                                                    selectOSitem(model.get(currentIndex))
                                            }
                                            Accessible.onPressAction: {
                                                if (currentIndex != -1)
                                                    selectOSitem(model.get(currentIndex))
                                            }
                                            Keys.onEnterPressed: Keys.onSpacePressed(event)
                                            Keys.onReturnPressed: Keys.onSpacePressed(event)
                                            // 初始化其余两个table以及下面的listview的状态
                                            function changeStatus(){
                                                body.isA2 = true
                                                dka2.isClick = true
                                                dk.isClick = false
                                                readbackddr.text = ""
                                                localicon.source = "icons/upload.svg"
                                                openlocal.text = qsTr("Select File")
                                                loacltips.text = qsTr("Click the button to select a file")
                                                openlocal.showBg = true
                                                openlocal.showBr = false
                                                openlocal.textColor = "#FFFFFF"
                                                wrimg.source = "icons/writeimage.png"
                                                sdcardinfo.text = qsTr("Click the button to write the image")
                                                operatebox.handleStatus()
                                            }
                                        }

                                        ComboBox {
                                            id: dkacombox
                                            visible: false
                                            width: parent.width - 162
                                            height: 40
                                            z: 10
                                            anchors.left: dka2.left
                                            anchors.leftMargin: 138
                                            anchors.top: dka2.top
                                            anchors.topMargin: 38
                                            property string color: "#4E5865"
                                            property string bodercolor: "#C3CEDF"
                                            property bool isFirst: true
                                            font.family: harmonyOSSansSCRegular.name
                                            font.pixelSize: 10
                                            background: Rectangle {
                                                implicitWidth: dkacombox.width
                                                implicitHeight: 40
                                                color:"transparent"
                                                border.color: dkacombox.bodercolor
                                                border.width: 1
                                                radius: 2
                                            }
                                            delegate: ItemDelegate { //呈现标准视图项 以在各种控件和控件中用作委托
                                                width: dkacombox.width
                                                contentItem: Text {
                                                    text: modelData   //即model中的数据
                                                    color: "#4E5865"
                                                    font: dkacombox.font
                                                    verticalAlignment: Text.AlignVCenter
                                                }
                                            }
                                            popup: Popup {    //弹出项
                                                y: dkacombox.height
                                                width: dkacombox.width
                                                implicitHeight: contentItem.implicitHeight + 5
                                                padding: 1
                                                //istView具有一个模型和一个委托。模型model定义了要显示的数据
                                                contentItem: ListView {   //显示通过ListModel创建的模型中的数据
                                                    clip: true
                                                    implicitHeight: contentHeight
                                                    model: dkacombox.popup.visible ? dkacombox.delegateModel : null
                                                }
                                                background: Rectangle {
                                                    border.color: "#C3CEDF"
                                                    radius: 2
                                                }
                                            }
                                            contentItem: Text {
                                                leftPadding: 12
                                                rightPadding: dkacombox.indicator.width + dkacombox.spacing
                                                topPadding: 5
                                                bottomPadding: 5
                                                text: dkacombox.displayText
                                                color: dkacombox.color
                                                font.pixelSize: 10
                                                verticalAlignment: Text.AlignVCenter
                                                elide: Text.ElideRight
                                                font.family: harmonyOSSansSCRegular.name
                                            }
                                            indicator: Canvas {
                                                id: canvas
                                                x: parent.width - width - parent.rightPadding
                                                y: parent.topPadding + (parent.availableHeight - height) / 2
                                                width: 8
                                                height: 4
                                                contextType: "2d"
                                                onPaint: {
                                                    context.reset();
                                                    context.moveTo(0, 0);
                                                    context.lineTo(width, 0);
                                                    context.lineTo(width / 2, height);
                                                    context.closePath();
                                                    context.fillStyle = dkacombox.color ;
                                                    context.fill();
                                                }
                                            }

                                            onCurrentIndexChanged: {
                                                dka2list.clear()
                                                dka2list.append(dka2.imagelist[dkacombox.currentIndex])
                                                // 如果combox初始化时触发 不切换operatebox的state
                                                if(!dkacombox.isFirst) {
                                                    operatebox.handleStatus()
                                                    // 选择就默认选中
                                                    dka2.changeStatus()
                                                    selectOSitem(dka2.imagelist[dkacombox.currentIndex])
                                                }
                                                dkacombox.isFirst = false
                                            }
                                        }
                                    }
                                    Rectangle {
                                        height: 223
                                        width: onlineview.width
                                        ListView {
                                            id: dk
                                            model: ListModel {
                                                id: dklist
                                            }
                                            property var imagelist:[]
                                            property bool isClick: false
                                            property bool isSetIp: false
                                            currentIndex: -1
                                            clip: true
                                            delegate: osdelegate
                                            width: parent.width
                                            height: parent.height
                                            implicitWidth: parent.width
                                            implicitHeight: parent.height
                                            interactive: false
                                            Keys.onSpacePressed: {
                                                if (currentIndex != -1)
                                                    selectOSitem(model.get(currentIndex))
                                            }
                                            Accessible.onPressAction: {
                                                if (currentIndex != -1)
                                                    selectOSitem(model.get(currentIndex))
                                            }
                                            Keys.onEnterPressed: Keys.onSpacePressed(event)
                                            Keys.onReturnPressed: Keys.onSpacePressed(event)
                                            function changeStatus(){
                                                body.isA2 = false
                                                dk.isClick = true
                                                dka2.isClick = false
                                                readbackddr.text = ""
                                                localicon.source = "icons/upload.svg"
                                                openlocal.text = qsTr("选择文件")
                                                loacltips.text = qsTr("将文件拖到此处或点击按钮选择文件")
                                                openlocal.showBg = true
                                                openlocal.showBr = false
                                                openlocal.textColor = "#FFFFFF"
                                                wrimg.source = "icons/writeimage.png"
                                                sdcardinfo.text = qsTr("Click the button to write the image")
                                                operatebox.handleStatus()
                                            }
                                        }
                                        ComboBox {
                                            id: dkcombox
                                            visible: false
                                            width: parent.width - 162
                                            height: 40
                                            z: 10
                                            anchors.left: dk.left
                                            anchors.leftMargin: 138
                                            anchors.top: dk.top
                                            anchors.topMargin: 38
                                            property string color: "#4E5865"
                                            property string bodercolor: "#C3CEDF"
                                            property bool isFirst: true
                                            font.family: harmonyOSSansSCRegular.name
                                            font.pixelSize: 10
                                            background: Rectangle {
                                                implicitWidth: dkcombox.width
                                                implicitHeight: 32
                                                color:"transparent"
                                                border.color: dkcombox.bodercolor
                                                border.width: 1
                                                radius: 2
                                            }
                                            delegate: ItemDelegate { //呈现标准视图项 以在各种控件和控件中用作委托
                                                width: dkcombox.width
                                                contentItem: Text {
                                                    text: modelData   //即model中的数据
                                                    color: "#4E5865"
                                                    font: dkcombox.font
                                                    verticalAlignment: Text.AlignVCenter
                                                }
                                            }
                                            popup: Popup {    //弹出项
                                                y: dkcombox.height
                                                width: dkcombox.width
                                                implicitHeight: contentItem.implicitHeight + 5
                                                padding: 1
                                                //istView具有一个模型和一个委托。模型model定义了要显示的数据
                                                contentItem: ListView {   //显示通过ListModel创建的模型中的数据
                                                    clip: true
                                                    implicitHeight: contentHeight
                                                    model: dkcombox.popup.visible ? dkcombox.delegateModel : null
                                                }
                                                background: Rectangle {
                                                    border.color: "#C3CEDF"
                                                    radius: 2
                                                }
                                            }
                                            contentItem: Text {
                                                leftPadding: 12
                                                rightPadding: dkcombox.indicator.width + dkcombox.spacing
                                                topPadding: 5
                                                bottomPadding: 5
                                                text: dkcombox.displayText
                                                color: dkcombox.color
                                                font.pixelSize: 10
                                                verticalAlignment: Text.AlignVCenter
                                                elide: Text.ElideRight
                                                font.family: harmonyOSSansSCRegular.name
                                            }
                                            indicator: Canvas {
                                                id: dkcanvas
                                                x: parent.width - width - parent.rightPadding
                                                y: parent.topPadding + (parent.availableHeight - height) / 2
                                                width: 8
                                                height: 4
                                                contextType: "2d"
                                                onPaint: {
                                                    context.reset();
                                                    context.moveTo(0, 0);
                                                    context.lineTo(width, 0);
                                                    context.lineTo(width / 2, height);
                                                    context.closePath();
                                                    context.fillStyle = dkcombox.color ;
                                                    context.fill();
                                                }
                                            }
                                            onCurrentIndexChanged: {
                                                dklist.clear()
                                                dklist.append(dk.imagelist[dkcombox.currentIndex])
                                                // 如果combox初始化时触发 不切换operatebox的state
                                                if(!dkcombox.isFirst) {
                                                    operatebox.handleStatus()
                                                    dk.changeStatus()
                                                    selectOSitem(dk.imagelist[dkcombox.currentIndex])
                                                }
                                                dkcombox.isFirst = false
                                            }
                                        }
                                    }
                                }
                                Rectangle {
                                    id: offlinebox
                                    visible: false
                                    anchors.fill: parent
                                    Image {
                                        id: offline
                                        source: "icons/offline.png"
                                        sourceSize.width: 220
                                        sourceSize.height: 165
                                        width: 220
                                        height: 165
                                        verticalAlignment: Image.AlignVCenter
                                        anchors.centerIn: parent
                                        anchors.verticalCenterOffset: -152
                                    }
                                    Text {
                                        id: offlineinfo
                                        text: qsTr("镜像列表获取失败,请检查网络连接")
                                        anchors.horizontalCenter: parent.horizontalCenter
                                        y: offline.y + offline.height + 14
                                        font.family: harmonyOSSansMedium.name
                                        color: "#4E5865"
                                        font.pixelSize: 14
                                    }

                                    RowLayout {
                                        id: offlinetips
                                        anchors.horizontalCenter: parent.horizontalCenter
                                        y: offlineinfo.y + offlineinfo.height + 2
                                        Text {
                                            text: qsTr("您还可以使用")
                                            font.family: harmonyOSSansMedium.name
                                            color: "#8D98AA"
                                            font.pixelSize: 12
                                        }
                                        Text {
                                            text: qsTr("本地制卡")
                                            font.family: harmonyOSSansMedium.name
                                            color: "#0077FF"
                                            font.pixelSize: 12
                                            MouseArea {
                                                anchors.fill: parent
                                                cursorShape: Qt.PointingHandCursor
                                                onClicked: {
                                                    swview.setCurrentIndex(1)
                                                    operatebox.handleStatus()
                                                }
                                            }
                                        }
                                        Text {
                                            text: qsTr("功能")
                                            font.family: harmonyOSSansMedium.name
                                            color: "#8D98AA"
                                            font.pixelSize: 12
                                        }
                                    }
                                    ImButton {
                                        id: renovate
                                        text: qsTr("刷新页面")
                                        height: 40
                                        width: 152
                                        enabled: true
                                        font.pixelSize: 16
                                        anchors.horizontalCenter: offlinebox.horizontalCenter
                                        y: offlinetips.y + offlinetips.height + 6
                                        onClicked: {
                                            fetchOSlist()
                                        }
                                    }
                                }
                            }
                            Rectangle {
                                id: loaclfile
                                width: onlineview.width
                                height:onlineview.height
                                // 提示的背景颜色
                                Rectangle {
                                    color: "#0077FF"
                                    radius: 2
                                    height: 48
                                    width: parent.width
                                    x: 0
                                    y: 0
                                    opacity: 0.2
                                }
                                Rectangle {
                                    id: localinfobox
                                    radius: 2
                                    height: 48
                                    width: parent.width
                                    x: 0
                                    y: 0
                                    color: "transparent"
                                    Image {
                                        source: "icons/remind.svg"
                                        sourceSize.width: 16
                                        sourceSize.height: 16
                                        verticalAlignment: Image.AlignVCenter
                                        anchors.left: parent.left
                                        anchors.top: parent.top
                                        anchors.topMargin: 12
                                        anchors.leftMargin: 16
                                    }
                                    Text {
                                        id: localinfo
                                        color: "#000"
                                        wrapMode: Text.WordWrap
                                        width: parent.width - 56
                                        anchors.centerIn: parent
                                        anchors.horizontalCenterOffset: 12
                                        font.family: harmonyOSSansMedium.name
                                        font.pixelSize: 12
                                        text: qsTr("Supported file formats:. img/. zip/. iso/. gz/. xz/. zst. Supported image file names: English letters, numbers, Chinese, and underscores")
                                    }
                                }

                                Rectangle {
                                    id: localbox
                                    height: parent.height - localinfobox.height - 24
                                    width: parent.width
                                    x: 0
                                    y: localinfobox.y + localinfobox.height + 24
                                    Image {
                                        id: localicon
                                        source: "icons/upload.svg"
                                        sourceSize.width: 64
                                        sourceSize.height: 64
                                        width: 64
                                        height: 64
                                        verticalAlignment: Image.AlignVCenter
                                        anchors.centerIn: parent
                                        anchors.verticalCenterOffset: -80
                                    }
                                    Text {
                                        id: loacltips
                                        text: qsTr("Click the button to select a file")
                                        anchors.horizontalCenter: parent.horizontalCenter
                                        y: localicon.y + localicon.height
                                        color: "#4E5865"
                                        font.pixelSize: 12
                                        font.family: harmonyOSSansMedium.name
                                    }
                                    ImButton {
                                        id: openlocal
                                        text: qsTr("Select File")
                                        height: 35
                                        width: 70
                                        enabled: true
                                        anchors.horizontalCenter: parent.horizontalCenter
                                        y: loacltips.y + loacltips.height + 24
                                        font.family: harmonyOSSansMedium.name
                                        /*打开选择文件的弹窗*/
                                        onClicked: {
                                            if (!imageWriter.isEmbeddedMode()) {
                                                operatebox.handleStatus()
                                                imageWriter.openFileDialog()
                                            }
                                        }
                                    }

                                    function changeStatus(){
                                        dk.isClick = false
                                        dka2.isClick = false
                                        readbackddr.text = ""
                                        wrimg.source = "icons/writeimage.png"
                                        sdcardinfo.text = qsTr("Click the button to write the image")
                                        // 改变openlocal自身状态
                                        openlocal.text = qsTr("Reselect")
                                        openlocal.showBr = true
                                        openlocal.showBg = false
                                        openlocal.textColor = "#4E5865"
                                    }
                                    DropArea {
                                        z: 10
                                        id: filedarp
                                        anchors.fill: parent
                                        onEntered: {
                                            drag.accepted = true
                                            var typeArr = ["img","zip", "iso", "gz", "xz", "zst"]
                                            var filearr =  drag.urls[0].split(".")
                                            var filetype = filearr[filearr.length - 1]
                                            if(typeArr.indexOf(filetype) > 0) {
                                                loacltips.text = drag.urls[0]
                                                onFileSelected(drag.urls[0])
                                            }
                                        }
                                    }
                                }
                            }
                            Rectangle {
                                id: readback
                                width: onlineview.width
                                height:onlineview.height
                                Image {
                                    id: readbackicon
                                    source: "icons/cloneSDcard.png"
                                    sourceSize.width: 165
                                    sourceSize.height: 165
                                    width: 165
                                    height: 165
                                    verticalAlignment: Image.AlignVCenter
                                    anchors.centerIn: parent
                                    anchors.verticalCenterOffset: -152
                                }
                                Text {
                                    id: readbacktips
                                    text: qsTr("Local backup address")
                                    anchors.horizontalCenter: parent.horizontalCenter
                                    y: readbackicon.y + readbackicon.height + 14
                                    font.family: harmonyOSSansMedium.name
                                    color: "#4E5865"
                                    font.pixelSize: 12
                                }
                                Rectangle {
                                    border.color: "#C3CEDF"
                                    radius: 3
                                    clip: true
                                    width: parent.width
                                    height: 32
                                    y: readbacktips.y + readbacktips.height + 31
                                    TextInput{
                                        rightPadding: 50
                                        font.pixelSize: 12
                                        id: readbackddr
                                        anchors.fill: parent
                                        anchors.leftMargin: 16
                                        anchors.rightMargin: 32
                                        verticalAlignment: TextInput.AlignVCenter
                                        color: "#4E5865"
                                        readOnly: true
                                    }
                                    Image {
                                        id: name
                                        source: "icons/more.svg"
                                        width: 18
                                        height: 18
                                        anchors.right: parent.right
                                        anchors.rightMargin: 5
                                        anchors.verticalCenter: parent.verticalCenter
                                    }
                                    MouseArea{
                                        anchors.fill: parent
                                        cursorShape: Qt.PointingHandCursor
                                        onClicked: {
                                            fileDialog.open()
                                            operatebox.handleStatus()
                                            // imageWriter.startDriveListPolling()
                                        }
                                    }
                                }
                                function changeStatus(){
                                    dka2.isClick = false
                                    dk.isClick = false
                                    localicon.source = "icons/upload.svg"
                                    openlocal.showBg = true
                                    openlocal.showBr = false
                                    openlocal.textColor = "#FFFFFF"
                                    openlocal.text = qsTr("Select File")
                                    loacltips.text = qsTr("Click the button to select a file")
                                    wrimg.source = "icons/cloneSD.png"
                                    sdcardinfo.text = qsTr("Click the button to Backup the SD card")
                                }
                            }
                        }
                    }
                }
                states: [
                    State {
                        name: "click"
                        PropertyChanges {
                            target: one
                            source: "icons/one.svg"
                        }
                        PropertyChanges {
                            target: operatetitle
                            color: "#000000"
                        }
                    },
                    State {
                        name: "disabled"
                        PropertyChanges {
                            target: operateboxdisabled
                            visible: true
                        }
                    }
                ]
                Component.onCompleted: {
                    if (imageWriter.isOnline()) {
                        fetchOSlist();
                    }
                }
                // 变换容器的states
                function handleStatus(){
                    operatebox.state = "click"
                    sdcardbox.state = ""
                    writebox.state = ""
                }
            }
            Rectangle {
                id: sdcardbox
                implicitHeight: window.height - 90
                implicitWidth: window.width/3 - 29
                radius: 4
                MouseArea {
                    id:sdcarddisabled
                    anchors.fill: parent
                    visible: false
                    z: 99
                    enabled: true
                }
                ColumnLayout {
                    id: sdcard
                    width: parent.width
                    height: operatebox.height
                    spacing: 0
                    Rectangle {
                        width: parent.width - 48
                        height: 112
                        Layout.leftMargin: 24
                        RowLayout {
                            spacing: 8
                            anchors.fill: parent
                            Image {
                                id: two
                                source: "icons/twodark.svg"
                                Layout.preferredHeight: 64
                                Layout.preferredWidth: 50
                                sourceSize.width: 50
                                sourceSize.height: 64
                                verticalAlignment: Image.AlignVCenter
                                Layout.alignment: Qt.AlignBottom
                                Layout.bottomMargin: 24
                            }
                            Text {
                                id: sdtitle
                                text: qsTr("Select SD card")
                                color: "#8D98AA"
                                font.pixelSize: 18
                                height: 24
                                Layout.fillWidth: true
                                Layout.alignment: Qt.AlignBottom
                                Layout.bottomMargin: 24
                                font.family: harmonyOSSansSCBold.name
                            }
                        }

                    }
                    Rectangle {
                        id: sdbox
                        implicitWidth: parent.width - 48
                        Layout.fillHeight: true
                        Layout.leftMargin: 24
                        Layout.bottomMargin: 24
                        Image {
                            id: sdimage
                            source: "icons/emptySDcard.png"
                            Layout.preferredHeight: 192
                            Layout.preferredWidth: 192
                            sourceSize.width: 192
                            sourceSize.height: 192
                            anchors.centerIn: parent
                            anchors.verticalCenterOffset: -116
                        }
                        Text {
                            id: sdinfo
                            text: qsTr("Please insert SD card")
                            font.pixelSize: 12
                            color: "#4E5865"
                            anchors.horizontalCenter: sdbox.horizontalCenter
                            y: sdimage.y + sdimage.height + 16
                            font.family: harmonyOSSansMedium.name
                        }
                        ComBox {
                            id: sdcom
                            comboBoxWidth: sdbox.width
                            model: driveListModel
                            delegate: dstdelegate
                            y: sdinfo.y + sdinfo.height + 24
                            color: "#8D98AA"
                            currentText: qsTr("Please select (unselected)")
                            z: 10
                            onClick: {
                                sdcardbox.handleStatus()
                            }
                        }
                        Component.onCompleted: {
                            imageWriter.startDriveListPolling()
                        }
                        states: [
                            State {
                                name: "hassd"
                                PropertyChanges {
                                    target: sdimage
                                    source: "icons/hasSDcard.png"
                                }
                            }
                        ]
                    }
                 }
                states: [
                    State {
                        name: "click"
                        PropertyChanges {
                            target: two
                            source: "icons/two.svg"
                        }
                        PropertyChanges {
                            target: sdtitle
                            color: "#000000"
                        }
                    },
                    State {
                        name: "disabled"
                        PropertyChanges {
                            target: sdcarddisabled
                            visible: true
                        }
                    }
                ]
                // 变换容器的states
                function handleStatus(){
                    operatebox.state = ""
                    sdcardbox.state = "click"
                    writebox.state = ""
                }
            }
            Rectangle {
                id: writebox
                implicitHeight: window.height - 90
                implicitWidth: window.width/3 - 29
                radius: 4
                ColumnLayout {
                    id: wrimage
                    width: parent.width
                    height: operatebox.height
                    spacing: 0
                    Rectangle {
                        width: parent.width - 48
                        height: 112
                        Layout.leftMargin: 24
                        RowLayout {
                            spacing: 8
                            anchors.fill: parent
                            Image {
                                id: three
                                source: "icons/threedark.svg"
                                Layout.preferredHeight: 64
                                Layout.preferredWidth: 48
                                sourceSize.width: 48
                                sourceSize.height: 64
                                verticalAlignment: Image.AlignVCenter
                                Layout.alignment: Qt.AlignBottom
                                Layout.bottomMargin: 24
                            }
                            Text {
                                id: wrtitle
                                text: qsTr("WRITE")
                                color: "#8D98AA"
                                font.pixelSize: 18
                                height: 24
                                Layout.fillWidth: true
                                Layout.alignment: Qt.AlignBottom
                                Layout.bottomMargin: 24
                                font.family: harmonyOSSansSCBold.name
                            }
                        }

                    }
                    Rectangle {
                        id: wrbox
                        implicitWidth: parent.width - 48
                        Layout.fillHeight: true
                        Layout.leftMargin: 24
                        Layout.bottomMargin: 24
                        Image {
                            id: wrimg
                            source: "icons/writeimage.png"
                            Layout.preferredHeight: 192
                            Layout.preferredWidth: 192
                            sourceSize.width: 192
                            sourceSize.height: 192
                            anchors.centerIn: parent
                            anchors.verticalCenterOffset: -116
                        }
                        Text {
                            id: sdcardinfo
                            text: qsTr("Click the button to write the image")
                            font.pixelSize: 12
                            color: "#4E5865"
                            anchors.horizontalCenter: wrbox.horizontalCenter
                            y: wrimg.y + wrimg.height + 16
                            visible: true
                            font.family: harmonyOSSansMedium.name
                        }
                        Text {
                            id: progressText
                            text: qsTr("")
                            font.pixelSize: 12
                            color: "#4E5865"
                            anchors.horizontalCenter: wrbox.horizontalCenter
                            y: wrimg.y + wrimg.height + 16
                            visible: false
                            font.family: harmonyOSSansMedium.name
                        }
                        ProgressBar {
                            id: progressBar
                            width: parent.width
                            visible: false
                            Material.background: "#DFE5EF"
                            y: sdcardinfo.y + sdcardinfo.height + 16
                        }
                        ImButton {
                            id: writebutton
                            text: qsTr("WRITE")
                            height: 40
                            width: 152
                            Accessible.description: qsTr("Select this button to start writing the image")
                            enabled: false
                            font.pixelSize: 14
                            anchors.horizontalCenter: wrbox.horizontalCenter
                            y: sdcardinfo.y + sdcardinfo.height + 24
                            onClicked: {
                                writebox.handleStatus()
                                if (!imageWriter.readyToWrite()) {
                                    return
                                }
                                if (!optionspopup.initialized && imageWriter.imageSupportsCustomization() && imageWriter.hasSavedCustomizationSettings()) {
                                    usesavedsettingspopup.openPopup()
                                } else if(imageWriter.getOperation() === 3) {
                                    confirmreadbackpopup.askForConfirmation()
                                } else {
                                    startWriteImage()
//                                     confirmwritepopup.askForConfirmation()
                                }
                            }
                        }
                        ImButton {
                            id: cancelwritebutton
                            text: qsTr("CANCEL WRITE")
                            onClicked: {
                                if(progressBar.visible){
                                    if(imageWriter.getOperation() == 3){
                                        cancleConfirm.title = "取消备份"
                                        cancleConfirm.text = "您确定要取消备份吗"
                                    } else {
                                        cancleConfirm.title = "取消烧录"
                                        cancleConfirm.text = "您确定要取消烧录吗"
                                    }
                                    cancleConfirm.openPopup()
                                }else{
                                    enabled = false
                                    progressText.text = qsTr("Cancelling...")
                                    imageWriter.cancelWrite()
                                }
                            }
                            height: 40
                            width: 152
                            anchors.horizontalCenter: wrbox.horizontalCenter
                            visible: false
                            font.pixelSize: 14
                            y: sdcardinfo.y + sdcardinfo.height + 24
                        }

                        ImButton {
                            id: cancelverifybutton
                            text: qsTr("CANCEL VERIFY")
                            onClicked: {
                                enabled = false
                                progressText.text = qsTr("Finalizing...")
                                imageWriter.setVerifyEnabled(false)
                            }
                            Layout.alignment: Qt.AlignRight
                            visible: false
                        }
                    }
                 }
                states: [
                    State {
                        name: "click"
                        PropertyChanges {
                            target: three
                            source: "icons/three.svg"
                        }
                        PropertyChanges {
                            target: wrtitle
                            color: "#000000"
                        }
                    }
                ]
                // 变换容器的states
                function handleStatus(){
                    operatebox.state = ""
                    sdcardbox.state = ""
                    writebox.state = "click"
                }
            }
        }
    }

    /*
      Popup for OS selection
     */
    Popup {
        id: ospopup
        x: 50
        y: 25
        width: parent.width-100
        height: parent.height-50
        padding: 0
        // closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
         property string categorySelected : ""
        Rectangle {
            width: parent.width
            height: parent.height
            color: "#1c213b"
        }
        // background of title
        Rectangle {
            color: "#383C52"
            anchors.right: parent.right
            anchors.top: parent.top
            height: 35
            width: parent.width
        }
        // line under title
        Rectangle {
            color: "#ffffff"
            width: parent.width
            opacity: 0.44
            y: 35
            implicitHeight: 1
        }

        Image {
            id: ospopupclose
            source: "icons/close.svg"
            sourceSize.width: 12
            sourceSize.height: 12
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.rightMargin: 25
            anchors.topMargin: 10
            MouseArea {
                anchors.fill: parent
                cursorShape: Qt.PointingHandCursor
                onClicked: {
                    ospopup.close()
                }
            }
        }

        Image {
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.rightMargin: parent.width - 25
            anchors.topMargin: 10
            source: "icons/dirurl.svg"
            height: 15
            width: 15
            visible: false
            MouseArea {
                anchors.fill: parent
                cursorShape: Qt.PointingHandCursor
                onClicked: {
                    jsonurlPop.open()
                }
            }
        }

        ColumnLayout {
            spacing: 11

            Text {
                text: qsTr("Operation Type")
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                Layout.fillWidth: true
                Layout.topMargin: 10
                font.family: roboto.name
                font.bold: true
                color: "#ffffff"
            }

            Item {
                clip: true
                Layout.preferredWidth: oslist.width
                Layout.preferredHeight: oslist.height

                SwipeView {
                    id: osswipeview
                    interactive: false
                    ListView {
                        id: oslist
                        model: osmodel
                        currentIndex: -1
                        delegate: osdelegate
                        width: window.width-100
                        height: window.height-100
                        boundsBehavior: Flickable.StopAtBounds
                        highlight: Rectangle { color: "#dfdfdf"; radius: 5}
                        ScrollBar.vertical: ScrollBar {
                            width: 10
                            policy: oslist.contentHeight > oslist.height ? ScrollBar.AlwaysOn : ScrollBar.AsNeeded
                        }
                        Keys.onSpacePressed: {
                            if (currentIndex != -1)
                                selectOSitem(model.get(currentIndex), true)
                        }
                        Accessible.onPressAction: {
                            if (currentIndex != -1)
                                selectOSitem(model.get(currentIndex), true)
                        }
                        Keys.onEnterPressed: Keys.onSpacePressed(event)
                        Keys.onReturnPressed: Keys.onSpacePressed(event)
                    }
                }
            }
        }
    }

    Component {
        id: suboslist

        ListView {
            model: ListModel {
                ListElement {
                    url: ""
                    icon: "icons/ic_chevron_left_40px.svg"
                    extract_size: 0
                    image_download_size: 0
                    extract_sha256: ""
                    contains_multiple_files: false
                    release_date: ""
                    subitems_url: "internal://back"
                    subitems_json: ""
                    name: qsTr("Back")
                    description: qsTr("Go back to main menu")
                    tooltip: ""
                    website: ""
                    init_format: ""
                    firmwareanddriverversions: ""
                    cannversion: ""
                    is_extensible: false
                    dk_version: ""
                    os_version: ""
                }
            }

            currentIndex: -1
            delegate: osdelegate
            width: window.width-100
            height: window.height-100
            boundsBehavior: Flickable.StopAtBounds
            highlight: Rectangle { color: "lightsteelblue"; radius: 5 }
            ScrollBar.vertical: ScrollBar {
                width: 10
                policy: parent.contentHeight > parent.height ? ScrollBar.AlwaysOn : ScrollBar.AsNeeded
            }
            Keys.onSpacePressed: {
                if (currentIndex != -1)
                    selectOSitem(model.get(currentIndex))
            }
            Accessible.onPressAction: {
                if (currentIndex != -1)
                    selectOSitem(model.get(currentIndex))
            }
            Keys.onEnterPressed: Keys.onSpacePressed(event)
            Keys.onReturnPressed: Keys.onSpacePressed(event)
        }
    }

    ListModel {
        id: osmodel

        ListElement {
            url: "internal://format"
            icon: "icons/erase.svg"
            extract_size: 0
            image_download_size: 0
            extract_sha256: ""
            contains_multiple_files: false
            release_date: ""
            subitems_url: ""
            subitems_json: ""
            name: qsTr("Erase")
            description: qsTr("Format card as FAT32")
            tooltip: ""
            website: ""
            init_format: ""
            firmwareanddriverversions: ""
            cannversion: ""
            is_extensible: false
            dk_version: ""
            os_version: ""
        }


    }

    Component {
        id: osdelegate
        Item {
            id: osbx
            height: 223
            width: onlineview.width
            Accessible.name: name+".\n"+description
            MouseArea {
                id: osMouseArea
                anchors.fill: parent
                cursorShape: Qt.PointingHandCursor
                hoverEnabled: true

                onEntered: {
                    bgrect.mouseOver = true
                }

                onExited: {
                    bgrect.mouseOver = false
                }

                onClicked: {
                    parent.ListView.view.changeStatus()

                    selectOSitem(model)
                }
            }

            Rectangle {
               id: bgrect
               anchors.fill: parent
               // visible: mouseOver && parent.ListView.view.currentIndex !== index
               visible: mouseOver || parent.ListView.view.isClick === true
               property bool mouseOver: false
               border.color: parent.ListView.view.isClick ? "#0077FF" : "#52A3FF"
               border.width: 1
               radius: 4
               Image {
                   source: "icons/tick-copy.png"
                   Layout.preferredHeight: 30
                   Layout.preferredWidth: 30
                   sourceSize.width: 30
                   sourceSize.height: 30
                   verticalAlignment: Image.AlignVCenter
                   anchors.top: parent.top
                   anchors.right: parent.right
                   visible: parent.parent.ListView.view.isClick === true
               }

            }
            RowLayout {
                id: contentLayout
                anchors {
                    left: parent.left
                    top: parent.top
                    right: parent.right
                    margins: 24
                }
                spacing: 18
                ColumnLayout {
                    spacing: 10
                    Image {
                        source: icon
                        Layout.preferredHeight: 54
                        Layout.preferredWidth: 96
                        sourceSize.width: 54
                        sourceSize.height: 76
                        fillMode: Image.PreserveAspectFit
                        verticalAlignment: Image.AlignVCenter
                        Layout.alignment: Qt.AlignHCenter
                    }
                    Text {
                        id: dkversion
                        text: dk_version
                        font.pixelSize: 11
                        font.family: harmonyOSSansSCBold.name
                        color: "#4E5865"
                        font.bold: true
                        Layout.alignment: Qt.AlignHCenter
                    }
                    ImButton {
                        id: ipinfo
                        visible: parent.parent.parent.ListView.view.isSetIp
                        text: qsTr("配置网络信息")
                        showBg: false
                        showBr: true
                        textColor: "#4E5865"
                        implicitWidth: 80
                        implicitHeight: 36
                        Layout.alignment: Qt.AlignHCenter
                        onClicked: {
                            setip.visible = true
                        }
                    }
                    Rectangle {
                       width: 50
                       Layout.fillHeight: true
                    }
                }

                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 5
                    Text {
                        text: qsTr("Image Version")
                        elide: Text.ElideRight
                        font.family: harmonyOSSansMedium.name
                        color: "#4E5865"
                        font.pixelSize: 11
                    }

                    // 选择框的占位
                    Rectangle {
                        height: 32
                        Layout.fillWidth: true
                    }

                    Text {
                        Layout.fillWidth: true
                        font.family: harmonyOSSansSCRegular.name
                        lineHeight: 1.2
                        text: description
                        wrapMode: Text.WordWrap
                        color: "#8D98AA"
                        font.pixelSize: 11
                        elide: Text.ElideRight
                        maximumLineCount: 2
                        clip: true
                        MouseArea{
                            z: 100
                            property bool entered: false
                            hoverEnabled: true
                            anchors.fill: parent
                            onEntered: {
                                entered = true
                            }
                            onExited: {
                                entered = false
                            }
                            ToolTip{
                                visible: parent.entered
                                text: description
                                delay: 1000
                                font.pixelSize: 11
                                font.family: harmonyOSSansSCRegular.name
                                y: parent.y + parent.height
                                width: parent.width
                                x: parent.x
                            }
                        }
                    }

                    Text {
                        Layout.fillWidth: true
                        elide: Text.ElideRight
                        font.family: harmonyOSSansSCBold.name
                        visible: typeof(os_version) == "string" && os_version
                        text: qsTr("OsVersion: %1").arg(os_version)
                        color: "#000000"
                        font.pixelSize: 11
                    }

                    Text {
                        Layout.fillWidth: true
                        elide: Text.ElideRight
                        font.family: harmonyOSSansSCBold.name
                        visible: typeof(firmwareanddriverversions) == "string" && firmwareanddriverversions
                        text: qsTr("Firmwareanddriverversions: %1").arg(firmwareanddriverversions)
                        color: "#000000"
                        font.pixelSize: 11
                    }

                    Text {
                        Layout.fillWidth: true
                        elide: Text.ElideRight
                        font.family: harmonyOSSansSCBold.name
                        visible: typeof(cannversion) == "string" && cannversion
                        text: qsTr("Cannversion: %1").arg(cannversion)
                        color: "#000000"
                        font.pixelSize: 11
                    }

                    Text {
                        visible: false
                        Layout.fillWidth: true
                        elide: Text.ElideRight
                        color: "#000000"
                        font.family: harmonyOSSansSCBold.name
                        // visible: typeof(release_date) == "string" && release_date
                        text: qsTr("Released: %1").arg(release_date)
                        font.pixelSize: 11
                    }

                    Text {
                        Layout.fillWidth: true
                        elide: Text.ElideRight
                        color: "#000000"
                        font.pixelSize: 11
                        font.family: harmonyOSSansSCBold.name
                        visible: typeof(url) == "string" && url != "" && url != "internal://format" && url != "input" && url != "internal://readback"
                        text: !url ? "" :
                              typeof(extract_sha256) != "undefined" && imageWriter.isCached(url,extract_sha256)
                                ? qsTr("Cached on your computer")
                                : url.startsWith("file://")
                                  ? qsTr("Local file")
                                  : qsTr("Online - %1 GB download").arg((image_download_size/1073741824).toFixed(1))
                    }

                    ToolTip {
                        visible: osMouseArea.containsMouse && typeof(tooltip) == "string" && tooltip != ""
                        delay: 1000
                        text: typeof(tooltip) == "string" ? tooltip : ""
                        clip: false
                        font.pixelSize: 11
                    }
                }
                Image {
                    source: "icons/ic_chevron_right_40px.svg"
                    visible: (typeof(subitems_json) == "string" && subitems_json != "") || (typeof(subitems_url) == "string" && subitems_url != "" && subitems_url != "internal://back")
                    Layout.preferredHeight: 20
                    Layout.preferredWidth: 20
                    fillMode: Image.PreserveAspectFit
                }
            }
        }
    }

    /*
      Popup for storage device selection
     */
    Popup {
        id: dstpopup
        x: 50
        y: 25
        width: parent.width-100
        height: parent.height-50
        padding: 0
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        onClosed: imageWriter.stopDriveListPolling()
        // background
        Rectangle {
            color: "#1c213b"
            anchors.fill: parent
        }
        // background of title
        Rectangle {
            color: "#383C52"
            anchors.right: parent.right
            anchors.top: parent.top
            height: 35
            width: parent.width
        }

        Image {
            id: dstpopupclose
            source: "icons/close.svg"
            sourceSize.width: 12
            sourceSize.height: 12
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.rightMargin: 25
            anchors.topMargin: 10
            MouseArea {
                anchors.fill: parent
                cursorShape: Qt.PointingHandCursor
                onClicked: {
                    dstpopup.close()
                }
            }
        }

        ColumnLayout {
            spacing: 10

            Text {
                text: qsTr("Storage")
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                Layout.fillWidth: true
                Layout.topMargin: 10
                font.family: roboto.name
                font.bold: true
                color: "#ffffff"
            }

            Item {
                clip: true
                Layout.preferredWidth: dstlist.width
                Layout.preferredHeight: dstlist.height

                ListView {
                    id: dstlist
                    model: driveListModel
                    delegate: dstdelegate
                    width: window.width-100
                    height: window.height-100
                    boundsBehavior: Flickable.StopAtBounds
                    highlight: Rectangle { color: "lightsteelblue"; radius: 5 }
                    ScrollBar.vertical: ScrollBar {
                        width: 10
                        policy: dstlist.contentHeight > dstlist.height ? ScrollBar.AlwaysOn : ScrollBar.AsNeeded
                    }
                    Keys.onSpacePressed: {
                        if (currentIndex == -1)
                            return
                        selectDstItem(currentItem)
                    }
                    Accessible.onPressAction: {
                        if (currentIndex == -1)
                            return
                        selectDstItem(currentItem)
                    }
                    Keys.onEnterPressed: Keys.onSpacePressed(event)
                    Keys.onReturnPressed: Keys.onSpacePressed(event)
                }
            }
        }
    }
    /* sd卡选择 */
    Component {
        id: dstdelegate
        Item {
            width: window.width-100
            height: 60
            Accessible.name: {
                var txt = description+" - "+(size/1000000000).toFixed(1)+" gigabytes"
                if (mountpoints.length > 0) {
                    txt += qsTr("Mounted as %1").arg(mountpoints.join(", "))
                }
                return txt;
            }
            property string description: model.description
            property string device: model.device
            property string size: model.size
            Row {
                leftPadding: 25
                Column {
                    Text {
                        textFormat: Text.StyledText
                        height: parent.parent.parent.height
                        verticalAlignment: Text.AlignVCenter
                        font.family: roboto.name
                        color: "#000"
                        text: {
                            var sizeStr = (size/1000000000).toFixed(1)+" GB";
                            var txt;
                            if (isReadOnly) {
                                txt = "<p><font size='4' color='#000'>"+description+" - "+sizeStr+"</font></p>"
                                txt += "<font color='#000'>"
                                if (mountpoints.length > 0) {
                                    txt += qsTr("Mounted as %1").arg(mountpoints.join(", "))+" "
                                }
                                txt += qsTr("[WRITE PROTECTED]")+"</font>"
                            } else {
                                txt = "<p><font size='4'>"+description+" - "+sizeStr+"</font></p>"
                                if (mountpoints.length > 0) {
                                    txt += "<font color='#000'>"+qsTr("Mounted as %1").arg(mountpoints.join(", "))+"</font>"
                                }
                            }
                            return txt;
                        }
                    }
                }
            }

            MouseArea {
                anchors.fill: parent
                cursorShape: Qt.PointingHandCursor
                hoverEnabled: true
                onClicked: {
                    selectDstItem(model)
                    sdcom.closePop()
                }
            }
        }
    }

    MsgPopup {
        id: msgpopup
        isSuccess: true
    }

    MsgPopup {
        id: errmsgpopup
        hasIcon: true
    }
    IpMsgPopup {
        id: ipmsg
        onYes: {
            // 用户选择过设置ip
            if(setip.isConfirm&&body.isA2){
                setIpData()
            }
        }
    }
    MsgPopup {
        id: quitpopup
        continueButton: false
        yesButton: true
        noButton: true
        hasIcon: true
        title: qsTr("Are you sure you want to quit?")
        text: qsTr("Ascend AI Devkit Imager is still busy.<br>Are you sure you want to quit?")
        onYes: {
            Qt.quit()
        }
    }

    MsgPopup {
        id: cancleConfirm
        continueButton: false
        yesButton: true
        noButton: true
        hasIcon: true
        onYes: {
            cancelwritebutton.enabled = false
            progressText.text = qsTr("Cancelling...")
            imageWriter.cancelWrite()
            resetWriteButton()
        }
    }

    MsgPopup {
        id: confirmwritepopup
        continueButton: false
        yesButton: true
        noButton: true
        title: qsTr("Warning")
        onYes: {
            writebutton.enabled = false
            cancelwritebutton.enabled = true
            cancelwritebutton.visible = true
            cancelverifybutton.enabled = true
            progressText.visible = true
            progressBar.visible = true
            progressBar.indeterminate = true
            progressBar.Material.accent = "#3F4BEF"
            sdcardinfo.visible = false
            writebutton.visible = false
            sdcardbox.state = "disabled"
            operatebox.state = "disabled"
            imageWriter.setVerifyEnabled(false)
            imageWriter.startWrite()
        }

        function askForConfirmation()
        {
            if(imageWriter.getOperation() == 4){
                cancelwritebutton.text = qsTr("CANCEL FORMAT")
                progressText.text = qsTr("Preparing to format...");
            } else {
                cancelwritebutton.text = qsTr("CANCEL WRITE")
                progressText.text = qsTr("Preparing to write...");
            }
            text = qsTr("All existing data on '%1' will be erased.<br>Are you sure you want to continue?").arg(sdcom.currentText)
            openPopup()
        }
    }

    UpdatePopup {
        id: updatepopup
        property url url
        title: qsTr("Update available")
        text: qsTr("检测到有新版本,是否更新？")
        onYes: {
            imageWriter.httpDownLoad(url,"./")
        }
    }

    OptionsPopup {
        id: optionspopup
    }

    UseSavedSettingsPopup {
        id: usesavedsettingspopup
        onYes: {
            optionspopup.initialize()
            optionspopup.applySettings()
            confirmwritepopup.askForConfirmation()
        }
        onNo: {
            imageWriter.clearSavedCustomizationSettings()
            confirmwritepopup.askForConfirmation()
        }
        onEditSettings: {
            optionspopup.openPopup()
        }
    }

    /* 自定义镜像url输入弹窗 */

    Popup{
        id: urlPop
        x: 100
        y: (parent.height - 200)/2
        width: parent.width-200
        padding: 0
        height: 200
        modal: true
        focus: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        property string categorySelected : ""
        Rectangle {
            color: "#1B1C2B"
            anchors.fill: parent
        }
        Rectangle {
            color: "#2A2F37"
            anchors.right: parent.right
            anchors.top: parent.top
            height: 35
            width: parent.width
        }
        Image {
            id: urlPopclose
            source: "icons/close.svg"
            sourceSize.width: 12
            sourceSize.height: 12
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.rightMargin: 25
            anchors.topMargin: 10
            MouseArea {
                anchors.fill: parent
                cursorShape: Qt.PointingHandCursor
                onClicked: {
                    urlPop.close()
                }
            }
        }
        Text {
            text: qsTr("Input Image Url")
            width: jsonurlPop.width
            height: 35
            anchors.right: parent.right
            anchors.top: parent.top
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.family: roboto.name
            font.bold: true
            color: "#ffffff"
        }
        ColumnLayout {
            id: inputBox
            width: parent.width
            height: parent.height - 35
            x:0
            y:35
            spacing: 10
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 36
                Layout.leftMargin: 20
                Layout.rightMargin: 20
                color: Qt.rgba(0,0,0,0.4)
                radius: 2
                clip: true
                TextInput{
                    font.pixelSize: 12
                    id: urlInput
                    anchors.fill: parent
                    anchors.margins: 10
                    verticalAlignment: TextInput.AlignVCenter
                    focus: true
                    color: "#ffffff"
                    selectByMouse: true
                }
            }

            Rectangle {
                Layout.fillWidth: false
                Layout.preferredWidth: 80
                Layout.preferredHeight: 32
                Layout.leftMargin: (urlPop.width - 80)/2
                color: "#3F4BEF"
                Text {
                    id: urltext
                    text: qsTr("Confirm Text")
                    anchors.fill: parent
                    color: "#ffffff"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                MouseArea{
                    anchors.fill: parent
                    cursorShape: Qt.PointingHandCursor
                    onClicked: {
                        imageWriter.setSrc(urlInput.text,5)
                        osbutton.text = qsTr("User-defined Mirror Address")
                        urlPop.close()
                        ospopup.close()
                        writebutton.text = qsTr("WRITE")
                        wrtitle.text = qsTr("WRITE")
                        if (imageWriter.readyToWrite()) {
                            writebutton.enabled = true
                        }
                        customizebutton.visible = imageWriter.imageSupportsCustomization()
                    }
                }
            }
        }
    }

    FileDialog{
        id: fileDialog
        acceptLabel: qsTr("Confirm")
        rejectLabel: qsTr("Cancel")
        folder: "file:///C:"
        title: qsTr("Save file path")
        fileMode: FileDialog.SaveFile
        currentFile: "file:///test.img"
        property string currentPath
        onAccepted: {
            body.isA2 = false
            var src = "internal:" + files[0]
            currentPath = files[0]
            imageWriter.setSrc(src,3)
            readbackddr.text = currentPath
            osbutton.text = qsTr("Image read back")
            writebutton.text = qsTr("Read back")
            wrtitle.text = qsTr("Read back")
            if (imageWriter.readyToWrite()) {
                writebutton.enabled = true
            }
            readback.changeStatus()
        }
    }

    Popup{
        id: jsonurlPop
        x: 100
        y: (parent.height - 200)/2
        width: parent.width-200
        padding: 0
        height: 200
        modal: true
        focus: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        property string categorySelected : ""
        Rectangle {
            color: "#1B1C2B"
            anchors.fill: parent
        }
        Rectangle {
            color: "#2A2F37"
            anchors.right: parent.right
            anchors.top: parent.top
            height: 35
            width: parent.width
        }
        Image {
            id: jsonurlPopclose
            source: "icons/close.svg"
            sourceSize.width: 12
            sourceSize.height: 12
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.rightMargin: 25
            anchors.topMargin: 10

            MouseArea {
                anchors.fill: parent
                cursorShape: Qt.PointingHandCursor
                onClicked: {
                    jsonurlPop.close()
                }
            }
        }
        Text {
            text: qsTr("Seting Image Url")
            width: jsonurlPop.width
            height: 35
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.family: roboto.name
            font.bold: true
            color: "#ffffff"
        }
        ColumnLayout {
            id: jsoninputBox
            width: parent.width
            height: parent.height - 35
            x:0
            y:35
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 36
                Layout.leftMargin: 20
                Layout.rightMargin: 20
                color: Qt.rgba(0,0,0,0.4)
                radius: 2
                clip: true
                TextInput{
                    font.pixelSize: 12
                    id: jsoninput
                    anchors.fill: parent
                    anchors.leftMargin: 10
                    verticalAlignment: TextInput.AlignVCenter
                    focus: true
                    selectByMouse: true
                    color: "#ffffff"
                }
            }

            Rectangle {
                Layout.fillWidth: false
                Layout.preferredWidth: 80
                Layout.preferredHeight: 32
                Layout.leftMargin: (jsonurlPop.width - 80)/2
                color: "#3F4BEF"
                Text {
                    id: jsonconirm
                    text: qsTr("Confirm Text")
                    color: "#ffffff"
                    anchors.fill: parent
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                MouseArea {
                    anchors.fill: parent
                    cursorShape: Qt.PointingHandCursor
                    onClicked: {
                        if (jsoninput.text) {
                           fetchOSlist(jsoninput.text)
                        }
                        jsonurlPop.close()
                    }
                }

            }
        }
    }

    MsgPopup {
        id: confirmreadbackpopup
        continueButton: false
        yesButton: true
        noButton: true
        title: qsTr("Confirm")
        onYes: {
            writebutton.enabled = false
            cancelwritebutton.enabled = true
            cancelwritebutton.visible = true
            cancelverifybutton.enabled = true
            progressText.visible = true
            progressBar.visible = true
            progressBar.indeterminate = true
            progressBar.Material.accent = "#3F4BEF"
            //
            sdcardinfo.visible = false
            writebutton.visible = false
            operatebox.state = "disabled"
            sdcardbox.state = "disabled"

            imageWriter.setReadbackCompression(true)
            imageWriter.setVerifyEnabled(false)
            imageWriter.startWrite()
        }
        onNo: {
            writebutton.enabled = false
            cancelwritebutton.enabled = true
            cancelwritebutton.visible = true
            cancelverifybutton.enabled = true
            progressText.visible = true
            progressBar.visible = true
            progressBar.indeterminate = true
            progressBar.Material.accent = "#3F4BEF"
            sdcardinfo.visible = false
            writebutton.visible = false
            operatebox.state = "disabled"
            sdcardbox.state = "disabled"
            imageWriter.setReadbackCompression(true)
            imageWriter.setVerifyEnabled(false)
            imageWriter.startWrite()
        }

        function askForConfirmation()
        {
            progressText.text = qsTr("Preparing to readback...");
            cancelwritebutton.text = qsTr("CANCEL READBACK")
            text = qsTr("Whether the read back file needs compression?")
            openPopup()
        }
    }

    function startWriteImage(){
        cancelwritebutton.text = qsTr("CANCEL WRITE")
        progressText.text = qsTr("Preparing to write...")
        writebutton.enabled = false
        cancelwritebutton.enabled = true
        cancelwritebutton.visible = true
        cancelverifybutton.enabled = true
        progressText.visible = true
        progressBar.visible = true
        progressBar.indeterminate = true
        progressBar.Material.accent = "#3F4BEF"
        sdcardinfo.visible = false
        writebutton.visible = false
        sdcardbox.state = "disabled"
        operatebox.state = "disabled"
        imageWriter.setVerifyEnabled(false)
        imageWriter.startWrite()
        var now = new Date()
    }
    Popup {
        id: setip
        visible: false
        x: 472
        y: (parent.height-height)/2
        width: 432
        height: 444
        padding: 0
        modal: true
        closePolicy: Popup.CloseOnEscape
        property bool isConfirm: false
        Rectangle {
            color: "#FEFEFE"
            anchors.fill: parent
            radius: 4
        }
        Rectangle {
            color: "#DFE5EF"
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.topMargin: 56
            height: 1
            width: parent.width
        }

        Image {
            id: ipx
            source: "icons/close.svg"
            sourceSize.width: 12
            sourceSize.height: 12
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.rightMargin: 25
            anchors.topMargin: 22
            MouseArea {
                anchors.fill: parent
                cursorShape: Qt.PointingHandCursor
                onClicked: {
                    setip.close()
                }
            }
        }
        Text {
            id: setipheader
            verticalAlignment: Text.AlignVCenter
            font.family: harmonyOSSansSCBold.name
            font.bold: true
            font.pixelSize: 16
            color: "#000"
            text: qsTr("镜像网络信息配置")
            height: 56
            x: 24
            y: 0
        }
        Text {
            id: iptip
            text: qsTr("以下为镜像默认IP信息，您可以根据PC/路由器IP信息重新填写网络信息")
            x: 36
            y: setipheader.y + setipheader.height + 16
            width: parent.width - 72
            height: 32
            color: "#1F2329"
            font{
                pixelSize: 12
                family: harmonyOSSansMedium.name
            }
            verticalAlignment: Text.AlignVCenter
            wrapMode: Text.WordWrap
        }
        RowLayout {
            id: ipbox
            width: parent.width - 72
            height: 32
            spacing: 0
            x: 36
            y: iptip.y + iptip.height + 12
            Rectangle {
                height: 32
                width: 132
                Text {
                    text: qsTr("网口信息")
                    font.pixelSize: 12
                    font.family: harmonyOSSansMedium.name
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    height: 32
                }
            }
            Rectangle {
                height: parent.height
                width: 72
                id: eth0
                color: "#E4EAF1"
                radius: 2
                border.color: "transparent"
                Image {
                    id: eth0tick
                    source: "icons/tick-copy.png"
                    Layout.preferredHeight: 16
                    Layout.preferredWidth: 16
                    sourceSize.width: 16
                    sourceSize.height: 16
                    verticalAlignment: Image.AlignVCenter
                    anchors.top: parent.top
                    anchors.right: parent.right
                    visible: false
                }
                Text{
                    id: eth0bt
                    text: qsTr("ETH0")
                    anchors.centerIn: parent
                    color: "#000000"
                    font.pixelSize: 12
                    font.family: harmonyOSSansMedium.name
                }
                MouseArea {
                    anchors.fill: parent
                    cursorShape: Qt.PointingHandCursor
                    onClicked: {
                        eth1bx.visible = false
                        eth0bx.visible = true
                        usb0bx.visible = false

                        // 变换states
                        usb0.state = ""
                        eth0.state = "click"
                        eth1.state = ""
                    }

                }
                states: [
                    State {
                        name: "click"
                        PropertyChanges {
                            target: eth0
                            color: "#ffffff"
                            border.color: "#0077FF"
                        }
                        PropertyChanges {
                            target: eth0tick
                            visible: true
                        }
                    }
                ]
            }
            Rectangle {
                height: parent.height
                width: 72
                id: eth1
                color: "#E4EAF1"
                radius: 2
                border.color: "transparent"
                Image {
                    id: eth1tick
                    source: "icons/tick-copy.png"
                    Layout.preferredHeight: 16
                    Layout.preferredWidth: 16
                    sourceSize.width: 16
                    sourceSize.height: 16
                    verticalAlignment: Image.AlignVCenter
                    anchors.top: parent.top
                    anchors.right: parent.right
                    visible: false
                }
                Text{
                    id: eth1bt
                    text: qsTr("ETH1")
                    anchors.centerIn: parent
                    color: "#000000"
                    font.pixelSize: 12
                    font.family: harmonyOSSansMedium.name
                }
                MouseArea {
                    anchors.fill: parent
                    cursorShape: Qt.PointingHandCursor
                    onClicked: {
                        eth0bx.visible = false
                        eth1bx.visible = true
                        usb0bx.visible = false

                        // 变换states
                        usb0.state = ""
                        eth0.state = ""
                        eth1.state = "click"
                    }
                }
                states: [
                    State {
                        name: "click"
                        PropertyChanges {
                            target: eth1
                            color: "#ffffff"
                            border.color: "#0077FF"
                        }
                        PropertyChanges {
                            target: eth1tick
                            visible: true
                        }
                    }
                ]
            }
            Rectangle {
                height: parent.height
                width: 72
                id: usb0
                color: "#E4EAF1"
                radius: 2
                border.color: "transparent"
                Image {
                    id: usb0tick
                    source: "icons/tick-copy.png"
                    Layout.preferredHeight: 16
                    Layout.preferredWidth: 16
                    sourceSize.width: 16
                    sourceSize.height: 16
                    verticalAlignment: Image.AlignVCenter
                    anchors.top: parent.top
                    anchors.right: parent.right
                    visible: false
                }
                Text{
                    id: usb0bt
                    text: qsTr("Type-C")
                    anchors.centerIn: parent
                    color: "#000000"
                    font.pixelSize: 12
                    font.family: harmonyOSSansMedium.name
                }
                MouseArea {
                    anchors.fill: parent
                    cursorShape: Qt.PointingHandCursor
                    onClicked: {
                        eth1bx.visible = false
                        eth0bx.visible = false
                        usb0bx.visible = true
                        // 变换states
                        usb0.state = "click"
                        eth0.state = ""
                        eth1.state = ""
                    }
                }
                states: [
                    State {
                        name: "click"
                        PropertyChanges {
                            target: usb0
                            color: "#ffffff"
                            border.color: "#0077FF"
                        }
                        PropertyChanges {
                            target: usb0tick
                            visible: true
                        }
                    }
                ]
            }
            Component.onCompleted: {
                usb0.state = ""
                eth0.state = "click"
                eth1.state = ""
            }
        }
        Rectangle {
            id: ipswipebox
            width: parent.width - 72
            height: 196
            x: 36
            y: ipbox.y + ipbox.height + 20
            SetIpInfo{
                id: eth0bx
                visible: true
                dhcp4: true
                address: ""
                mask: ""
                route: ""
                dns_pre: ""
                dns_alter: ""
                defaultAddress: "192.168.1.100"
                defaultDns_pre: "8.8.8.8"
                defaultDns_alter: "114.114.114.114"
                defaultMask: "255.255.255.0"
            }
            SetIpInfo{
                id: eth1bx
                visible: false
                dhcp4: false
                address: "192.168.137.100"
                mask: "255.255.255.0"
                route: "192.168.137.1"
                dns_pre: "8.8.8.8"
                dns_alter: "114.114.114.114"
                defaultAddress: "192.168.137.100"
                defaultDns_pre: "8.8.8.8"
                defaultDns_alter: "114.114.114.114"
                defaultMask: "255.255.255.0"
            }
            SetIpInfo{
                id: usb0bx
                visible: false
                dhcp4: false
                address: "192.168.0.2"
                mask: "255.255.255.0"
                route: ""
                dns_pre: ""
                dns_alter: ""
                defaultAddress: "192.168.0.2"
                defaultDns_pre: ""
                defaultDns_alter: ""
                defaultMask: "255.255.255.0"
            }
            ToolTip{
                id: showTip
            }
        }
        RowLayout {
            x: (parent.width - implicitWidth)/2
            y: ipswipebox.y + ipswipebox.height + 20
            spacing: 8
            ImButton {
                implicitWidth: 80
                implicitHeight: 36
                text: qsTr("确认")
                onClicked: {
                    // 检查网段是否冲突
                    var eth0ip = eth0bx.address.split(".")
                    var eth0str = eth0ip.splice(0,3).join()

                    var eth1ip = eth1bx.address.split(".")
                    var eth1str = eth1ip.splice(0,3).join()

                    var usb0ip = usb0bx.address.split(".")
                    var usb0str = usb0ip.splice(0,3).join()

                    if(eth0str === eth1str){
                        showTip.show("ETH0和ETH1网段冲突",1000)
                    } else if(eth0str === usb0str){
                        showTip.show("ETH0和USB0网段冲突",1000)
                    } else if(eth1str === usb0str){
                        showTip.show("ETH1和USB0网段冲突",1000)
                    } else {
                        setip.isConfirm = true
                        setip.visible = false
                    }                    
                }
            }
            ImButton {
                implicitWidth: 80
                implicitHeight: 36
                text: qsTr("取消")
                onClicked: {
                    setip.isConfirm = false
                    setip.visible = false
                }
                showBg: false
                showBr: true
                textColor: "#4E5865"
            }
        }

    }
    /* Utility functions */
    function httpRequest(url, callback) {
        var xhr = new XMLHttpRequest();
        xhr.timeout = 5000
        xhr.onreadystatechange = (function(x) {
            return function() {
                if (x.readyState === x.DONE)
                {
                    if (x.status === 200)
                    {
                        callback(x)
                    }
                    else
                    {
                        offlinebox.visible = true
                        onlinebox.visible = false
                        onError(qsTr("Error downloading OS list from Internet"))
                        console.log(JSON.stringify(x))
                    }
                }
            }
        })(xhr)
        xhr.open("GET", url)
        xhr.send()
    }
    function onDownUpdateexeProgress(now,total) {
        var newPos = now/(total+1)
        var text = qsTr("已下载... %1%").arg(Math.floor(newPos*100))
        updatepopup.updateProgreText(text)
        updatepopup.updateProgreBar(newPos)
    }
    /* Slots for signals imagewrite emits */
    function onDownloadProgress(now,total) {
        var newPos
        if (total) {
            newPos = now/(total+1)
            // 计算还需要多久时间
            var nowDate = new Date()
            var s = 0
            var min = 0
            if(body.startDownloadDate > 0) {
                var totalDate = nowDate.getTime() - body.startDownloadDate
                // 下载的平均速度
                var average = now / totalDate
                // 还需要多久下完
                var needTime = (total - now) / average
                var totalTime = Math.floor(needTime / 1000)
                s = Math.floor(totalTime % 60)
                min = Math.floor(totalTime / 60)
                // 限制下
                if(min>20){
                    min = 20
                }
            } else {
                body.startDownloadDate = nowDate.getTime()
            }
        } else {
            newPos = 0
        }

        if (progressBar.value !== newPos) {
            if (progressText.text === qsTr("Cancelling..."))
                return
            if(imageWriter.getOperation() === 3){
                progressText.text = qsTr("Readback... %1%，need%1min%1s").arg(Math.floor(newPos*100)).arg(min).arg(s)
            } else {
                progressText.text = qsTr("Writing... %1%，need%2min%3s").arg(Math.floor(newPos*100)).arg(min).arg(s)
            }
            progressBar.indeterminate = false
            progressBar.value = newPos
        }
    }

    function onVerifyProgress(now,total) {
        var newPos
        if (total) {
            newPos = now/total
        } else {
            newPos = 0
        }

        if (progressBar.value !== newPos) {
            if (cancelwritebutton.visible) {
                cancelwritebutton.visible = false
                cancelverifybutton.visible = true
            }

            if (progressText.text === qsTr("Finalizing..."))
                return

            progressText.text = qsTr("Verifying... %1%").arg(Math.floor(newPos*100))
            progressBar.Material.accent = "#6cc04a"
            progressBar.value = newPos
        }
    }

    function onPreparationStatusUpdate(msg) {
        progressText.text = qsTr("Preparing to write... (%1)").arg(msg)
    }
    // 初始化按钮的状态
    function resetWriteButton() {
        sdcardbox.state = ""
        operatebox.state = ""
        progressText.visible = false
        progressBar.visible = false
        sdcardinfo.visible = true
        writebutton.visible = true
        writebutton.enabled = imageWriter.readyToWrite()
        cancelwritebutton.visible = false
        cancelverifybutton.visible = false
        // 开始下载时间
        body.startDownloadDate = 0
    }

    function onError(msg) {
        errmsgpopup.title = qsTr("Error")
        errmsgpopup.text = msg
        errmsgpopup.openPopup()
        resetWriteButton()
    }
    function onInfo(msg) {
        msgpopup.title = qsTr("Info")
        msgpopup.text = msg
        msgpopup.isSuccess = false
        msgpopup.openPopup()
    }

    function onSuccess() {
        msgpopup.title = qsTr("Write Successful")
        if (osbutton.text === qsTr("Erase"))
            msgpopup.text = qsTr("<b>%1</b> has been erased<br><br>You can now remove the SD card from the reader").arg(sdcom.currentText)
        else if (imageWriter.isEmbeddedMode()) {
            //msgpopup.text = qsTr("<b>%1</b> has been written to <b>%2</b>").arg(osbutton.text).arg(dstbutton.text)
            /* Just reboot to the installed OS */
            Qt.quit()
        } else if (imageWriter.getOperation() == 3){
            msgpopup.text = qsTr("Backup succeeded, please eject SD card!")
            //msgpopup.text = qsTr("<b>%1</b> has been copied to <b>%2</b><br><br>You can now remove the SD card from the reader").arg(osbutton.text).arg(fileDialog.currentPath)
        }
        else if(imageWriter.getOperation() == 1) {
            msgpopup.text = qsTr("Written succeeded, please eject SD card!")
            //msgpopup.text = qsTr("<b>%1</b> has been written to <b>%2</b><br><br>Click the Cancel button in the Windows disk format pop-up window<br><br>You can now remove the SD card from the reader").arg(osbutton.text).arg(sdcom.currentText)
        }
        else { // == 2
            ipmsg.title = qsTr("Write Successful")
            ipmsg.msg = qsTr("Written succeeded, please eject SD card!")
        }
        if (imageWriter.isEmbeddedMode()) {
            msgpopup.continueButton = false
            msgpopup.quitButton = true
        }
        if(imageWriter.getOperation() === 2&&body.isA2) {

            // 设置显示信息
            if(setip.isConfirm){ // 用户设置过ip
                ipmsg.eth0_dhcp = eth0bx.dhcp4 ? "是" : "否"
                ipmsg.eth0_address = eth0bx.dhcp4 ? "" : eth0bx.address
                ipmsg.eth0_mask = eth0bx.dhcp4 ? "" : eth0bx.mask
                ipmsg.eth0_dns_pre = eth0bx.dhcp4 ? "" : eth0bx.dns_pre
                ipmsg.eth0_dns_alter = eth0bx.dhcp4 ? "" : eth0bx.dns_alter

                ipmsg.eth1_dhcp = eth1bx.dhcp4 ? "是" : "否"
                ipmsg.eth1_address = eth1bx.dhcp4 ? "" : eth1bx.address
                ipmsg.eth1_mask = eth1bx.dhcp4 ? "" : eth1bx.mask
                ipmsg.eth1_dns_pre = eth1bx.dhcp4 ? "" : eth1bx.dns_pre
                ipmsg.eth1_dns_alter = eth1bx.dhcp4 ? "" : eth1bx.dns_alter

                ipmsg.usb0_dhcp = usb0bx.dhcp4 ? "是" : "否"
                ipmsg.usb0_address = usb0bx.dhcp4 ? "" : usb0bx.address
                ipmsg.usb0_mask = usb0bx.dhcp4 ? "" : usb0bx.mask
                ipmsg.usb0_dns_pre = usb0bx.dhcp4 ? "" :  usb0bx.dns_pre
                ipmsg.usb0_dns_alter = usb0bx.dhcp4 ? "" : usb0bx.dns_alter

            } else { //用户未设置ip 显示默认信息
                ipmsg.eth0_dhcp = "是"
                ipmsg.eth0_address = ""
                ipmsg.eth0_mask = ""
                ipmsg.eth0_dns_pre = ""
                ipmsg.eth0_dns_alter = ""

                ipmsg.eth1_dhcp = "否"
                ipmsg.eth1_address = "192.168.137.100"
                ipmsg.eth1_mask = "24"
                ipmsg.eth1_dns_pre = "8.8.8.8"
                ipmsg.eth1_dns_alter = "114.114.114.114"

                ipmsg.usb0_dhcp = "否"
                ipmsg.usb0_address = "192.168.0.2"
                ipmsg.usb0_mask = "24"
                ipmsg.usb0_dns_pre = ""
                ipmsg.usb0_dns_alter = ""
            }
            ipmsg.openPopup()
        } else {
            msgpopup.isSuccess = true
            msgpopup.openPopup()
        }
        // 烧录完成后重置选择的sd卡
        //restsdcard()
        resetWriteButton()
    }
    function maskFormat(mask){
        if(!mask)
           return ""
        var arr = mask.split(".")
        var str = ""
        var res = ""
        for(var i = 0; i < arr.length; i++){
            str += parseInt(arr[i],10).toString(2)
        }
        res = str.indexOf("0")
        return res
    }
    function setIpData() {
        var mou = []
        for (var i = 0; i < driveListModel.rowCount(); i++)
        {
            console.log("driveListModel.rowCount()",driveListModel.rowCount())

            /* FIXME: there should be a better way to iterate drivelist than
               fetch data by numeric role number */
            if (driveListModel.data(driveListModel.index(i,0), 0x101) === body.device) {
                mou = driveListModel.data(driveListModel.index(i,0), 0x107)
                break
            }
        }
        console.log("mou",mou)
        var path
        for (var j = 0; j < mou.length; j++)
        {
            var filePath = mou[j][0] + ":/config.ini"
            console.log(filePath)
            if(imageWriter.fileIsExist(filePath))
            {
                console.log("fileIsExist",filePath)
                path = filePath
                break
            }
        }
        if(path){
            let str = "################################################################################\n"
            str += "# 特别注意！！！\n"
            str += "# 该文件用于设置开发板ip，请勿编辑任何无关内容，否则可能导致开发板无法启动\n"
            str += '# 建议在两个网口均能直连时再根据需要将一个网口设置为"自动获得ip地址"\n'
            str += "#   此时，一个网口连接路由，由路由分配ip，另一个网口直连电脑，用以调试\n\n"
            str += "# 生效标志位：设置为true，则该配置文件生效一次后会自动改成false\n"
            str += "# 若修改该文件并希望下次开机重新配置ip，将标志位改成true\n"
            str += "setting_flag=true\n\n"
            str += "################################################################################\n"
            str += "# 网口0-由路由器分配ip地址(若设置为yes，则该网口的路由和ip不会生效)\n"
            if(eth0bx.dhcp4){
                str += "eth0_dhcp4=yes\n\n"
            } else {
                str += "eth0_dhcp4=no\n\n"
            }
            str += "# 网口0-ip地址\n"
            str += "eth0_address=" + eth0bx.address + "\n\n"
            str += "# 网口0-掩码位数\n"
            str += "eth0_mask=" + maskFormat(eth0bx.mask) + "\n\n"
            str += "# 网口0-路由(多个ip配置路由，仅有首个会生效)\n"
            str += "eth0_route=" + eth0bx.route + "\n\n"
            str += "# 网口0-域名地址-首选域名\n"
            str += "eth0_dns_pre=" + eth0bx.dns_pre + "\n\n"
            str += "# 网口0-域名地址-备选域名\n"
            str += "eth0_dns_alter=" + eth0bx.dns_alter + "\n\n"
            str += "################################################################################\n"
            str += "# 网口1-由路由器分配ip地址(若设置为yes，则该网口的路由和ip不会生效)\n"
            if(eth1bx.dhcp4){
                str += "eth1_dhcp4=yes\n\n"
            } else {
                str += "eth1_dhcp4=no\n\n"
            }
            str += "# 网口1-ip地址\n"
            str += "eth1_address=" + eth1bx.address + "\n\n"
            str += "# 网口1-掩码位数\n"
            str += "eth1_mask=" + maskFormat(eth1bx.mask) + "\n\n"
            str += "# 网口1-路由(多个ip配置路由，仅有首个会生效)\n"
            str += "eth1_route=" + eth1bx.route + "\n\n"
            str += "# 网口1-域名地址-首选域名\n"
            str += "eth1_dns_pre=" + eth1bx.dns_pre + "\n\n"
            str += "# 网口1-域名地址-备选域名\n"
            str += "eth1_dns_alter=" + eth1bx.dns_alter + "\n\n"
            str += "################################################################################\n"
            str += "# usb-由路由器分配ip地址(若设置为yes，则该网口的路由和ip不会生效)\n"
            if(usb0bx.dhcp4){
                str += "usb0_dhcp4=yes\n\n"
            } else {
                str += "usb0_dhcp4=no\n\n"
            }
            str += "# typeC-ip地址\n"
            str += "usb0_address=" + usb0bx.address + "\n\n"
            str += "# typeC-掩码位数\n"
            str += "usb0_mask=" + maskFormat(usb0bx.mask) + "\n\n"
            str += "# typeC-路由(多个ip配置路由，仅有首个会生效)\n"
            str += "usb0_route=" + usb0bx.route + "\n\n"
            str += "# typeC-域名地址-首选域名\n"
            str += "usb0_dns_pre=" + usb0bx.dns_pre + "\n\n"
            str += "# typeC-域名地址-备选域名\n"
            str += "usb0_dns_alter=" + usb0bx.dns_alter
            console.log(str)
            imageWriter.writeIpInfo(path, str)
        }
    }

    function restsdcard(){
        imageWriter.setDst("")
        sdcom.currentText = qsTr("Please select (unselected)")
        sdinfo.text = qsTr("Please insert SD card")
        sdbox.state = ""
    }
    function onFileSelected(file) {
        body.isA2 = false
        imageWriter.setSrc(file, 1)

        loacltips.text = imageWriter.srcFileName()
        osbutton.text = imageWriter.srcFileName()
        //ospopup.close()
        if (imageWriter.readyToWrite()) {
            writebutton.enabled = true
        }
        writebutton.text = qsTr("WRITE")
        wrtitle.text =  qsTr("WRITE")
        /*获取文件类型并替换对应的图标*/
        var filename = imageWriter.srcFileName()
        var filearr =  filename.split(".")
        var filetype = filearr[filearr.length - 1]
        localicon.source = "/icons/file-" + filetype +".svg"
        localbox.changeStatus()
        //customizebutton.visible = imageWriter.imageSupportsCustomization()
    }

    function onCancelled() {
        resetWriteButton()
    }

    function onFinalizing() {
        progressText.text = qsTr("Finalizing...")
    }

    function shuffle(arr) {
        for (var i = 0; i < arr.length - 1; i++) {
            var j = i + Math.floor(Math.random() * (arr.length - i));
            var t = arr[j];
            arr[j] = arr[i];
            arr[i] = t;
        }
    }

    function checkForRandom(list) {
        for (var i in list) {
            var entry = list[i]

            if ("subitems" in entry) {
                checkForRandom(entry["subitems"])
                if ("random" in entry && entry["random"]) {
                    shuffle(entry["subitems"])
                }
            }
        }
    }

    function oslistFromJson(o) {
        var oslist = false
        var lang_country = Qt.locale().name
        if ("os_list_"+lang_country in o) {
            oslist = o["os_list_"+lang_country]
        }
        else if (lang_country.includes("_")) {
            var lang = lang_country.substr(0, lang_country.indexOf("_"))
            if ("os_list_"+lang in o) {
                oslist = o["os_list_"+lang]
            }
        }

        if (!oslist) {
            if (!"os_list" in o) {
                onError(qsTr("Error parsing os_list.json"))
                return false
            }

            oslist = o["os_list"]
        }

        checkForRandom(oslist)

        /* Flatten subitems to subitems_json */
        for (var i in oslist) {
            var entry = oslist[i];
            if ("subitems" in entry) {
                entry["subitems_json"] = JSON.stringify(entry["subitems"])
                delete entry["subitems"]
            }
        }

        return oslist
    }

    function selectNamedOS(name, collection)
    {
        for (var i = 0; i < collection.count; i++) {
            var os = collection.get(i)

            if (typeof(os.subitems_json) == "string" && os.subitems_json != "") {
                selectNamedOS(name, os.subitems_json)
            }
            else if (typeof(os.url) !== "undefined" && name === os.name) {
                selectOSitem(os, false)
                break
            }
        }
    }

    function fetchOSlist(url) {
        var currentUrl = url ? url : imageWriter.constantOsListUrl()
        httpRequest(currentUrl, function (x) {
            var o = JSON.parse(x.responseText)
            var oslist = oslistFromJson(o)
            if (oslist === false) {
                offlinebox.visible = true
                onlinebox.visible = false
                return
            }
            // 如果自定义json url需要删除之前的渲染的节点，OS列表中固定的3个 不需要移除
            if(url){
                var count = osmodel.count - 3
                osmodel.remove(0,count)
            }
            for (var i in oslist) {
                if(oslist[i].name === "Atlas 200I DK A2"){
                    var dka2model = []
                    var subitems = JSON.parse(oslist[i].subitems_json)
                    for(var j in subitems) {
                        dka2model.push(subitems[j].name)
                        dka2.imagelist.push(subitems[j])
                    }
                    dkacombox.model = dka2model
                    dka2list.append(dka2.imagelist[0])
                    // windwos下 如果localurl不为空 代表用户是打开镜像文件通过制卡工具打开的
                    if(localurl){
                        onFileSelected(localurl)
                        swview.setCurrentIndex(1)
                    } else { // 默认选择第一个镜像
                        selectOSitem(dka2.imagelist[0])
                        dka2.isClick = true
                    }
                }
                if(oslist[i].name === "Atlas 200 DK"){
                    var dkmodel = []
                    var dksubitems = JSON.parse(oslist[i].subitems_json)
                    for(var k in dksubitems) {
                        dkmodel.push(dksubitems[k].name)
                        dk.imagelist.push(dksubitems[k])
                    }
                    dkcombox.model = dkmodel
                    dklist.append(dk.imagelist[0])
                }
            }
            dkacombox.visible = true
            dkcombox.visible = true
            offlinebox.visible = false
            onlinebox.visible = true
            if ("imager" in o) {
                var imager = o["imager"]
                // 设置反馈社区地址
                if("feed_back_url" in imager) {
                    body.feedBackUrl = imager["feed_back_url"]
                }
                // 检查版本
                if (imageWriter.getBoolSetting("check_version") && "latest_version" in imager && "url" in imager) {
                    if (!imageWriter.isEmbeddedMode() && imageWriter.isVersionNewer(imager["latest_version"])) {
                        updatepopup.url = imager["url"]
                        updatepopup.openPopup()
                    }
                }
                if ("default_os" in imager) {
                    selectNamedOS(imager["default_os"], osmodel)
                }
                if (imageWriter.isEmbeddedMode()) {
                    if ("embedded_default_os" in imager) {
                        selectNamedOS(imager["embedded_default_os"], osmodel)
                    }
                    if ("embedded_default_destination" in imager) {
                        imageWriter.startDriveListPolling()
                        setDefaultDest.drive = imager["embedded_default_destination"]
                        setDefaultDest.start()
                    }
                }
            }
        })
    }
    /*定时器*/
    Timer {
        /* Verify if default drive is in our list after 100 ms */
        id: setDefaultDest
        property string drive : ""
        interval: 100
        onTriggered: {
            for (var i = 0; i < driveListModel.rowCount(); i++)
            {
                /* FIXME: there should be a better way to iterate drivelist than
                   fetch data by numeric role number */
                if (driveListModel.data(driveListModel.index(i,0), 0x101) === drive) {
                    selectDstItem({
                        device: drive,
                        description: driveListModel.data(driveListModel.index(i,0), 0x102),
                        size: driveListModel.data(driveListModel.index(i,0), 0x103),
                        readonly: false
                    })
                    break
                }
            }
        }
    }

    function newSublist() {
        if (osswipeview.currentIndex == (osswipeview.count-1))
        {
            var newlist = suboslist.createObject(osswipeview)
            osswipeview.addItem(newlist)
        }

        var m = osswipeview.itemAt(osswipeview.currentIndex+1).model

        if (m.count>1)
        {
            m.remove(1, m.count-1)
        }

        return m
    }
    /* 选择镜像列表 */
    function selectOSitem(d, selectFirstSubitem)
    {
        /* 镜像列表下还有subitems */
        if (typeof(d.subitems_json) == "string" && d.subitems_json !== "") {
            // console.log("镜像列表下还有subitems")
            /* 创建镜像弹框 */
            var m = newSublist()
            var subitems = JSON.parse(d.subitems_json)

            for (var i in subitems)
            {
                var entry = subitems[i];
                if ("subitems" in entry) {
                    /* Flatten sub-subitems entry */
                    entry["subitems_json"] = JSON.stringify(entry["subitems"])
                    delete entry["subitems"]
                }
                m.append(entry)
            }

            osswipeview.itemAt(osswipeview.currentIndex+1).currentIndex = (selectFirstSubitem === true) ? 0 : -1
            osswipeview.incrementCurrentIndex()
            ospopup.categorySelected = d.name
            /* 子列表为url的情况 */
        } else if (typeof(d.subitems_url) == "string" && d.subitems_url !== "") {
            /*返回上一级*/
            if (d.subitems_url === "internal://back")
            {
                osswipeview.decrementCurrentIndex()
                ospopup.categorySelected = ""
            }
            /*请求镜像列表url*/
            else
            {
                ospopup.categorySelected = d.name
                var suburl = d.subitems_url
                var m = newSublist()

                httpRequest(suburl, function (x) {
                    var o = JSON.parse(x.responseText)
                    var oslist = oslistFromJson(o)
                    if (oslist === false)
                        return
                    for (var i in oslist) {
                        m.append(oslist[i])
                    }
                })

                osswipeview.itemAt(osswipeview.currentIndex+1).currentIndex = (selectFirstSubitem === true) ? 0 : -1
                osswipeview.incrementCurrentIndex()
            }
        } else if (d.url === "") {
            /*选择本地文件*/
            if (!imageWriter.isEmbeddedMode()) {
                imageWriter.openFileDialog()
            }
            else {
                // 选择sd卡
                if (imageWriter.mountUsbSourceMedia()) {
                    var m = newSublist()

                    var oslist = JSON.parse(imageWriter.getUsbSourceOSlist())
                    for (var i in oslist) {
                        m.append(oslist[i])
                    }
                    osswipeview.itemAt(osswipeview.currentIndex+1).currentIndex = (selectFirstSubitem === true) ? 0 : -1
                    osswipeview.incrementCurrentIndex()
                }
                else
                {
                    onError(qsTr("Connect an USB stick containing images first.<br>The images must be located in the root folder of the USB stick."))
                }
            }
        } else if(d.url === "input"){
            urlPop.open()
        } else if(d.url === "internal://readback") {
            fileDialog.open()
        }else {
            var urlType = d.url == "internal://format" ? 4 : 2
            imageWriter.setSrc(d.url, urlType , d.image_download_size, d.extract_size, typeof(d.extract_sha256) != "undefined" ? d.extract_sha256 : "", typeof(d.contains_multiple_files) != "undefined" ? d.contains_multiple_files : false,
                               ospopup.categorySelected, d.name, typeof(d.init_format) != "undefined" ? d.init_format : "", typeof(d.is_extensible) != "undefined" ? d.is_extensible : false)
            osbutton.text = d.name
            if(urlType === 4){
                writebutton.text = qsTr("Format")
            } else {
                writebutton.text = qsTr("WRITE")
                wrtitle.text = qsTr("WRITE")
            }
            
            if (imageWriter.readyToWrite()) {
                writebutton.enabled = true
            }
        }
    }

    function selectDstItem(d) {
        body.device = d.device
        if (d.isReadOnly) {
            onError(qsTr("SD card is write protected.<br>Push the lock switch on the left side of the card upwards, and try again."))
            return
        }
        imageWriter.setDst(d.device, d.size)
        if (imageWriter.readyToWrite()) {
            writebutton.enabled = true
        }
        sdcom.currentText = d.description
        var sizeStr = (d.size/1000000000).toFixed(1)+" GB";
        sdinfo.text =  qsTr("Memory %1").arg(sizeStr)
    }
}
