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
 * 2022.08.26-Add MsgPopup add icon
 *                 Huawei Technologies Co., Ltd.
 *
 */

import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.0
import QtQuick.Controls.Material 2.2

Popup {
    id: updatepopup
    x: 472
    y: (parent.height-height)/2
    modal: true
    width: 432
    height: msgpopupbody.implicitHeight+120
    padding: 0
    closePolicy: Popup.CloseOnEscape
    property alias title: msgpopupheader.text
    property alias text: msgpopupbody.text
    signal yes()
    signal no()

    Rectangle {
        color: "#fff"
        anchors.fill: parent
    }
    // background of title
    Rectangle {
        color: "#ccc"
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.topMargin: 34
        height: 1
        width: parent.width
    }
    ColumnLayout {
        anchors.fill: parent
        Text {
            id: msgpopupheader
            verticalAlignment: Text.AlignVCenter
            Layout.fillWidth: true
            Layout.topMargin: 10
            Layout.leftMargin: 24
            Layout.rightMargin: 24
            font.family: harmonyOSSansSCBold.name
            font.bold: true
            color: "#000"
        }

        RowLayout {
            Layout.topMargin: 20
            Layout.leftMargin: 24
            Layout.rightMargin: 24
            Text {
                id: msgpopupbody
                Layout.fillHeight: true
                font.pixelSize: 12
                wrapMode: Text.WordWrap
                textFormat: Text.StyledText
                font.family: harmonyOSSansSCRegular.name
                Layout.maximumWidth: msgpopup.width - 68
                Accessible.name: text.replace(/<\/?[^>]+(>|$)/g, "")
                color: "#000"
                verticalAlignment: Text.AlignVCenter
            }
        }
        Text {
            id: progreText
            visible: false
            Layout.leftMargin: 24
            Layout.rightMargin: 24
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            text: qsTr("正在下载...")
            font.pixelSize: 12
            color: "#4E5865"
            font.family: harmonyOSSansMedium.name
        }
        ProgressBar {
            id: progreBar
            visible: false
            Layout.leftMargin: 24
            Layout.rightMargin: 24
            Layout.topMargin: 6
            Layout.fillWidth: true
            Material.background: "#DFE5EF"
        }
        RowLayout {
            Layout.alignment: Qt.AlignCenter | Qt.AlignBottom
            Layout.bottomMargin: 10
            spacing: 20
            ImButton {
                id: cancele
                text: qsTr("取消")
                onClicked: {
                    updatepopup.close()
                    updatepopup.no()
                }
            }

            ImButton {
                id: confirme
                text: qsTr("更新")
                onClicked: {
                    cancele.visible = false
                    confirme.visible = false
                    progreBar.Material.accent = "#3F4BEF"
                    progreText.visible = true
                    progreBar.visible = true
                    updatepopup.yes()
                }
            }

        }
    }

    function openPopup() {
        open()
        // trigger screen reader to speak out message
        msgpopupbody.forceActiveFocus()
    }
    function closePopup() {
        close()
    }
    function updateProgreText(text){
        progreText.text = text
    }
    function updateProgreBar(newPos){
       progreBar.value = newPos
    }
}
