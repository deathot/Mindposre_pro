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
import "qmlcomponents"

Popup {
    id: msgpopup
    x: 472
    y: (parent.height-height)/2
    modal: true
    width: 432
    height: msgpopupbody.implicitHeight+150
    padding: 0
    closePolicy: Popup.CloseOnEscape

    property alias title: msgpopupheader.text
    property alias text: msgpopupbody.text
    property bool continueButton: true
    property bool quitButton: false
    property bool yesButton: false
    property bool noButton: false
    property bool hasIcon: false
    property bool isSuccess: false
    signal yes()
    signal no()
    signal setIp()

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
    Image {
        id: msgx
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
                msgpopup.close()
            }
        }
    }

    ColumnLayout {
        spacing: 20
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
            Image {
                source: msgpopup.hasIcon ? "icons/alarm.svg" : msgpopup.isSuccess ? "icons/success.svg" : ""
                width: 20
                height: 20
                visible: msgpopup.hasIcon || msgpopup.isSuccess
                sourceSize.width: 20
                sourceSize.height: 20
                Layout.alignment: Qt.AlignHCenter
            }
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

        RowLayout {
            Layout.alignment: Qt.AlignCenter | Qt.AlignBottom
            Layout.bottomMargin: 10
            spacing: 20
            ImButton {
                text: qsTr("NO")
                onClicked: {
                    msgpopup.close()
                    msgpopup.no()
                }
                visible: msgpopup.noButton
            }

            ImButton {
                text: qsTr("YES")
                onClicked: {
                    msgpopup.close()
                    msgpopup.yes()
                }
                visible: msgpopup.yesButton
            }

            ImButton {
                text: qsTr("CONTINUE")
                onClicked: {
                    msgpopup.close()
//                    msgpopup.setIp()
                }
                visible: msgpopup.continueButton
            }

            ImButton {
                text: qsTr("QUIT")
                onClicked: {
                    Qt.quit()
                }
                font.family: roboto.name
                visible: msgpopup.quitButton
            }

        }
    }

    function openPopup() {
        open()
        // trigger screen reader to speak out message
        msgpopupbody.forceActiveFocus()
    }
}
