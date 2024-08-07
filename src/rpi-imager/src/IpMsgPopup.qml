
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
    id: msgbx
    x: (parent.width-width)/2
    y: (parent.height-height)/2
    width: 608
    height: 340
    padding: 0
    modal: true
    closePolicy: Popup.CloseOnEscape
    property alias msg: msgpopupbody.text
    property alias title: titletext.text

    property alias eth0_dhcp: eth0.dhcp
    property alias eth0_address: eth0.address
    property alias eth0_mask: eth0.mask
    property alias eth0_dns_pre: eth0.dns_pre
    property alias eth0_dns_alter: eth0.dns_alter

    property alias eth1_dhcp: eth1.dhcp
    property alias eth1_address: eth1.address
    property alias eth1_mask: eth1.mask
    property alias eth1_dns_pre: eth1.dns_pre
    property alias eth1_dns_alter: eth1.dns_alter

    property alias usb0_dhcp: usb0.dhcp
    property alias usb0_address: usb0.address
    property alias usb0_mask: usb0.mask
    property alias usb0_dns_pre: usb0.dns_pre
    property alias usb0_dns_alter: usb0.dns_alter
    signal yes()
    Rectangle {
        color: "#fff"
        anchors.fill: parent
    }
    // background of title
    Rectangle {
        color: "#EBEFF6"
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.topMargin: 56
        height: 1
        width: parent.width
    }
    Rectangle{
        id: title
        height: 56
        width: parent.width - 24
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.leftMargin: 24
        Text {
            id: titletext
            anchors.fill: parent
            font.pixelSize: 16
            color: "#000000"
            font.bold: true
            verticalAlignment: Text.AlignVCenter
            font.family: harmonyOSSansSCBold.name
        }
    }
    Rectangle {
        id: infobox
        height: 20
        anchors.top: title.bottom
        anchors.topMargin: 24
        anchors.left: parent.left
        anchors.leftMargin: 24
        width: parent.width - 24
        RowLayout {
            anchors.fill: parent
            spacing: 0
            Image {
                source: "icons/success.svg"
                width: 20
                height: 20
                visible: true
                sourceSize.width: 20
                sourceSize.height: 20
                Layout.alignment: Qt.AlignHCenter
            }
            Text {
                id: msgpopupbody
                Layout.fillHeight: true
                Layout.fillWidth: true
                font.pixelSize: 12
                wrapMode: Text.WordWrap
                textFormat: Text.StyledText
                font.family: harmonyOSSansSCRegular.name
                Layout.maximumWidth: msgpopup.width - 68
                Accessible.name: text.replace(/<\/?[^>]+(>|$)/g, "")
                color: "#4E5865"
                verticalAlignment: Text.AlignVCenter
            }
        }
    }
    Rectangle {
        id: iptitle
        height: 20
        width: parent.width - 50
        anchors.left: parent.left
        anchors.leftMargin: 50
        anchors.top: infobox.bottom
        anchors.topMargin: 3
        Text {
            anchors.fill: parent
            verticalAlignment: Text.AlignVCenter
            color: "#8D98AA"
            text: qsTr("镜像网络信息:")
            font.pixelSize: 12
        }
    }
    Rectangle {
        id: ipinfo
        height: 160
        width: 540
        anchors.left: parent.left
        anchors.leftMargin: 40
        anchors.top: iptitle.bottom
        ColumnLayout {
            id: info
            anchors.fill: parent
            spacing: 6
            RowLayout {
                Rectangle {
                    width: 174
                    height: 155
                    border.color: "#EBEFF6"
                    radius: 2
                    ShowIp {
                        id: eth0
                        tiletext: "ETH0"
                    }
                }
                Rectangle {
                    width: 174
                    height: 155
                    border.color: "#EBEFF6"
                    radius: 2
                    ShowIp {
                        id: eth1
                        tiletext: "ETH1"
                    }
                }
                Rectangle {
                    width: 174
                    height: 155
                    border.color: "#EBEFF6"
                    radius: 2
                    ShowIp {
                        id: usb0
                        tiletext: "Type-C"
                    }
                }
            }
        }
        Rectangle {
            height: 24
            width: parent.width
            anchors.left: parent.left
            anchors.top: ipinfo.bottom
            anchors.topMargin: 18
            ImButton {
                anchors.centerIn: parent
                text: qsTr("完成")
                onClicked: {
                    msgbx.yes()
                    msgbx.close()
                }
            }

        }
    }
    function openPopup() {
        open()
        // trigger screen reader to speak out message
//        msgpopupbody.forceActiveFocus()
    }
}
