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
 * 2022.08.26-Add Button change color
 *                 Huawei Technologies Co., Ltd.
 * 
 */

import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.0
import QtQuick.Controls.Styles 1.4

Button {
    id: btn
    font.family: roboto.name
    Accessible.onPressAction: clicked()
    Keys.onEnterPressed: clicked()
    Keys.onReturnPressed: clicked()
    implicitWidth: 66
    implicitHeight: 32
    property color textColor: "#FFFFFF"
    property color brColor: "#C3CEDF"
    property bool showBg: true
    property bool showBr: false
    background: Rectangle {
        id: btnbg
        color: btn.showBg ? btn.enabled ? "#0077FF" : "#B8D9FF" : "transparent"
        radius: 3
        border.color: btn.showBr ? btn.brColor : "transparent"
    }
    contentItem: Text {
        text: btn.text
        color: btn.textColor
        font.pixelSize: 12
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        font.family: harmonyOSSansMedium.name
    }

    MouseArea {
        anchors.fill: parent
        cursorShape: Qt.PointingHandCursor
        hoverEnabled: true
        onClicked: {
            btn.clicked()
        }
        onExited: {
            btnbg.color = btn.showBg ? btn.enabled ? "#0077FF" : "#B8D9FF" : "transparent"
        }
        onEntered: {
            btnbg.color = btn.showBg ? btn.enabled ? "#52A3FF" : "#B8D9FF" : "transparent"
        }
    }
    onEnabledChanged: {
        btnbg.border.color = btn.showBr ? btn.brColor : "transparent"
        btnbg.color = btn.showBg ? btn.enabled ? "#0077FF" : "#B8D9FF" : "transparent"
    }
    onShowBgChanged: {
        btnbg.border.color = btn.showBr ? btn.brColor : "transparent"
        btnbg.color = btn.showBg ? btn.enabled ? "#0077FF" : "#B8D9FF" : "transparent"
    }
    onShowBrChanged: {
        btnbg.border.color = btn.showBr ? btn.brColor : "transparent"
        btnbg.color = btn.showBg ? btn.enabled ? "#0077FF" : "#B8D9FF" : "transparent"
    }
}
