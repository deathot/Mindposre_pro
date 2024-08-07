import QtQuick 2.0
import QtQuick 2.6
import QtQuick.Controls 2.2
//自定义单选按钮
RadioButton {
    id: control
    spacing: 0
    padding: 0
    property alias lbtext: lb.text
    implicitHeight: 16
    indicator: Rectangle {
        implicitWidth: 13
        implicitHeight: 13
        x: 0
        y: parent.height / 2 - height / 2
        radius: 6.5
        border.color: control.down ? "#1F2329" : "#0077FF"
        Rectangle {
            width: 8
            height: 8
            x: 2.5
            y: 2.5
            radius: 4
            color: control.down ? "#1F2329" : "#0077FF"
            visible: control.checked
        }
    }

    contentItem:Text {
        id: lb
        opacity: enabled ? 1.0 : 0.3
        color: "#1F2329"
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
         //@disable-check M16
        leftPadding: control.indicator.implicitWidth + 9
        font.family: harmonyOSSansMedium.name
     }
}
