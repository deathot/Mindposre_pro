import QtQuick 2.0
import QtQuick.Layouts 1.0

ColumnLayout{
    property alias tiletext: tile.text
    property alias address: ip.text
    property alias mask: mask.text
    property alias dns_pre: dns_pre.text
    property alias dns_alter: dns_alter.text
    property alias dhcp: dhcp.text
    Rectangle {
        Layout.topMargin: 2
        Layout.leftMargin: 2
        width: 172
        height: 20
        Text {
            id: tile
            anchors.left: parent.left
            anchors.leftMargin: 10
            color: "#0077FF"
            height: 19
            verticalAlignment: Text.AlignVCenter
            font.family: harmonyOSSansSCRegular.name
        }
        Rectangle {
            color: "#EBEFF6"
            anchors.top: tile.bottom
            anchors.left: parent.left
            height: 1
            width: 172
        }
    }
    RowLayout{
        height: 20
        width: 147
        Layout.leftMargin: 10
        Layout.rightMargin: 10
        spacing: 0
        Rectangle {
            width: 60
            height: 20
            Text {
                id: dhcplabel
                height: 20
                text: qsTr("自动获取IP")
                color: "#C3CEDF"
                font.pixelSize: 12
                verticalAlignment: Text.AlignVCenter
                font.family: harmonyOSSansSCRegular.name
            }
        }
        Rectangle {
            Layout.fillWidth: true
            height: 20
            Text {
                id: dhcp
                height: 20
                text: qsTr("是")
                color: "#8D98AA"
                font.pixelSize: 12
                verticalAlignment: Text.AlignVCenter
                font.family: harmonyOSSansSCRegular.name
            }
        }
    }
    RowLayout{
        height: 20
        width: 147
        Layout.leftMargin: 10
        Layout.rightMargin: 10
        spacing: 0
        Rectangle {
            width: 60
            height: 20
            Text {
                id: iplabel
                height: 20
                text: qsTr("IP")
                color: "#C3CEDF"
                font.pixelSize: 12
                verticalAlignment: Text.AlignVCenter
                font.family: harmonyOSSansSCRegular.name
            }
        }
        Rectangle {
            Layout.fillWidth: true
            height: 20
            Text {
                id: ip
                height: 20
                text: qsTr("255.255.255.255")
                color: "#8D98AA"
                font.pixelSize: 12
                verticalAlignment: Text.AlignVCenter
                font.family: harmonyOSSansSCRegular.name
            }
        }
    }
    RowLayout{
        height: 20
        width: 147
        Layout.leftMargin: 10
        Layout.rightMargin: 10
        spacing: 0
        Rectangle {
            width: 60
            height: 20
            Text {
                height: 20
                text: qsTr("子网掩码")
                color: "#C3CEDF"
                font.pixelSize: 12
                verticalAlignment: Text.AlignVCenter
                font.family: harmonyOSSansSCRegular.name
            }
        }
        Rectangle {
            Layout.fillWidth: true
            height: 20
            Text {
                id: mask
                height: 20
                text: qsTr("255.255.255.255")
                color: "#8D98AA"
                font.pixelSize: 12
                verticalAlignment: Text.AlignVCenter
                font.family: harmonyOSSansSCRegular.name
            }
        }
    }
    RowLayout{
        height: 20
        width: 147
        Layout.leftMargin: 10
        Layout.rightMargin: 10
        spacing: 0
        Rectangle {
            width: 60
            height: 20
            Text {
                height: 20
                text: qsTr("域名")
                color: "#C3CEDF"
                font.pixelSize: 12
                verticalAlignment: Text.AlignVCenter
                font.family: harmonyOSSansSCRegular.name
            }
        }
        Rectangle {
            Layout.fillWidth: true
            height: 20
            Text {
                id: dns_pre
                height: 20
                text: qsTr("255.255.255.255")
                color: "#8D98AA"
                font.pixelSize: 12
                verticalAlignment: Text.AlignVCenter
                font.family: harmonyOSSansSCRegular.name
            }
        }
    }
    RowLayout{
        height: 20
        width: 147
        Layout.leftMargin: 10
        Layout.rightMargin: 10
        spacing: 0
        Rectangle {
            width: 60
            height: 20
            Text {
                height: 20
                text: qsTr("备选域名")
                color: "#C3CEDF"
                font.pixelSize: 12
                verticalAlignment: Text.AlignVCenter
                font.family: harmonyOSSansSCRegular.name
            }
        }
        Rectangle {
            Layout.fillWidth: true
            height: 20
            Text {
                id: dns_alter
                height: 20
                text: qsTr("255.255.255.255")
                color: "#8D98AA"
                font.pixelSize: 12
                verticalAlignment: Text.AlignVCenter
                font.family: harmonyOSSansSCRegular.name
            }
        }
    }
}
