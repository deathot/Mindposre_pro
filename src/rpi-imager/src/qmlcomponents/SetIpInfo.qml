import QtQuick 2.9
import QtQuick.Layouts 1.0

Rectangle {
    id: box
    width: parent.width
    height: parent.height
    property string address: ""
    property string route: ""
    property string dns_pre: ""
    property string dns_alter: ""
    property bool dhcp4: false
    property string mask: ""
    property string defaultAddress: ""
    property string defaultDns_pre: ""
    property string defaultDns_alter: ""
    property string defaultMask: ""
    MouseArea {
        id: msk
        visible: dhcp4
        width: parent.width
        height: parent.height - 16
        anchors.top: parent.top
        anchors.topMargin: 16
        enabled: true
        z: 99
    }
    ColumnLayout {
        id: ipbox
        width: parent.width
        height: parent.height
        spacing: 12
        RowLayout {
            height: 16
            Layout.fillWidth: true
            spacing: 0
            Radio{
                lbtext: qsTr("自动获取IP地址")
                checked: box.dhcp4
                onCheckedChanged: {
                    box.dhcp4 = checked
                    msk.visible = checked
                    address.enabled = checked
                    dnsalter.enabled = checked
                    dnspre.enabled = checked
                    mask.enabled = checked
                    if(checked){
                        address.setDefaultIp("")
                        mask.setDefaultIp("")
                        dnspre.setDefaultIp("")
                        dnsalter.setDefaultIp("")
                    } else {
                        address.setDefaultIp(box.defaultAddress)
                        mask.setDefaultIp(box.defaultMask)
                        dnspre.setDefaultIp(box.defaultDns_pre)
                        dnsalter.setDefaultIp(box.defaultDns_alter)
                    }
                }
            }
            Radio{
                Layout.leftMargin: 64
                lbtext: qsTr("使用下面的IP地址")
                checked: !box.dhcp4
            }
        }
        RowLayout {
            height: 32
            Layout.fillWidth: true
            spacing: 0
            Rectangle {
                height: 32
                width: 132
                Text {
                    text: qsTr("IP地址")
                    font.pixelSize: 12
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    height: 32
                    color: "#1F2329"
                    font.family: harmonyOSSansMedium.name
                }
            }
            IpInput{
                id: address
                height: 32
                Layout.leftMargin: 8
                ip: box.address
                enabled: box.dhcp4
                onIpChanged: {
                    box.address = ip
                }
            }
       }
       RowLayout {
           height: 32
           Layout.fillWidth: true
           spacing: 0
            Rectangle {
               width: 132
               height: 32
                Text {
                    text: qsTr("子网掩码")
                    font.pixelSize: 12
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    height: 32
                    color: "#4E5865"
                    font.family: harmonyOSSansMedium.name
                }
            }
            IpInput{
                id: mask
                height: 32
                Layout.leftMargin: 8
                ip: box.mask
                onIpChanged: {
                    box.mask = ip
                }

            }
        }
        RowLayout {
            height: 32
            Layout.fillWidth: true
            spacing: 0
            Rectangle {
                width: 132
                height: 32
                Text {
                    text: qsTr("首选DNS服务器")
                    font.pixelSize: 12
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    height: 32
                    color: "#4E5865"
                    font.family: harmonyOSSansMedium.name
                }
            }
            IpInput{
                id: dnspre
                height: 32
                Layout.leftMargin: 8
                ip: box.dns_pre
                onIpChanged: {
                    box.dns_pre = ip
                }

            }
        }
        RowLayout {
            height: 32
            Layout.fillWidth: true
            spacing: 0
            Rectangle {
               width: 132
               height: 32
                Text {
                    text: qsTr("备选DNS服务器")
                    font.pixelSize: 12
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    height: 32
                    color: "#4E5865"
                    font.family: harmonyOSSansMedium.name
                }
            }
            IpInput{
                id: dnsalter
                height: 32
                Layout.leftMargin: 8
                ip: box.dns_alter
                onIpChanged: {
                    box.dns_alter = ip
                }
            }
        }
    }
}
