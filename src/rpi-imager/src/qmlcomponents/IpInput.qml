import QtQuick 2.12
import QtQuick.Controls 2.12

Rectangle {
    id: control
    width: 221
    height: 32
    color: "#ffffff"
    border.color: "#C3CEDF"
    radius: 2
    property int _itemWidth: (width-3*style.width)/4
    property string ip: ""
    property bool enabled: false
    property bool showTip: false
    Text{
        id: style
        visible: false
        text: "·"
        font{
            pixelSize: 12
            family: harmonyOSSansMedium.name
        }
        color: "#4E5865"
    }
    Row {
        spacing: 0
        anchors.verticalCenter: parent.verticalCenter
        Repeater{
            id: repeater
            model: 7
            Component.onCompleted: {
                let ipList = control.ip.split(".")
                if(ipList.length === 4){
                    repeater.itemAt(0).text=ipList[0];
                    repeater.itemAt(2).text=ipList[1];
                    repeater.itemAt(4).text=ipList[2];
                    repeater.itemAt(6).text=ipList[3];
                    control.showTip = true
                }
            }
            IpItem {
                //height: control.height
                width: (index%2==1)?style.width:control._itemWidth
                enabled: (index%2==0)
                text: (index%2==1)?style.text:""
                font: style.font
                color: (index%2==1)?"#BFC7D7":style.color
                //到上一段
                onGotoPrev: {
                    if(index%2==0 && index-2>=0){
                        let item=repeater.itemAt(index-2);
                        item.forceActiveFocus();
                        item.cursorPosition=cursorLeft?0:item.text.length;
                        if(checkAll)
                            item.selectAll();
                    }
                }
                //到下一段
                onGotoNext: {
                    if(index%2==0 && index+2<7){
                        let item=repeater.itemAt(index+2);
                        item.forceActiveFocus();
                        item.cursorPosition=cursorLeft?0:item.text.length;
                        if(checkAll)
                            item.selectAll();
                    }
                }
                onDoTextChange: {
                    let ip_list = []
                    if(repeater.itemAt(0)&&repeater.itemAt(0).text){
                        ip_list.push(repeater.itemAt(0).text)
                    }
                    if(repeater.itemAt(2)&&repeater.itemAt(2).text){
                        ip_list.push(repeater.itemAt(2).text)
                    }
                    if(repeater.itemAt(4)&&repeater.itemAt(4).text){
                        ip_list.push(repeater.itemAt(4).text)
                    }
                    if(repeater.itemAt(6)&&repeater.itemAt(6).text){
                        ip_list.push(repeater.itemAt(6).text)
                    }
                    if(ip_list.length === 4){
                        control.ip = ip_list.join(".")
                    } else {
                        if(control.showTip){
                            tip.show("IP格式错误",1000)
                        }
                    }
                }
            }
        }
    }
    onEnabledChanged: {
        if(enabled){
            control.color = "#EBEDF3"
        } else {
            control.color = "#ffffff"
        }
    }
    ToolTip{
        id: tip
    }
    function setDefaultIp(ipInfo){
        let ipList = ipInfo.split(".")
        control.showTip = false
        if(ipList.length === 4){
            repeater.itemAt(0).text=ipList[0];
            repeater.itemAt(2).text=ipList[1];
            repeater.itemAt(4).text=ipList[2];
            repeater.itemAt(6).text=ipList[3];
        } else {
            repeater.itemAt(0).text="";
            repeater.itemAt(2).text="";
            repeater.itemAt(4).text="";
            repeater.itemAt(6).text="";
        }
        control.showTip = true
    }
}
