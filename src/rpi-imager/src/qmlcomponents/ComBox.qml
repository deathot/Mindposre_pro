
import QtQuick 2.0
import QtQuick 2.6
import QtQuick.Controls 2.2



Item {


    id: comboBoxComponentItem

    property alias currentText: cur.text
    property alias currentIndex: control.currentIndex
    property alias model: control.model
    property var   bindRadio: null
    property alias delegate: control.delegate
    property string   color: "#4E5865"

    property var   borderWidth: comboBoxComponentItem.bindRadio == null?1:
                                                                   (comboBoxComponentItem.bindRadio.checked?2:1)
    property int  leftMargin: 20
    property int  comboBoxWidth: 150
    signal click()

    ComboBox {
        id: control
        anchors.leftMargin:comboBoxComponentItem.leftMargin
        // 源数据
        model:[]
        textRole: "description"
        // 下拉框的图标
        indicator: Canvas {
            id: canvas
            x: control.width - width - control.rightPadding
            y: control.topPadding + (control.availableHeight - height) / 2
            width: 8
            height: 4
            contextType: "2d"

            Connections {
                target: control
                function onPressedChanged() {
                    canvas.requestPaint()
                }
            }

            onPaint: {
                context.reset();
                context.moveTo(0, 0);
                context.lineTo(width, 0);
                context.lineTo(width / 2, height);
                context.closePath();
                context.fillStyle =  comboBoxComponentItem.color ;
                context.fill();
            }
        }
        contentItem: Text {
            id: cur
            //@disable-check M16
            leftPadding: 10
            // @disable-check M16
            rightPadding: control.indicator.width + control.spacing
            text: ""
            color:  "#4E5865"
            horizontalAlignment: Text.AlignLeft
            verticalAlignment: Text.AlignVCenter
            elide: Text.ElideRight
            font.pixelSize: 11
            font.family: harmonyOSSansSCRegular.name
        }
        background: Rectangle {
            implicitWidth: comboBoxComponentItem.comboBoxWidth
            implicitHeight: 32
            color:"transparent"
            border.color: "#C3CEDF"
            border.width: 1
            radius: 2
            MouseArea {
                anchors.fill: parent
                onClicked: {
                    pop.open()
                    click()
                }
            }
        }
        popup: Popup {    //弹出项
            id: pop
            y: control.height
            width: control.width
            implicitHeight: contentItem.implicitHeight
            padding: 1
            //listView具有一个模型和一个委托。模型model定义了要显示的数据
            contentItem: ListView {   //显示通过ListModel创建的模型中的数据
                id: comboxview
                clip: true
                implicitHeight: contentHeight
                model: control.delegateModel
                delegate: control.delegate
                onCountChanged: {
                    if(count>0 && (sdinfo.text === "Please insert SD card" || sdinfo.text === "请插入SD卡")){
                        comboxview.currentIndex = 0
                        selectDstItem(currentItem)
                        sdbox.state = "hassd"
                    } else if (count == 0) {
                        restsdcard()
                    }
                }
            }
            background: Rectangle {
                border.color: comboBoxComponentItem.color
                radius: 2
            }
        }

    }
    function closePop(){
        pop.visible = false
    }
}



