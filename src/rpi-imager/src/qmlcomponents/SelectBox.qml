
import QtQuick 2.0
import QtQuick 2.6
import QtQuick.Controls 2.1



Item {


    id:comboBoxComponentItem

    property alias label: labelText.text
    property alias currentText: control.currentText
    property alias currentIndex: control.currentIndex
    property alias model: control.model
    property var   bindRadio: null
    property var   color: comboBoxComponentItem.bindRadio == null?"#696969":
                                                                   (comboBoxComponentItem.bindRadio.checked?"#6495ED":"#696969")

    property var   borderWidth: comboBoxComponentItem.bindRadio == null?1:
                                                                   (comboBoxComponentItem.bindRadio.checked?2:1)

    property int leftMargin: 20
    property int  comboBoxWidth: 150
    property int labelWidth: 60


    Text
    {
        id: labelText
        width: comboBoxComponentItem.labelWidth
        height: 30
        font.family: "微软雅黑"
        font.pointSize: 15
        color: comboBoxComponentItem.color
        horizontalAlignment: Text.AlignLeft
        verticalAlignment: Text.AlignVCenter

    }


    ComboBox {

        id: control

        anchors.left:labelText.right
        anchors.leftMargin:comboBoxComponentItem.leftMargin

        model:[]
        delegate: ItemDelegate
        {
            width: control.width
            contentItem: Text {
                text: modelData
                color:  comboBoxComponentItem.color;
                elide: Text.ElideRight
                verticalAlignment: Text.AlignVCenter
                font.family: "微软雅黑"
                font.pointSize: 15
            }
            highlighted: control.highlightedIndex == index
        }

        indicator: Canvas {
            id: canvas
            x: control.width - width - control.rightPadding
            y: control.topPadding + (control.availableHeight - height) / 2
            width: 12
            height: 8
            contextType: "2d"

            Connections {
                target: control
                onPressedChanged: canvas.requestPaint()
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
            //@disable-check M16
            leftPadding: 10
            //@disable-check M16
            rightPadding: control.indicator.width + control.spacing
            text: control.currentText
            color:  comboBoxComponentItem.color
            horizontalAlignment: Text.AlignLeft
            verticalAlignment: Text.AlignVCenter
            elide: Text.ElideRight
            font.family: "微软雅黑"
            font.pixelSize: 15
        }

        background: Rectangle {
            implicitWidth: comboBoxComponentItem.comboBoxWidth
            implicitHeight: 30
           // border.color: control.pressed ? "#6495ED" : "#696969"
           // border.width: control.visualFocus ? 2 : 1
            color:"transparent"
            border.color: comboBoxComponentItem.color
            border.width: 1
            radius: 6
        }

    }
}


