import QtQuick 2.12
import QtQuick.Controls 2.12

//每一段的编辑框
TextInput {
    id: control

    //向前移动，如删除时
    //checkAll=true全选，cursorLeft=true光标在左侧
    signal gotoPrev(bool checkAll, bool cursorLeft)
    //向后移动，如已输入三个数字
    signal gotoNext(bool checkAll, bool cursorLeft)
    //向外弹出用户填写信息
    signal doTextChange()
    selectByMouse: true
    validator: RegExpValidator{
        regExp: /^(\d|[1-9]\d|1\d\d|2[0-4]\d|25[0-5])$/
    }

    verticalAlignment: TextInput.AlignVCenter
    horizontalAlignment: TextInput.AlignHCenter

    onTextChanged: {
        if(text.length>=3){
            control.gotoNext(false,true);
        }
        control.doTextChange(text)

    }

    Keys.enabled: true
    Keys.onPressed: {
        event.accepted=false;
        switch(event.key){
        case Qt.Key_Tab:
        case Qt.Key_Period:
             control.gotoNext(true,false);
            break;
        case Qt.Key_Left:
            if(cursorPosition==0){
                control.gotoPrev(false,false);
            }
            break;
        case Qt.Key_Right:
            if(cursorPosition==text.length){
                control.gotoNext(false,true);
            }
            break;
        case Qt.Key_Backspace:
            if(cursorPosition==0 && text.length<=0){
                control.gotoPrev(false,false);
            }
            break;
        }
    }
    Keys.onReleased: {
        event.accepted=false;
        switch(event.key){
        case Qt.Key_C:
        case Qt.Key_V:
            event.accepted=(event.modifiers===Qt.ControlModifier);
            break;
        }
    }
}
