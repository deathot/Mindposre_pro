<script lang="ts">
import {
    ref,
    nextTick
} from 'vue'
import {
    ElMessage
} from 'element-plus'
import {
    onMounted,
    onUnmounted
} from 'vue';

import avtor from "../assets/avtor.jpg"
import me from "../assets/me.jpg"

const chatBox = ref < any > (null)
const originMessage = ref < string > ("")
const msgList = ref < Array < any >> ([])
const sendDisable = ref < boolean > (false)
const randomId = ref < string > ("")
let timer = ref < any > ("")
let isSend = ref < Boolean > (false)

let param = ref < any > (null)
param.value = {
    sampleBits: 8,
    numChannels: 1
}

// 生成随机ID的函数
function generateId() {
    let id = ""
    const characters = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789'
    for (let i = 0; i < 50; i++) {
        id += characters.charAt(Math.floor(Math.random() * characters.length))
    }
    randomId.value = id
}

generateId()

function getMsg() {
    const url = `/getMsg?id=${randomId.value}`
    fetch(url)
        .then(res => res.json())
        .then(res => {
            if (res.code === 200) {
                if (isSend.value) {
                    msgList.value.push({
                        isme: false,
                        avator: avtor,
                        content: res.data.message
                    })
                } else {
                    msgList.value[msgList.value.length - 1].content += res.data.message
                }
                if (res.data.isEnd) {
                    clearInterval(timer.value)
                    sendDisable.value = false
                }
                isSend.value = false
                scrollBottom()
            } else {
                ElMessage({
                    type: 'warning',
                    message: res.message,
                })
                sendDisable.value = false
                isSend.value = false
                clearInterval(timer.value)
            }
        })
        .catch(err => {
            ElMessage({
                message: err,
                type: 'warning',
            })
            sendDisable.value = false
            isSend.value = false
            clearInterval(timer.value)
        })
        .finally(() => {
            chatBox.value.focus()
        })
}

//发送给客户
function chatToUser() {
    if (sendDisable.value === true) return
    if (originMessage.value) {
        msgList.value.push({
            isme: true,
            content: originMessage.value,
            avator: me
        })
        sendDisable.value = true
        isSend.value = true
        const url = `/chat?message=${originMessage.value}&id=${randomId.value}`
        fetch(url)
            .then(res => res.json())
            .then(res => {
                if (res.code === 200) {
                    // 循环请求结果
                    timer.value = setInterval(() => {
                        getMsg()
                    }, 100)
                } else {
                    ElMessage({
                        message: res.message,
                        type: 'warning',
                    })
                    sendDisable.value = false
                    isSend.value = false
                }
            })
            .catch(err => {
                ElMessage({
                    message: err,
                    type: 'warning',
                })
                sendDisable.value = false
                isSend.value = false
            })

        originMessage.value = ""
    }
}
//滚动到底部
function scrollBottom() {
    nextTick(() => {
        chatBox.value.scrollTop = 999999999
    });
}

function sendCloseTime() {
    const closeTime = Date.now()
    const url = `/close?time=${closeTime}&id=${randomId.value}`
    fetch(url)
}

// 在页面关闭时调用sendCloseTime函数
onMounted(() => {
    window.addEventListener('beforeunload', handleBeforeUnload);
});

onUnmounted(() => {
    window.removeEventListener('beforeunload', handleBeforeUnload);
});

function handleBeforeUnload(event: BeforeUnloadEvent) {
    event.preventDefault();
    sendCloseTime();
    event.returnValue = '';
}
</script>

<template>
<div class="chatAppBody">
    <div class="chatBox" ref="chatBox">
        <div v-for="row in msgList" style="margin-top: 10px;">
            <div v-if="row.isme!=true">
                <div class="chatNotice">{{row.time}}</div>
                <div class="chatRow">
                    <el-avatar class="chatAvatar" :size="30" :src="row.avator"></el-avatar>
                    <div class="chatMsgContent">
                        <div class="chatContent" v-html="row.content"></div>
                    </div>
                </div>
            </div>
            <div v-if="row.isme==true">
                <div class="chatNotice" v-if="row.show_time==true">{{row.time}}</div>
                <div class="chatRow chatRowMe">
                    <div class="chatContent" v-html="row.content" style="background-color: aquamarine;"></div>
                    <el-avatar class="chatAvatarMe" :size="30" :src="row.avator"></el-avatar>
                </div>
            </div>
        </div>
    </div>
    <div class="chatBottom">
        <div class="chatAreaBox">
            <div class="chatArea">
                <textarea class="chatAreaInput" :disabled="sendDisable" v-model.tirm="originMessage" @keyup.ctrl.enter.exact="chatToUser()" @keyup.enter.exact="chatToUser()"></textarea>

            </div>
        </div>
    </div>
</div>
</template>

<style scoped>
 .chatAppBody {
     display: flex;
     flex-direction: column;
     height: 100vh;
     width: 800px;
     margin: 0 auto;
     border: 1px solid #ccc;
 }

 .chatTitle {
     background: #fff;
     padding: 5px 0px;
     text-align: center;
     font-size: 14px;
 }

 .chatBox {
     flex: 1;
     padding: 0px 5px;
     padding-bottom: 15px;
     overflow: auto;
     overflow: -moz-scrollbars-none;
 }

 .chatBox::-webkit-scrollbar {
     width: 0 !important
 }

 .chatBottom {
     display: flex;
     flex-direction: column;
 }

 .chatRow {
     display: flex;
     align-items: flex-end;
     margin: 5px 0px;
     align-items: start;
 }

 .chatAvatar {
     margin-top: 10px;
     margin-right: 5px;
     flex-shrink: 0;
 }

 .chatUsername {
     font-size: 12px;
     white-space: nowrap;
     color: #999;
     margin-bottom: 2px;
 }

 .chatContent {
     border-radius: 10px 10px 10px 0px;
     padding: 10px;
     background-color: rgb(255, 255, 255);
     box-shadow: 0 5px 30px rgb(50 50 93 / 8%), 0 1px 3px rgb(0 0 0 / 5%);
     font-size: 14px;
     word-break: break-all;
     line-height: 21px;
     display: inline-block;
     max-width: 600px;
 }

 .chatRowMe {
     justify-content: flex-end;
 }

 .chatRowMe .chatContent {
     border-radius: 10px 10px 0px 10px;
 }

 .chatAvatarMe {
     justify-content: flex-end;
     margin-left: 10px;
 }

 .chatNotice {
     text-align: center;
     color: #bbb;
     margin: 8px 0;
     font-size: 12px;
 }

 .chatAreaBox {
     margin: 0px 10px;
     margin-bottom: 10px;
     box-shadow: 0 5px 30px rgba(11, 11, 102, 0.08), 0 1px 3px rgba(145, 108, 28, 0.05);
     border-radius: 10px;
     border: 1PX solid #ccc;
 }

 .chatArea {
     display: flex;
     padding: 3px 5px;
     align-items: center;
     background: #fff;
     border-radius: 10px;
 }

 .chatArea .iconfont {
     color: #383838;
     font-size: 18px;
     margin: 0px 6px;
     cursor: pointer;
 }

 .chatArea .iconfont:hover {
     color: #409eff;
 }

 .chatAreaInput {
     border-radius: 10px;
     border: 0;
     flex: 1;
     outline: none;
     resize: none;
     box-sizing: border-box;
     color: #505050;
     min-height: 50px;
     font-size: 16px;
 }

 .chatCopyright {
     color: #999a9b;
     font-size: 12px;
     text-align: center;
     margin-bottom: 10px;
     filter: grayscale(1);
     opacity: .9;
     font-family: Inter, -apple-system, system-ui, BlinkMacSystemFont, Segoe UI, Roboto, Helvetica Neue, Tahoma, Arial, sans-serif;
 }

 .chatContent a {
     color: #07a9fe;
     text-decoration: none;
 }

 .alink {
     display: inline-block;
     word-break: break-all;
     color: #07a9fe;
     font-size: 12px;
     cursor: pointer;
 }
</style>
