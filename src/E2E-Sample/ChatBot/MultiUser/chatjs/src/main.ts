import { createApp } from 'vue'
import './style.css'
import App from './App.vue'
import ElementPlus from 'element-plus'
import 'element-plus/dist/index.css'
import {
    Microphone
} from '@element-plus/icons-vue'
const app = createApp(App)
app.use(ElementPlus)
app.component("Microphone",Microphone)
app.mount('#app')

