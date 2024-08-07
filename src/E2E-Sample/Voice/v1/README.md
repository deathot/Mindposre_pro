# 台灯语音样例代码使用指南
1. 执行`git clone --branch 2.5.4 git@github.com:wzpan/wukong-robot.git`或`git clone --branch 2.5.4 https://github.com/wzpan/wukong-robot.git`克隆`wukong-robot`仓库到本地；
2. 将`asr.py`和`speaker.py`放入`wukong-robot/snowboy`目录；
3. 将两个补丁文件`Conversation.patch`和`snowboydecoder.patch`放到克隆仓库的根目录；
4. 使用`git apply <patch_file_path>`命令分别处理两个补丁文件（执行git apply Conversation.patch和git apply snowboydecoder.patch），检查`wukong-robot/robot/Conversation.py`
和`wukong-robot/snowboy/snowboydecoder.py`两个文件内容是否已发生改变。