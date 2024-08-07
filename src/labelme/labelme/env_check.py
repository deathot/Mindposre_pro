import os
from PyQt5.QtCore import QThread, pyqtSignal


class EnvCheckThread(QThread):
    thread_end = pyqtSignal(int)

    def __init__(self):
        super(EnvCheckThread, self).__init__()

    def run(self):
        if os.system('conda --version') != 0:
            ret = -1
        elif os.system('conda activate model-adapter-tool') != 0:
            ret = 0
        else:
            os.system('conda deactivate')
            ret = 1
        self.thread_end.emit(ret)
