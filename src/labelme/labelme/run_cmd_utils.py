import os
import subprocess


def cmd_run(bat_path, process_value, ret_value):
    cur_path = os.getcwd()
    os.chdir(os.path.dirname(bat_path))
    bat_path = f'"{str(bat_path)}"'
    cmd_str = "cmd.exe /c " + bat_path
    print(cmd_str)
    p = subprocess.Popen(cmd_str, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    os.chdir(cur_path)
    curline = p.stdout.readline()
    while (curline != b''):
        if b'\x1b[0m' in curline or b'\xce\xc4\xbc\xfe\xc3\xfb\xa1\xa2\xc4\xbf\xc2\xbc\xc3\xfb\xbb\xf2\xbe\xed\xb1\xea\xd3\xef\xb7\xa8\xb2\xbb\xd5\xfd\xc8\xb7\xa1\xa3' in curline:
            curline = p.stdout.readline()
            continue
        curline = curline.decode('utf-8', 'ignore').strip()
        print(curline)
        if 'process_value' in curline:
            process_value.emit(int(curline.split("=")[-1]))
        elif 'ERROR' in curline:
            ret_value.emit(str(curline))
            return 1
        else:
            ret_value.emit(str(curline))
        curline = p.stdout.readline()
    p.wait()
    return 0


def is_contains_chinese(path):
    for i in path:
        if '\u4e00' <= i <= '\u9fa5':
            return True
    return False


def check_path(path):
    path = path.split(":")[1]
    invalid_str = "~!@#$%^&+,.、[]！￥……（）——{}：' “，。《》？【】`· "
    for p in path:
        if p in invalid_str:
            return p
    return ""
