import cv2
import os
import configparser

config = configparser.ConfigParser()
cur_dir_path = os.path.dirname(os.path.realpath(__file__))
cfg_dir_path = os.path.join(cur_dir_path, "config")
cfg_file_path = os.path.join(cfg_dir_path, 'hsv_file_setting.ini')
config.read(cfg_file_path)
image_test_path = config['DEFAULT']['imagepath']
_SHOW_IMAGE = True
h_list = []
s_list = []
v_list = []

def show_image(title, frame, show=_SHOW_IMAGE):
    if show:
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        cv2.imshow(title, frame)
        cv2.setMouseCallback('image', click_event)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
def click_event(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        font = cv2.FONT_HERSHEY_SIMPLEX
        h = hsv[y, x, 0]
        h_list.append(h)
        s = hsv[y, x, 1]
        s_list.append(s)
        v = hsv[y, x, 2]
        v_list.append(v)
        print(h, ' ', s, ' ', v)
        cv2.putText(hsv, str(h) + ',' + str(s) + ',' + str(v), (x,y), font, 1, (255,0,0), 2)
        cv2.imshow('image', hsv)
    
frame = cv2.imread(image_test_path) 
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
cv2.imshow("image", hsv)
cv2.setMouseCallback("image", click_event)
cv2.waitKey(0)
cv2.destroyAllWindows()

h_max = max(h_list)
h_min = min(h_list)
print("h_list is:", h_list)
print("h_max is:", h_max)
print("h_min is:", h_min)

s_max = max(s_list)
s_min = min(s_list)
print("s_list is:", s_list)
print("s_max is:", s_max)
print("s_min is:", s_min)

v_max = max(v_list)
v_min = min(v_list)
print("v_list is:", v_list)
print("v_max is:", v_max)
print("v_min is:", v_min)

tmp_folder_name = "hsv-data"
folder_exist = True
if not os.path.exists(tmp_folder_name):
    folder_exist = False
    
os.makedirs(tmp_folder_name, exist_ok=True)
tmp_file_path = os.path.join(tmp_folder_name, "hsv.txt")

if folder_exist:
    with open(tmp_file_path, 'r') as f:
        content = f.read()
        file_h_min, file_h_max, file_s_min, file_s_max, file_v_min, file_v_max = \
            [int(elm) for elm in content.split('\n')]
        h_min = min(file_h_min, h_min)
        h_max = max(file_h_max, h_max)
        s_min = min(file_s_min, s_min)
        s_max = max(file_s_max, s_max)
        v_min = min(file_v_min, v_min)
        v_max = max(file_v_max, v_max)
        
with open(tmp_file_path, 'w') as f:
    f.write(str(h_min) + '\n')
    f.write(str(h_max) + '\n')
    f.write(str(s_min) + '\n')
    f.write(str(s_max) + '\n')
    f.write(str(v_min) + '\n')
    f.write(str(v_max))
        
    
    