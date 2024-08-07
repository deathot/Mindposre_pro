import smbus2
import time

'''
以下为配置相关的函数和变量，程序执行在最后
'''
# 语音识别模块地址
i2c_addr = 0x30
date_head = 0xfd

def I2C_WriteBytes(str_, bus):
    global i2c_addr
    for ch in str_:
        try:
            bus.write_byte(i2c_addr, ch)
            time.sleep(0.01)
        except:
            # print("write I2C error")
            pass


EncodingFormat_Type = {
	                  'GB2312':0x00,
	                  'GBK':0X01,
	                  'BIG5':0x02,
	                  'UNICODE':0x03
	                  }

def Speech_text(str_, encoding_format, bus):
    str_ = str_.encode('gb2312')   
    size = len(str_) + 2
    DataHead = date_head
    Length_HH = size>>8
    Length_LL = size & 0x00ff
    Commond = 0x01
    EncodingFormat = encoding_format
    Date_Pack = [DataHead, Length_HH, Length_LL, Commond, EncodingFormat]
    I2C_WriteBytes(Date_Pack, bus)
    I2C_WriteBytes(str_, bus)

def SetBase(str_, bus):
    str_ = str_.encode('gb2312')   
    size = len(str_) + 2
    DataHead = date_head
    Length_HH = size>>8
    Length_LL = size & 0x00ff
    Commond = 0x01
    EncodingFormat = 0x00
    Date_Pack = [DataHead, Length_HH, Length_LL, Commond, EncodingFormat]
    I2C_WriteBytes(Date_Pack, bus)
    I2C_WriteBytes(str_, bus)

def TextCtrl(ch,num, bus):
    if num != -1:
        str_T = '[' + ch + str(num) + ']'
        SetBase(str_T, bus)
    else:
        str_T = '[' + ch + ']'
        SetBase(str_T, bus)

ChipStatus_Type = {
                  'ChipStatus_InitSuccessful':0x4A,  # 初始化成功回传
                  'ChipStatus_CorrectCommand':0x41,  # 收到正确的命令帧回传
                  'ChipStatus_ErrorCommand':0x45,    # 收到不能识别命令帧回传
                  'ChipStatus_Busy':0x4E,            # 芯片忙碌状态回传
                  'ChipStatus_Idle':0x4F             # 芯片空闲状态回传                  
                  }

def GetChipStatus(bus):
    global i2c_addr
    AskState = [0xfd, 0x00, 0x01, 0x21]
    try:
        I2C_WriteBytes(AskState, bus)
        time.sleep(0.05)
    except:
        # print("I2CRead_Write error")
        pass
    try:
        Read_result = bus.read_byte(i2c_addr)
        return Read_result
    except:
        # print("I2CRead error")
        pass

Style_Type = {
             'Style_Single':0,   # 为0，一字一顿的风格
             'Style_Continue':1  # 为1，正常合成
             }                   # 合成风格设置[f?]

def SetStyle(num, bus):
    TextCtrl('f',num, bus)
    while GetChipStatus(bus) != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)   

Language_Type = {
                'Language_Auto':0,     # 为0，自动判断语种
                'Language_Chinese':1,  # 为1，阿拉伯数字、度量单位、特殊符号等合成为中文
                'Language_English':2   # 为1，阿拉伯数字、度量单位、特殊符号等合成为中文
                }                      # 合成语种设置[g?]

def SetLanguage(num, bus):
	TextCtrl('g',num, bus)
	while GetChipStatus(bus) != ChipStatus_Type['ChipStatus_Idle']:
		time.sleep(0.002)

Articulation_Type = {
                    'Articulation_Auto':0,    # 为0，自动判断单词发音方式
                    'Articulation_Letter':1,  # 为1，字母发音方式
                    'Articulation_Word':2     # 为2，单词发音方式
                    }                         # 设置单词的发音方式[h?]

def SetArticulation(num, bus):
	TextCtrl('h',num, bus)
	while GetChipStatus(bus) != ChipStatus_Type['ChipStatus_Idle']:
		time.sleep(0.002)

Spell_Type = {
             'Spell_Disable':0,  # 为0，不识别汉语拼音
             'Spell_Enable':1    # 为1，将“拼音＋1 位数字（声调）”识别为汉语拼音，例如：hao3
             }                   # 设置对汉语拼音的识别[i?]

def SetSpell(num, bus):
	TextCtrl('i', num, bus)
	while GetChipStatus(bus) != ChipStatus_Type['ChipStatus_Idle']:
		time.sleep(0.002)

Reader_Type = {
              'Reader_XiaoYan':3,      # 为3，设置发音人为小燕(女声, 推荐发音人)
              'Reader_XuJiu':51,       # 为51，设置发音人为许久(男声, 推荐发音人)
              'Reader_XuDuo':52,       # 为52，设置发音人为许多(男声)
              'Reader_XiaoPing':53,    # 为53，设置发音人为小萍(女声
              'Reader_DonaldDuck':54,  # 为54，设置发音人为唐老鸭(效果器)
              'Reader_XuXiaoBao':55    # 为55，设置发音人为许小宝(女童声)                
              }                        # 选择发音人[m?]

def SetReader(num, bus):
	TextCtrl('m', num, bus)
	while GetChipStatus(bus) != ChipStatus_Type['ChipStatus_Idle']:
		time.sleep(0.002)

NumberHandle_Type = {
                    'NumberHandle_Auto':0,    # 为0，自动判断
                    'NumberHandle_Number':1,  # 为1，数字作号码处理
                    'NumberHandle_Value':2    # 为2，数字作数值处理
                    }                         # 设置数字处理策略[n?]

def SetNumberHandle(num, bus):
	TextCtrl('n', num, bus)
	while GetChipStatus(bus) != ChipStatus_Type['ChipStatus_Idle']:
		time.sleep(0.002)

ZeroPronunciation_Type = {
                         'ZeroPronunciation_Zero':0,  # 为0，读成“zero
                         'ZeroPronunciation_O':1      # 为1，读成“欧”音
                         }                            # 数字“0”在读 作英文、号码时的读法[o?]

def SetZeroPronunciation(num, bus):
	TextCtrl('o', num, bus)
	while GetChipStatus(bus) != ChipStatus_Type['ChipStatus_Idle']:
		time.sleep(0.002)

NamePronunciation_Type = {
                         'NamePronunciation_Auto':0,       # 为0，自动判断姓氏读音
                         'NamePronunciation_Constraint':1  # 为1，强制使用姓氏读音规则
                         }                                 # 设置姓名读音策略[r?]


def SetNamePronunciation(num, bus):
	TextCtrl('r', num, bus)
	while GetChipStatus(bus) != ChipStatus_Type['ChipStatus_Idle']:
		time.sleep(0.002)

# 设置语速[s?] ?为语速值，取值：0～10
def SetSpeed(speed, bus):
	TextCtrl('s', speed, bus)
	while GetChipStatus(bus) != ChipStatus_Type['ChipStatus_Idle']:
		time.sleep(0.002)

# 设置语调[t?] ?为语调值，取值：0～10
def SetIntonation(intonation, bus):
	TextCtrl('t', intonation, bus)
	while GetChipStatus(bus) != ChipStatus_Type['ChipStatus_Idle']:
		time.sleep(0.002)

# 设置音量[v?] ?为音量值，取值：0～10
def SetVolume(volume, bus):
	TextCtrl('v', volume, bus)
	while GetChipStatus(bus) != ChipStatus_Type['ChipStatus_Idle']:
		time.sleep(0.002)

OnePronunciation_Type = {
                        'OnePronunciation_Yao':0,  # 为0，合成号码“1”时读成幺
                        'OnePronunciation_Yi':1    # 为1，合成号码“1”时读成一
                        }                          # 设置号码中“1”的读法[y?]

def SetOnePronunciation(num, bus):
	TextCtrl('y', num, bus)
	while GetChipStatus(bus) != ChipStatus_Type['ChipStatus_Idle']:
		time.sleep(0.002)

Rhythm_Type = {
              'Rhythm_Diasble':0,  # 为0，“*”和“#”读出符号
              'Rhythm_Enable':1    # 为1，处理成韵律，“*”用于断词，“#”用于停顿
              }                    # 是否使用韵律标记“*”和“#” [z?]

def SetRhythm(num, bus):
	TextCtrl('z', num, bus)
	while GetChipStatus(bus) != ChipStatus_Type['ChipStatus_Idle']:
		time.sleep(0.002)

# 恢复默认的合成参数 [d] 所有设置（除发音人设置、语种设置外）恢复为默认值
def SetRestoreDefault(bus):
	TextCtrl('d', -1, bus)
	while GetChipStatus(bus) != ChipStatus_Type['ChipStatus_Idle']:
		time.sleep(0.002)

'''
程序执行部分，以上为配置相关的函数和字典定义
'''
def init_speaker(bus):
    # 选择播音人晓萍
    SetReader(Reader_Type["Reader_XuDuo"], bus)
    SetVolume(7, bus)


def say_sth(text, bus):
    Speech_text(text, EncodingFormat_Type["GB2312"], bus)

    # 等待当前语句播报结束
    while GetChipStatus(bus) != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.1)  
