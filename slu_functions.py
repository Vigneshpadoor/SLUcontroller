import serial
ser = serial.Serial('/dev/ttyUSB0')  # open serial port


def set_position_absolutely(abs_pos):
    data = f'spa,1,{abs_pos}\n'
    ser.write(data)
    # 書式(text) ：spa,[axis],[value1](delim)
    # Return ：0(ret)


def get_position():
    data = f'gp,1\n'
    ser.write(data)
    # 書式(text) ：gp,[axis](delim)
    #  Return ：1000(ret)
    # 解説：軸の現在位置を問い合わせる

def set_position_relatively(spr):
    data = f'spr,1,{spr}\n'
    ser.write(data)
    # 書式(text) ：spr,[axis],[value1](delim)
    # Return ：0(ret)
    # 解説：軸を現在位置からの相対位置で指定する位置に移動させる。


def set_velocity_absolutely(sva):
    data = f'sva,1,{sva}\n'
    ser.write(data)
    # 書式(text) ：sva,[axis],[value2](delim)
    # Return ：0(ret)
    # 解説：位置制御モード時：軸の移動速度を設定する。（正の整数）
    # 速度制御モード時：軸を指定速度で移動する。（正負の整数）


def get_velocity():
    data = f'gv,1\n'
    ser.write(data)
    # 書式(text) ：gv,[axis](delim)
    # Return ：1000(ret)
    # 解説：軸の現在速度を問い合わせる。戻り値は steps/sec


def set_velocity_relatively(svr):
    data = f'svr,1,{svr}\n'
    ser.write(data)
    # 書式(text) ：svr,[axis],[value2](delim)
    # Return ：0(ret)
    # 解説：位置制御モード時：軸の移動速度を現在速度との相対速度で設定する。
    # 速度制御モード時：軸を現在速度との相対速度で指定した速度で移動する。

def stop_all() :
    data = f'sta,1\n'
    ser.write(data)
    # 書式(text) ：sta,[axis](delim)
    # Return ：0(ret)
    # 解説：全ての回転軸を停止する。

def set_acceleration_absolutely(saa):
    data = f'saa,1,{saa}\n'
    ser.write(data)
    # 書式(text) ：saa,[axis],[value1](delim)
    # Return ：0(ret)
    # 解説：軸の加速度を設定する。単位は steps/sec2


def get_acceleration():
    data = f'ga,1\n'
    ser.write(data)
    # 書式(text) ：ga,[axis](delim)
    # Return ：1000(ret)
    # 解説：転軸の設定加速度を問い合わせる、戻り値は steps/sec2


def set_lower_velocity_absolutely(slva):
    data = f'slva,1,{slva}\n'
    ser.write(data)
    # 書式(text) ：slva,[axis],[value1](delim)
    # Return ：0(ret)
    # 解説：軸の最低速度を設定する。単位は steps/sec


def get_lower_velocity():
    data = f'glv,1\n'
    ser.write(data)
    # 書式(text) ：glv,[axis](delim)
    # Return ：1000(ret)
    # 解説：軸の最低速度を問い合わせる、戻り値は steps/sec


def save():
    data = f'save,1\n'
    ser.write(data)
    # 書式(text) ：save,[axis](delim)
    # Return ：0(ret)
    # 解説：パラメータを内蔵フラッシュメモリにセーブする。


def initialize():
    data = f'init,1\n'
    ser.write(data)
    # 書式(text) ：init,[axis](delim)
    # Return ：0(ret)
    # 解説：軸のイニシャライズを行う。原点を設定する。


def set_drive_mode(sdm):
    data = f'sdm,1,{sdm}\n'
    ser.write(data)
    # 書式(text) ：sdm,[axis],[value3](delim) 
    # Return ：0(ret)
    # 解説：モーター制御モードをセットする。
    # ０ 位置制御モード
    # １ 速度制御モード

def get_drive_mode():
    data = f'gdm,1\n'
    ser.write(data)
    # 書式(text) ：gdm,[axis](delim)
    # Return ：1(ret)
    # 解説：現在のモーター制御モードを問い合わせる
    # ０ 位置制御モード
    # １ 速度制御モード

def version():
    data = f'ver,1\n'
    ser.write(data)
    # 書式(text) ：ver,[axis](delim)
    # Return ：103(ret)
    # 解説：ファームウェアのバージョンを問い合わせます。


def axis():
    data = f'axis,1\n'
    ser.write(data)
    # 書式(text) ：axis,[axis](delim) 
    # Return ：1(ret)
    # 解説：軸の数を問い合わせます。Linear Unit では常に 1 が返されます。


def load():
    data = f'load,1\n'
    ser.write(data)
    # 書式(text) ：load,[axis],[value3](delim)
    # Return ：0(ret)
    # 解説：パラメータに値を設定する。設定元データを指定できる。
    # ０ 内蔵フラッシュメモリに保存されている値を設定する。
    # １ 工場出荷状態の値を設定する。


def get_target_velocity():
    data = f'gtv,1\n'
    ser.write(data)
    # 書式(text) ：gtv,[axis](delim)
    # Return ：1000(ret)
    # 解説：設定してある速度を問い合わせる、戻り値は steps/sec


def set_baud_rate(sbr):
    data = f'sbr,1,{sbr}\n'
    ser.write(data)
    # 書式(text) ：sbr,[value4](delim) 
    # Return ：9600(ret)
    # 解説：通信速度を設定する。
    # 設定可能な値は(9600,19200,38400,115200)
    # 正常な場合、リターンコードを返した後、2 秒後にボーレートを変更します。
    # ボーレートを変更した後，save コマンドを使用すると，電源再投入時に設定
    # ボーレートに再び設定されます。

def wait():
    data = f'wait,1\n'
    ser.write(data)
    # 書式(text) ：wait,[axis](delim)
    # Return ：0(ret)
    # 解説：直前に実行された位置指令動作(spa,spr,sop)が終了を待つ。

def remove_wait():
    data = f'rwait,1\n'
    ser.write(data)
    # 書式(text) ：rwait,[axis](delim)
    # Return ：0(ret)
    # 解説：現在有効な wait がある場合、それを終了する。


def get_disired_position():
    data = f'gdp,1\n'
    ser.write(data)
    # 書式(text) ：gdp, [axis] (delim)
    # Return ：10(ret)
    # 解説：位置指令動作で，目的位置を問い合わせる。

def get_driver_status():
    data = f'gds,1\n'
    ser.write(data)
    # 書式(text) ：gdp, [axis] (delim)
    # Return ：9(ret)
    # 解説：ドライバーのステータスビットを問いあわせる。
    # Bit0 : ALARM( 0:Active 1:Nonactive )
    # Bit1 : Driver Temp( 0:Normal 1:High )
    # Bit2 : Motor Temp( 0:Normal 1:High )
     #Bit3 : Moving( 0:Stop 1:Move )