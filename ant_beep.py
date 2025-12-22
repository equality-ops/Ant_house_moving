from machine import *
import time

# 蜂鸣器初始化
beep    = Pin('D24', Pin.OUT, value = False)

class BeepState:
    beep_state = 0
    BEEP_OFF = 0
    BEEP_ON = 1

# 蜂鸣器警告函数(响3声，每500ms响一声，每次持续50ms)
def beep_warn() -> None:
    if BeepState.beep_state == BeepState.BEEP_OFF:
        BeepState.beep_state = BeepState.BEEP_ON
        for i in range(3):
            time.sleep_ms(50)
            beep.high()
            time.sleep_ms(50)
            beep.low()
            time.sleep_ms(400)
        BeepState.beep_state = BeepState.BEEP_OFF
        return 
    elif BeepState.beep_state == BeepState.BEEP_ON:
        return 




