from machine import *
import time
import user_main

class BeepState:
    BEEP_OFF = 0
    BEEP_ON = 1

# 蜂鸣器警告函数(响3声，每500ms响一声，每次持续50ms)
def beep_warn(beep: Pin) -> None:
    if user_main.beep_state == BeepState.BEEP_OFF:
        user_main.beep_state = BeepState.BEEP_ON
        for i in range(3):
            time.sleep_ms(50)
            beep.high()
            time.sleep_ms(50)
            beep.low()
            time.sleep_ms(400)
        user_main.beep_state = BeepState.BEEP_OFF
        return 
    elif user_main.beep_state == BeepState.BEEP_ON:
        return 



