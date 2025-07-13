from machine import Pin, PWM, ADC, time_pulse_us, UART
from time import sleep, sleep_us
import sys
import _thread
import random

led_0 = PWM(Pin(2))
led_1 = PWM(Pin(3))
led_2 = PWM(Pin(7))
led_3 = PWM(Pin(13))


LED = [led_0, led_1, led_2, led_3]

_button = Pin(22, Pin.IN, Pin.PULL_UP)

voltage_26 = ADC(26)

voltage_27 = ADC(27)

buzzer = PWM(Pin(28))

IS_BUTTON_PRESSED = False

motor_A_1 = PWM(Pin(4))
motor_A_1.freq(1000)
motor_A_2 = PWM(Pin(5))
motor_A_2.freq(1000)

motor_E_1 = PWM(Pin(16))
motor_E_1.freq(1000)
motor_E_2 = PWM(Pin(17))
motor_E_2.freq(1000)

SOUND_SPEED=340
TRIG_PULSE_DURATION_US=10

trig_pin_F = Pin(27, Pin.OUT) # white
echo_pin_F = Pin(26, Pin.IN)  # yellow

def init():
    for i in range(4):
        LED[i].duty_u16(0)
    buzzer.duty_u16(0)
    motor_A_1.duty_u16(0)
    motor_A_2.duty_u16(0)
    motor_E_1.duty_u16(0)
    motor_E_2.duty_u16(0)
    print('모든 센서가 초기화 되었습니다')


def led(num, power):
    LED[num].duty_u16(int(65000 * power * 0.01))


def led_off():
    for i in range(4):
        LED[i].duty_u16(0)


def button():
    global IS_BUTTON_PRESSED
    return IS_BUTTON_PRESSED


def speaker(tone, power):
    buzzer.duty_u16(power * 10)
    buzzer.freq(tone)

def get_voltage_26():
    return voltage_26.read_u16()


def get_voltage_27():
    return voltage_27.read_u16()


def set_power(per):
    # return 100-int(65535*per*0.01)
    return int(65535 * per * 0.01)


def motor_A_stop():
    motor_A_1.duty_u16(0)
    motor_A_2.duty_u16(0)


def motor_A_right(power):
    motor_A_1.duty_u16(set_power(power))
    motor_A_2.duty_u16(0)


def motor_A_left(power):
    motor_A_1.duty_u16(0)
    motor_A_2.duty_u16(set_power(power))


def motor_E_stop():
    motor_E_1.duty_u16(0)
    motor_E_2.duty_u16(0)


def motor_E_right(power):
    motor_E_1.duty_u16(set_power(power))
    motor_E_2.duty_u16(0)


def motor_E_left(power):
    motor_E_1.duty_u16(0)
    motor_E_2.duty_u16(set_power(power))

def cm_F():
    trig_pin_F.value(0)
    sleep_us(5)
    trig_pin_F.value(1)
    sleep_us(TRIG_PULSE_DURATION_US)
    trig_pin_F.value(0)
    ultrason_duration = time_pulse_us(echo_pin_F, 1, 30000)
    return SOUND_SPEED * ultrason_duration / 20000

def lidar_B():
    uart = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9))
    buffer = bytearray(9)
    distance = 0
    if uart.any() >= 9:
            byte = uart.read(1)
            if byte == b'\x59':
                next_byte = uart.read(1)
                if next_byte == b'\x59':
                    buffer[0] = 0x59
                    buffer[1] = 0x59
                    rest = uart.read(7)
                    if len(rest) == 7:
                        buffer[2:] = rest
                        distance = buffer[2] + (buffer[3] << 8)
                        strength = buffer[4] + (buffer[5] << 8)
                        #print("Distance: {} cm | Strength: {}".format(distance, strength))
    return distance

def lidar_E():
    uart = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17))
    buffer = bytearray(9)
    distance = 0
    if uart.any() >= 9:
            byte = uart.read(1)
            if byte == b'\x59':
                next_byte = uart.read(1)
                if next_byte == b'\x59':
                    buffer[0] = 0x59
                    buffer[1] = 0x59
                    rest = uart.read(7)
                    if len(rest) == 7:
                        buffer[2:] = rest
                        distance = buffer[2] + (buffer[3] << 8)
                        strength = buffer[4] + (buffer[5] << 8)
                        #print("Distance: {} cm | Strength: {}".format(distance, strength))
    return distance

def background_thread():
    global IS_BUTTON_PRESSED
    while (1):
        if (_button.value() == 0):
            IS_BUTTON_PRESSED = True
        else:
            IS_BUTTON_PRESSED = False

init()
_thread.start_new_thread(background_thread, ())
_thread.allocate_lock()
