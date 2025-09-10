from machine import Pin, I2C
import utime
from picobrickss import NEC_16, IR_RX, MotorDriver, HCSR04

FAST = 255
SLOW = 120
FORWARD_TIME = 1500
OTHER_TIME   = 1500
OBSTACLE_DIST = 10.0

ir_data   = None
data_rcvd = False

def ir_callback(data, addr, ctrl):
    global ir_data, data_rcvd
    if data > 0:
        ir_data   = data
        data_rcvd = True
        print("IR DATA:", hex(data), "ADDR:", hex(addr))

i2c   = I2C(0, scl=Pin(22), sda=Pin(21))
motor = MotorDriver(i2c)
# IR pini 17, pull-up ile
ir    = NEC_16(Pin(17, Pin.IN, Pin.PULL_UP), ir_callback)
sonic = HCSR04(26, 27)

def stop():
    motor.dc(1, 0, 0)
    motor.dc(2, 0, 0)

def forward():
    motor.dc(1, FAST, 0)
    motor.dc(2, FAST, 0)

def backward():
    motor.dc(1, FAST, 1)
    motor.dc(2, FAST, 1)

def turn_left():
    motor.dc(1, SLOW, 0)
    motor.dc(2, SLOW, 1)

def turn_right():
    motor.dc(1, SLOW, 1)
    motor.dc(2, SLOW, 0)

while True:
    if data_rcvd:
        data_rcvd = False
        dist = sonic.distance_cm()

        if ir_data == IR_RX.number_up:
            print("CMD: FORWARD")
            if dist < 0 or dist > OBSTACLE_DIST:
                forward()
                start = utime.ticks_ms()
                while utime.ticks_diff(utime.ticks_ms(), start) < FORWARD_TIME:
                    d2 = sonic.distance_cm()
                    if 0 < d2 < OBSTACLE_DIST:
                        break
                    utime.sleep_ms(10)
                stop()
            else:
                stop()

        elif ir_data == IR_RX.number_down:
            print("CMD: BACKWARD")
            backward()
            utime.sleep_ms(OTHER_TIME)
            stop()

        elif ir_data == IR_RX.number_left:
            print("CMD: LEFT")
            turn_left()
            utime.sleep_ms(OTHER_TIME)
            stop()

        elif ir_data == IR_RX.number_right:
            print("CMD: RIGHT")
            turn_right()
            utime.sleep_ms(OTHER_TIME)
            stop()

        elif ir_data == IR_RX.number_ok:
            print("CMD: STOP")
            stop()

    utime.sleep_ms(20)
