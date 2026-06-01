import time

import RPi.GPIO as GPIO


LEFT = 1
RIGHT = 2


def valid_speed(speed):
    if speed > 100:
        return 100
    if speed < 0:
        return 0
    return speed


class MotionController:
    def __init__(self, left_speed=30, right_speed=30, frequency=55):
        self.ea, self.i2, self.i1, self.eb, self.i4, self.i3 = (13, 19, 26, 16, 20, 21)
        self.left_speed = left_speed
        self.right_speed = right_speed

        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.ea, self.i2, self.i1, self.eb, self.i4, self.i3], GPIO.OUT)
        GPIO.output([self.ea, self.i2, self.eb, self.i3], GPIO.LOW)
        GPIO.output([self.i1, self.i4], GPIO.HIGH)

        self.pwm_right = GPIO.PWM(self.ea, frequency)
        self.pwm_left = GPIO.PWM(self.eb, frequency)
        self.pwm_right.start(0)
        self.pwm_left.start(0)

    def move_forward(self):
        self.pwm_right.ChangeDutyCycle(self.right_speed)
        self.pwm_left.ChangeDutyCycle(self.left_speed)

    def stop(self):
        self.pwm_right.ChangeDutyCycle(0)
        self.pwm_left.ChangeDutyCycle(0)

    def turn_left_not_in_place(self):
        self.pwm_right.ChangeDutyCycle(30)
        self.pwm_left.ChangeDutyCycle(20)
        time.sleep(0.1)
        self.move_forward()
        return LEFT

    def turn_right_not_in_place(self):
        self.pwm_right.ChangeDutyCycle(20)
        self.pwm_left.ChangeDutyCycle(30)
        time.sleep(0.1)
        self.move_forward()
        return RIGHT

    def turn_in_place(self, direction, seconds, duty_cycle=20):
        if direction == LEFT:
            self.pwm_right.ChangeDutyCycle(duty_cycle)
            self.pwm_left.ChangeDutyCycle(0)
        elif direction == RIGHT:
            self.pwm_right.ChangeDutyCycle(0)
            self.pwm_left.ChangeDutyCycle(duty_cycle)
        time.sleep(seconds)
        self.stop()

    def move_forward_for(self, seconds):
        self.move_forward()
        time.sleep(seconds)
        self.stop()

    def safe_move_forward_for(self, seconds, safety_check=None):
        start = time.time()
        while time.time() - start < seconds:
            if safety_check is not None and safety_check():
                continue
            self.move_forward()
            time.sleep(0.05)
        self.stop()

    def cleanup(self):
        self.stop()
        self.pwm_right.stop()
        self.pwm_left.stop()
        GPIO.cleanup()

