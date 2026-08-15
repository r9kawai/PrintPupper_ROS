# run_robot_caliblate_mode_for_pp_champ.py

# system imports ----------------------
import sys
import math
import numpy as np
from enum import IntEnum, auto
import time
import threading
import queue
import pigpio
import tty
import termios
from pathlib import Path

# -----------------------------------------------------------------------------
class PWMParams:
    def __init__(self):
# PrintPupper pin assigens      FR  FL  BR  BL
        self.pins = np.array([[ 23, 17, 16,  5], \
                              [ 24, 27, 20,  6], \
                              [ 25, 22, 21, 19]])

# -----------------------------------------------------------------------------
class ServoParams:
    def __init__(self):
        svflg = "_servo_type_180deg"
        self.pwm_usec_neutral = 1500
        self.pwm_usec_max = 1500 + 1000
        self.pwm_usec_min = 1500 - 1000
        print(f"{svflg} rotate 1rad in PWM usec {self.pwm_usec_min} - {self.pwm_usec_neutral} - {self.pwm_usec_max}")

        self.pwm_freq = 100
        self.pwm_usec_range = int(1000000 / self.pwm_freq)
        self.pwm_usec_per_rad = (self.pwm_usec_max - self.pwm_usec_min) / np.pi
        return

# -----------------------------------------------------------------------------
class HardwareInterface:
    def __init__(self):
        self.pigpio = pigpio.pi()
        self.pwm_params = PWMParams()
        self.servo_params = ServoParams()
        self.initialize_pwm()
        return

    def set_actuator_position_noncab(self, joint_angle, axis, leg):
        self.send_servo_command_noncab(joint_angle, axis, leg)
        return

    def deactivate(self):
        self.deactivate_servos()
        return

    def initialize_pwm(self):
        print('GPIO ', self.servo_params.pwm_freq, 'Hz ', self.servo_params.pwm_usec_range, 'range ', self.servo_params.pwm_usec_neutral, 'neutral',  sep='')
        for leg_index in range(4):
            if leg_index == 0:
                print('GPIO FR-[ ', end='')
            if leg_index == 1:
                print('GPIO FL-[ ', end='')
            if leg_index == 2:
                print('GPIO BR-[ ', end='')
            if leg_index == 3:
                print('GPIO BL-[ ', end='')
            for axis_index in range(3):
                self.pigpio.set_PWM_frequency(self.pwm_params.pins[axis_index, leg_index], self.servo_params.pwm_freq)
                self.pigpio.set_PWM_range(self.pwm_params.pins[axis_index, leg_index], self.servo_params.pwm_usec_range)
                print('{:02d}'.format(self.pwm_params.pins[axis_index, leg_index]), 'pin ', sep='', end='')
            print(']')
        return

    def deactivate_servos(self):
        for leg_index in range(4):
            for axis_index in range(3):
                self.pigpio.set_PWM_dutycycle(self.pwm_params.pins[axis_index, leg_index], 0)
        return

    def angle_to_pwmdutycycle_noncab(self, angle, axis_index, leg_index):
        usec_val = self.servo_params.pwm_usec_neutral + (self.servo_params.pwm_usec_per_rad * angle)
        usec_val_limited = max(self.servo_params.pwm_usec_min, min(usec_val, self.servo_params.pwm_usec_max))
        return int(usec_val_limited)

    def send_servo_command_noncab(self, joint_angle, axis, leg):
        duty_cycle = self.angle_to_pwmdutycycle_noncab(joint_angle, axis, leg)
        self.pigpio.set_PWM_dutycycle(self.pwm_params.pins[axis, leg], duty_cycle)
        return

# -----------------------------------------------------------------------------
class Btn(IntEnum):
    NEXT = auto()
    EXIT = auto()
    HIP_UP = auto()
    HIP_DOWN = auto()
    LEG_UP = auto()
    LEG_DOWN = auto()
    KNEE_UP = auto()
    KNEE_DOWN = auto()

# -----------------------------------------------------------------------------
class run_robot_caliblate_mode():
    def __init__(self, hardware_interface):
        self.hardwareif = hardware_interface
        self.mode_btn_que = queue.Queue()
        self.subloop_exit = False
        self.subloop_thread = threading.Thread(target=self.pad_input_subloop)
        self.subloop_thread.start()

        print('----- operate key map -----')
        print('n : Select next foot')
        print('  page uo/down    : Servo 1 Hip  +/-')
        print('cursor uo/down    : Servo 2 Leg  +/-')
        print('cursor right/left : Servo 3 Knee +/-')
        print('q : Quit, save, done.')
        print('----------')
        print('All servo to neutral signal, please wait ...')

        self.ndeg_ofsts = np.array([[0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 0.]])
        self.all_servo_neutral_signal()
        print('push \'n\' key operating ...')

        while True:
            try:
                btn = self.mode_btn_que.get(timeout=0.1)
                if btn == Btn.NEXT:
                    break
            except queue.Empty:
                continue

        mode_exit = False
        while True:
            for leg in [1, 3, 0, 2]:
                mode_exit = self.caliblate_leg(leg)
                if mode_exit:
                    break
            if mode_exit:
                break

        self.subloop_exit = True
        self.subloop_thread.join(timeout=1)
        # self.hardwareif.deactivate()

        # output order form to pp_champ
        v = self.ndeg_ofsts
        self.ndeg_ofsts_out = np.array([
            [v[0,1], v[1,1], v[2,1]],
            [v[0,0], v[1,0], v[2,0]],
            [v[0,3], v[1,3], v[2,3]],
            [v[0,2], v[1,2], v[2,2]]
        ])
        self.overwrite_ServoCalibrationConf_file()
        print('Result of calibration -> ./ServoCalibration.conf')
        print(self.ndeg_ofsts_out)
        return

# -------------------------------------
    def pad_input_subloop(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        while not self.subloop_exit:
            c = sys.stdin.read(1)
            if c == "\x1b":
                if sys.stdin.read(1) == "[":
                    c = sys.stdin.read(1)
                    if c == "5":
                        self.mode_btn_que.put(Btn.HIP_UP)
                    elif c == "6":
                        self.mode_btn_que.put(Btn.HIP_DOWN)
                    elif c == "A":
                        self.mode_btn_que.put(Btn.LEG_UP)
                    elif c == "B":
                        self.mode_btn_que.put(Btn.LEG_DOWN)
                    elif c == "C":
                        self.mode_btn_que.put(Btn.KNEE_UP)
                    elif c == "D":
                        self.mode_btn_que.put(Btn.KNEE_DOWN)
            else:
                if c == "q":
                    self.subloop_exit = True
                    self.mode_btn_que.put(Btn.EXIT)
                if c == "n":
                    self.mode_btn_que.put(Btn.NEXT)
            time.sleep(0.1)

        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return

# -------------------------------------
    def set_actuator(self, deg, axis, leg):
        rad = deg * np.pi / 180.0
        self.hardwareif.set_actuator_position_noncab(rad, axis, leg)
        return

# -------------------------------------
    def overwrite_ServoCalibrationConf_file(self):
        formatted_str = [[x for x in row] for row in self.ndeg_ofsts_out]
        with open("./ServoCalibration.conf", "w") as f:
            print(formatted_str, file = f)
        return

# -------------------------------------
    def caliblate_leg(self, leg):
        leg_pos = {0: "Front-Right", 1: "Front-Left", 2: "Back-Right", 3: "Back-Left"}
        print("caliblate_leg", leg_pos[leg])

        hip_val = self.ndeg_ofsts[0, leg]
        leg_val = self.ndeg_ofsts[1, leg]
        knee_val = self.ndeg_ofsts[2, leg]
        print(f"Sv1 Hip={hip_val} / Sv2 Leg={leg_val} / Sv3 Knee={knee_val}")
        self.set_actuator(hip_val, 0, leg)
        self.set_actuator(leg_val, 1, leg)
        self.set_actuator(knee_val, 2, leg)

        self.set_actuator(hip_val + 5, 0, leg)
        time.sleep(0.25)
        self.set_actuator(hip_val,  0, leg)
        time.sleep(0.25)
        self.set_actuator(hip_val + 5, 0, leg)
        time.sleep(0.25)
        self.set_actuator(hip_val,  0, leg)

        mode_exit = False
        btn = 0
        updown_rv = [1, -1, 1, -1]
        knee_updown_rv = updown_rv[leg]
        while True:
            try:
                btn = self.mode_btn_que.get(timeout=0.1)
            except queue.Empty:
                continue

            if btn == Btn.HIP_UP:
                hip_val += 0.5
            if btn == Btn.HIP_DOWN:
                hip_val -= 0.5
            if btn == Btn.LEG_UP:
                leg_val += 0.5
            if btn == Btn.LEG_DOWN:
                leg_val -= 0.5
            if btn == Btn.KNEE_UP:
                knee_val += (0.5 * knee_updown_rv)
            if btn == Btn.KNEE_DOWN:
                knee_val -= (0.5 * knee_updown_rv)

            print(f"Sv1 Hip={hip_val} / Sv2 Leg={leg_val} / Sv3 Knee={knee_val}")
            self.set_actuator(hip_val, 0, leg)
            self.set_actuator(leg_val, 1, leg)
            self.set_actuator(knee_val, 2, leg)

            if btn == Btn.NEXT:
                print('Next')
                break
            if btn == Btn.EXIT:
                print('Quit')
                mode_exit = True
                break

            btn = 0

        self.ndeg_ofsts[0, leg] = hip_val
        self.ndeg_ofsts[1, leg] = leg_val
        self.ndeg_ofsts[2, leg] = knee_val
        return mode_exit

# -------------------------------------
    def all_servo_neutral_signal(self):
        degree = 0
        for leg in range(0,4):
            for axis in range(0,3):
                self.set_actuator(degree, axis, leg)
                print(f'signal boot Leg{leg}-Servo{axis+1}')
                time.sleep(0.4)
        return

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    print('Start : run_robot_caliblate_mode_for_pp_champ.py')
    hardware_interface = HardwareInterface()
    run_robot_caliblate_mode(hardware_interface)
    print('run_robot_caliblate_mode_for_pp_champ.py : Done')

