#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import edubot

SPEED = 200
SERVO_MID_POS = 62

robot = edubot.EduBot(1)

assert robot.Check(), 'EduBot not found!!!'

robot.Start()
print ('EduBot started!!!')

shiftServoPos = 20

try:
    while True:
        robot.leftMotor.SetSpeed(SPEED)
        robot.rightMotor.SetSpeed(SPEED)
        time.sleep(1)
        print('Voltage: %.2f, Current: %.2f' % (robot.GetVoltage(), robot.GetCurrent()))
        robot.leftMotor.SetSpeed(0)
        robot.rightMotor.SetSpeed(0)
        time.sleep(1)
        print('Voltage: %.2f, Current: %.2f' % (robot.GetVoltage(), robot.GetCurrent()))
        
        robot.servo[0].SetPosition(SERVO_MID_POS + shiftServoPos)
        shiftServoPos = -shiftServoPos

        robot.Beep()
            
except KeyboardInterrupt:
    print('Ctrl+C pressed')



robot.leftMotor.SetSpeed(0)
robot.rightMotor.SetSpeed(0)

robot.servo[0].SetPosition(SERVO_MID_POS)
robot.servo[1].SetPosition(SERVO_MID_POS)
robot.servo[2].SetPosition(SERVO_MID_POS)
robot.servo[3].SetPosition(SERVO_MID_POS)

robot.Release()
print('Stop EduBot')
