#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import edubot

SPEED = 128

robot = edubot.EduBot(1)

assert robot.Check(), 'EduBot not found!!!'

print ('EduBot started!!!')

pos = 0

try:
    while True:
        robot.leftMotor.SetSpeed(SPEED)
        robot.rightMotor.SetSpeed(SPEED)
        time.sleep(1)
        robot.leftMotor.SetSpeed(0)
        robot.rightMotor.SetSpeed(0)
        time.sleep(0.5)
        robot.leftMotor.SetSpeed(-SPEED)
        robot.rightMotor.SetSpeed(-SPEED)
        time.sleep(1)
        robot.leftMotor.SetSpeed(0)
        robot.rightMotor.SetSpeed(0)
        time.sleep(0.5)

        robot.

        pos += 25
        if pos > 125:
            pos = 0

        print('Voltage: %d, Current: %d' % (robot.GetVoltage(), robot.GetCurrent()))
            
except KeyboardInterrupt:
    print('Ctrl+C pressed')


robot.leftMotor.SetSpeed(0)
robot.rightMotor.SetSpeed(0)

robot.servo[0].SetPosition(0)
robot.servo[1].SetPosition(0)
robot.servo[2].SetPosition(0)
robot.servo[3].SetPosition(0)

print('Stop EduBot')
