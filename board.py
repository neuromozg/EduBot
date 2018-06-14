#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import edubot #подлючение модулей
from time import sleep
from xmlrpc.server import SimpleXMLRPCServer
import threading

IP = '173.1.0.29'
PORT = 8000

servoPos = 62

#поток для контроля напряжения и тока
#параметр 
class StateThread(threading.Thread):
    
    def __init__(self, robot):
        super(StateThread, self).__init__()
        self.daemon = True
        self._stopped = threading.Event() #событие для остановки потока
        self._robot = robot

    def run(self):
        print('State thread started')
        while not self._stopped.is_set():
            voltage = self._robot.GetVoltage()
            current = self._robot.GetCurrent()
            print('Voltage: %.2f, Current: %d' % (voltage, current))
            sleep(1)
            
        print('State thread stopped')

    def stop(self): #остановка потока
        self._stopped.set()
        self.join()

def Speak():
    # бибикнуть
    return 0

def SetSpeed(leftSpeed, rightSpeed):
    robot.leftMotor.SetSpeed(leftSpeed)
    robot.rightMotor.SetSpeed(rightSpeed)
    return 0

def StopMotor():    
    robot.leftMotor.SetSpeed(0)
    robot.rightMotor.SetSpeed(0)
    return 0

def ServoUp():
    global servoPos
    servoPos -= 10
    if servoPos < 0:
        servoPos = 0
    robot.servo[0].SetPosition(servoPos)
    print ('ServoPos = %d' % servoPos)
    return 0

def ServoDown():
    global servoPos
    servoPos += 10
    if servoPos > 125:
        servoPos = 125
    robot.servo[0].SetPosition(servoPos)
    print ('ServoPos = %d' % servoPos)
    return 0

robot = edubot.EduBot(1)
assert robot.Check(), 'EduBot not found!!!'
print ('EduBot started!!!')

stateThread = StateThread(robot)
stateThread.start()

# Создаем сервер (IP адрес, порт, отключен лог)
server = SimpleXMLRPCServer((IP, PORT), logRequests = False)
print('Control XML-RPC server listening on %s:%d' % (IP, PORT))

# регистрируем функции
server.register_function(Speak)
server.register_function(SetSpeed)
server.register_function(StopMotor)
server.register_function(ServoUp)
server.register_function(ServoDown)

# запускаем сервер
try:
    server.serve_forever()
except KeyboardInterrupt:
    robot.servo[0].SetPosition(62)
    robot.leftMotor.SetSpeed(0)
    robot.rightMotor.SetSpeed(0)
    stateThread.stop()
    print('Stop program')
