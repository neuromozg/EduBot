import smbus as I2C
import threading
import time

#I2C Address of device
EDUBOT_ADDRESS = 0x27

REG_WHY_IAM = 0x00
REG_ONLINE  = 0x01
REG_SERVO0  = 0x02
REG_SERVO1  = 0x03
REG_SERVO2  = 0x04
REG_SERVO3  = 0x05
REG_DIR0    = 0x06
REG_PWM0    = 0x07
REG_DIR1    = 0x08
REG_PWM1    = 0x09
REG_BEEP    = 0x0A

class _Motor():
    def __init__(self, bus, lock, regDir, regPWM):
        self._bus = bus #объект для работы с шиной I2C
        self._busLock = lock #блокировка для раздельного доступа к I2C
        self._regDir = regDir
        self._regPWM = regPWM
        

    def SetSpeed(self, speed):
        #нормализуем скорость
        if speed > 255:
            speed = 255
        elif speed < -255:
            speed = -255
            
        self._busLock.acquire()
        try:   
            #задаем направление
            direction = (speed > 0)
            self._bus.write_byte_data(EDUBOT_ADDRESS, self._regDir, direction)
            #задаем ШИМ
            self._bus.write_byte_data(EDUBOT_ADDRESS, self._regPWM, abs(speed))
        finally:
            self._busLock.release()

class _Servo():
    def __init__(self, bus, lock, numServo):
        self._bus = bus #объект для работы с шиной I2C
        self._busLock = lock #блокировка для раздельного доступа к I2C
        self._numServo = numServo

    def SetPosition(self, pos):
        #нормализуем позицию
        if pos < 0:
            pos = 0
        elif pos > 125:
            pos = 125
            
        self._busLock.acquire()
        try:    
            #задаем позицию
            self._bus.write_byte_data(EDUBOT_ADDRESS, REG_SERVO0 + self._numServo, pos)
        finally:
            self._busLock.release()

class _OnLiner(threading.Thread):
    def __init__(self, bus, lock):
        super(_OnLiner, self).__init__()
        self._bus = bus
        self._busLock = lock #блокировка для раздельного доступа к I2C
        self.daemon = True
        self._stopped = threading.Event() #событие для остановки потока

    def run(self):
        print('OnLiner thread started')
        
        while not self._stopped.is_set():
            self._busLock.acquire()
            try:
                self._bus.write_byte_data(EDUBOT_ADDRESS, REG_ONLINE, 1)
            finally:
                self._busLock.release()
            time.sleep(1)
            
        print('OnLiner thread stopped')

    def stop(self): #остановка потока
        self._stopped.set()
        self.join()
        
          
class EduBot():
    def __init__(self, busNumber):
        self._bus = I2C.SMBus(busNumber) #объект для работы с шиной I2C
        self._busLock = threading.Lock() #блокировка для раздельного доступа к I2C
        self.leftMotor = _Motor(self._bus, self._busLock, REG_DIR0, REG_PWM0)
        self.rightMotor = _Motor(self._bus, self._busLock, REG_DIR1, REG_PWM1)
        self._servo0 = _Servo(self._bus, self._busLock, 0)
        self._servo1 = _Servo(self._bus, self._busLock, 1)
        self._servo2 = _Servo(self._bus, self._busLock, 2)
        self._servo3 = _Servo(self._bus, self._busLock, 3)
        self.servo = (self._servo0, self._servo1, self._servo2, self._servo3)
        self._onLiner = _OnLiner(self._bus, self._busLock)

    def Check(self):
        self._busLock.acquire()
        try:
            res = self._bus.read_byte_data(EDUBOT_ADDRESS, REG_WHY_IAM)
        finally:
            self._busLock.release()
        return (res == 0X2A) #True если полученный байт является ответом на главный вопрос жизни, вселенной и всего такого

    def Start(self):
        self._onLiner.start()

    def Release(self):
        self._onLiner.stop()

    def Beep(self):
        self._busLock.acquire()
        try:
            self._bus.write_byte_data(EDUBOT_ADDRESS, REG_BEEP, 3)
        finally:
            self._busLock.release()
        

    
