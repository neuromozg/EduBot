import smbus as I2C
import threading
import time

#библиотеки для работы с i2c монитором питания INA219
#установка библиотеки sudo pip3 install pi-ina219
from ina219 import INA219
from ina219 import DeviceRangeError

I2C_1 = 1 #номер шины I2C

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

SHUNT_OHMS = 0.01 #значение сопротивления шунта на плате EduBot
MAX_EXPECTED_AMPS = 2.0

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
            direction = int(speed > 0)
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
    def __init__(self):
        self._bus = I2C.SMBus(I2C_1) #объект для работы с шиной I2C
        self.busLock = threading.Lock() #блокировка для раздельного доступа к I2C
        self.leftMotor = _Motor(self._bus, self.busLock, REG_DIR0, REG_PWM0)
        self.rightMotor = _Motor(self._bus, self.busLock, REG_DIR1, REG_PWM1)
        self._servo0 = _Servo(self._bus, self.busLock, 0)
        self._servo1 = _Servo(self._bus, self.busLock, 1)
        self._servo2 = _Servo(self._bus, self.busLock, 2)
        self._servo3 = _Servo(self._bus, self.busLock, 3)
        self.servo = (self._servo0, self._servo1, self._servo2, self._servo3)
        self._onLiner = _OnLiner(self._bus, self.busLock)
        #конфигурируем INA219
        self.busLock.acquire()
        try:
            self._ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS) #создаем обект для работы с INA219
            self._ina.configure(INA219.RANGE_16V)
        finally:
            self.busLock.release()

    def Check(self):
        self.busLock.acquire()
        try:
            res = self._bus.read_byte_data(EDUBOT_ADDRESS, REG_WHY_IAM)
        finally:
            self.busLock.release()
        return (res == 0X2A) #возвращает True если полученный байт является ответом на главный вопрос жизни, вселенной и всего такого

    def Start(self):
        self._onLiner.start()

    def Release(self):
        self._onLiner.stop()

    def Beep(self):
        self.busLock.acquire()
        try:
            self._bus.write_byte_data(EDUBOT_ADDRESS, REG_BEEP, 3)
        finally:
            self.busLock.release()

    def GetPowerData(self):
        voltage = 0
        current = 0
        power = 0
        
        self.busLock.acquire()
        try:
            voltage = self._ina.voltage()
            current = self._ina.current()
            power = self._ina.power()
        finally:
            self.busLock.release()
        
        return voltage, current, power


        

    
