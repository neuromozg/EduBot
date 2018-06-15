import smbus as I2C

#I2C Address of device
EDUBOT_ADDRESS = 0x27

REG_WHY_IAM = 0x00
REG_ONLINE  = 0x01
REG_VOLTAGE = 0x02
REG_CURRENT = 0x03
REG_SERVO0  = 0x04
REG_SERVO1  = 0x05
REG_SERVO2  = 0x06
REG_SERVO3  = 0x07
REG_DIR0    = 0x08
REG_PWM0    = 0x09
REG_DIR1    = 0x0A
REG_PWM1    = 0x0B

REF_VOLTAGE = 3.3 #опорное напряжение

class Motor():
    def __init__(self, bus, regDir, regPWM):
        self._bus = bus #объект для работы с шиной I2C
        self._regDir = regDir
        self._regPWM = regPWM
        

    def SetSpeed(self, speed):
        if speed > 255:
            speed = 255
        elif speed < -255:
            speed = -255

        #задаем направление
        direction = (speed > 0)
        self._bus.write_byte_data(EDUBOT_ADDRESS, self._regDir, direction)

        #задаем ШИМ
        self._bus.write_byte_data(EDUBOT_ADDRESS, self._regPWM, abs(speed))

class Servo():
    def __init__(self, bus, numServo):
        self._bus = bus #объект для работы с шиной I2C
        self._numServo = numServo

    def SetPosition(self, pos):
        if pos < 0:
            pos = 0
        elif pos > 125:
            pos = 125

        #задаем позицию
        self._bus.write_byte_data(EDUBOT_ADDRESS, REG_SERVO0 + self._numServo, pos)
          
class EduBot():
    def __init__(self, busNumber):
        self._bus = I2C.SMBus(busNumber) #объект для работы с шиной I2C
        self.leftMotor = Motor(self._bus, REG_DIR0, REG_PWM0)
        self.rightMotor = Motor(self._bus, REG_DIR1, REG_PWM1)
        self._servo0 = Servo(self._bus, 0)
        self._servo1 = Servo(self._bus, 1)
        self._servo2 = Servo(self._bus, 2)
        self._servo3 = Servo(self._bus, 3)
        self.servo = (self._servo0, self._servo1, self._servo2, self._servo3)

    def Check(self):
        res = self._bus.read_byte_data(EDUBOT_ADDRESS, REG_WHY_IAM)
        return (res == 0X2A) #True если полученный байт является ответом на главный вопрос жизни, вселенной и всего такого

    def GetVoltage(self):
        voltageADC = self._bus.read_byte_data(EDUBOT_ADDRESS, REG_VOLTAGE)
        voltage = (REF_VOLTAGE*voltageADC*4)/256
        return voltage

    def GetCurrent(self):
        currentADC = self._bus.read_byte_data(EDUBOT_ADDRESS, REG_CURRENT)
        # Is = (Vout x 1k) / (RS x RL) RS = 0.01 RL = 10
        current = ((REF_VOLTAGE*currentADC)/1024)/0.1
        return current
