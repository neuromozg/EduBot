#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

#библиотеки для работы с i2c монитором питания INA219
from ina219 import INA219
from ina219 import DeviceRangeError

#библиетека для работы с OLED дисплеем
import Adafruit_SSD1306

#библиотеки для работы с изображениями Python Image Library
from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw

#библиотека для работы с контроллером EduBot
import edubot

SPEED = 200 #скорость моторов
SERVO_MID_POS = 62 #среднее положение сервомотора
I2C_1 = 1 #номер шины I2C

SHUNT_OHMS = 0.01 #значение сопротивления шунта на плате EduBot
MAX_EXPECTED_AMPS = 2.0

robot = edubot.EduBot(I2C_1) #создаем обект для работы с EduBot (параметр, номер шины i2c)

ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS) #создаем обект для работы с INA219
ina.configure(ina.RANGE_16V)

disp = Adafruit_SSD1306.SSD1306_128_64(rst = None) #создаем обект для работы c OLED дисплеем 128х64

disp.begin() #инициализируем дисплей

width = disp.width #получаем ширину и высоту дисплея в пикселях
height = disp.height
image = Image.new('1', (width, height)) #создаем ч/б картинку для отрисовки на дисплее

draw = ImageDraw.Draw(image) #создаем объект для рисования на картинке

disp.clear() #очищаем дисплей
disp.display() #обновляем дисплей

font = ImageFont.load_default() #создаем шрифт для отрисовки на дисплее

assert robot.Check(), 'EduBot not found!!!' #проверяем наличие платы

robot.Start() #обязательная процедура, запуск потока отправляющего на контроллер EduBot онлайн сообщений
print ('EduBot started!!!')

shiftServoPos = 20

robot.Beep() #бибикаем

try:
    while True:
        #крутим моторами
        robot.leftMotor.SetSpeed(SPEED)
        robot.rightMotor.SetSpeed(SPEED)
        time.sleep(1)
        robot.leftMotor.SetSpeed(0)
        robot.rightMotor.SetSpeed(0)
        time.sleep(1)

        #наклоняем серво
        robot.servo[0].SetPosition(SERVO_MID_POS + shiftServoPos)
        shiftServoPos = -shiftServoPos #инвертируем положение
        
        # Отрисовываем на картинке черный прямоугольник, тем самым её очищая
        draw.rectangle((0, 0, width, height), outline=0, fill=0)

        #Отрисовываем строчки текста с текущими значениями напряжения, сылы тока и мощности
        draw.text((0, 0), "EDUBOT PROJECT", font=font, fill=255)
        draw.text((0, 10), "Voltage: %.2f" % ina.voltage(), font=font, fill=255)
        draw.text((0, 20), "Current: %.2f" % ina.current(), font=font, fill=255)
        draw.text((0, 30), "Power: %.2f" % ina.power(), font=font, fill=255)
        
        # Копируем картинку на дисплей
        disp.image(image)

        #Обновляем дисплей
        disp.display()
            
except KeyboardInterrupt:
    print('Ctrl+C pressed')

robot.Beep() #бибикаем

# Очистка дисплея
disp.clear()
disp.display()

#останов двигателей
robot.leftMotor.SetSpeed(0)
robot.rightMotor.SetSpeed(0)

#все сервы в среднее положение
robot.servo[0].SetPosition(SERVO_MID_POS)
robot.servo[1].SetPosition(SERVO_MID_POS)
robot.servo[2].SetPosition(SERVO_MID_POS)
robot.servo[3].SetPosition(SERVO_MID_POS)

#останавливаем поток отправки онлайн сообщений в контроллер EduBot
robot.Release()
print('Stop EduBot')
