/*
 * main.c
 *
 *  Created on: 31.05.2018
 *      Author: max
 */

#include <avr/io.h>
#include <util/delay.h>
#include <compat/twi.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>

//reset
#define RESET_BIT			PC6
#define RESET_DDR			DDRC
#define RESET_PORT			PORTC

//светодиод
#define LED_BIT				PB2
#define LED_DDR				DDRB
#define LED_PORT			PORTB

//пищалка
#define BUZZER_BIT			PD3
#define BUZZER_DDR			DDRD
#define BUZZER_PORT			PORTD

//моторы
#define MOTOR0_PWM_BIT		PD6
#define MOTOR0_PWM_DDR		DDRD
#define MOTOR0_PWM_PORT		PORTD

#define MOTOR0_INA_BIT		PB1
#define MOTOR0_INA_DDR		DDRB
#define MOTOR0_INA_PORT		PORTB

#define MOTOR0_INB_BIT		PD7
#define MOTOR0_INB_DDR		DDRD
#define MOTOR0_INB_PORT		PORTD

#define MOTOR0_TIMER_REG	OCR0A

#define MOTOR1_PWM_BIT		PD5
#define MOTOR1_PWM_DDR		DDRD
#define MOTOR1_PWM_PORT		PORTD

#define MOTOR1_INA_BIT		PD4
#define MOTOR1_INA_DDR		DDRD
#define MOTOR1_INA_PORT		PORTD

#define MOTOR1_INB_BIT		PD2
#define MOTOR1_INB_DDR		DDRD
#define MOTOR1_INB_PORT		PORTD

#define MOTOR1_TIMER_REG	OCR0B

//i2c
#define I2C_SLAVE_ADDR      0x27

#define REG_WHY_IAM			0x00
#define REG_ONLINE			0x01
#define REG_SERVO0			0x02
#define REG_SERVO1			0x03
#define REG_SERVO2			0x04
#define REG_SERVO3			0x05
#define REG_DIR0			0x06
#define REG_PWM0			0x07
#define REG_DIR1			0x08
#define REG_PWM1			0x09
#define REG_BEEP			0x0A


//MultiServo
#define MaxServo 		4 //количество серв
#define Tick_1ms 		125 //количество тиков таймера в 1мс
#define	SERVO_PORT 		PORTC
#define	SERVO_DDR 		DDRC

uint8_t onLine = 0; //признак того что система доступна с Linux компьютера
uint8_t offLineCount = 0;
uint8_t offLineLedCount = 0;

unsigned char regAddr; // Store the Requested Register Address
unsigned char regData; // Store the Register Address Data

uint8_t dirMotor0;
uint8_t dirMotor1;

typedef struct {
	uint8_t position;
	uint8_t bit;
} SArray_def;

SArray_def servo[MaxServo];
SArray_def *servoSorted[MaxServo];

//uint8_t servo_need_update = 0;
uint8_t servoState = 0; //значение конечного автомата
uint8_t servoPortState[MaxServo+1]; // Значение порта которое надо вывести
uint8_t servoNextOCR[MaxServo+1]; // Время вывода значения

uint8_t servoNeedUpdate;

uint8_t adcCounter = 0; //текущий ацп из массива adcValueList
uint16_t tmpAdcSumm = 0; //сумма значений АЦП для усреднения
uint8_t averageCounter = 0; //счетчик усредняемых значений

uint8_t buzzerActive = 0;
uint8_t buzzerTime = 0;

void I2CSlaveAction(unsigned char rwStatus);
void ServoSort(void);
void ServoUpd(void);
void ServoInit(void);
void ServoSetPos(uint8_t servoNum, uint8_t pos);

ISR(TWI_vect)
{
    static unsigned char i2c_state;
    unsigned char twi_status;

    // Disable Global Interrupt
    cli();

    // Get TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
    twi_status=TWSR & 0xF8;

    switch(twi_status) {
        case TW_SR_SLA_ACK: // 0x60: SLA+W received, ACK returned
            i2c_state=0;    // Start I2C State for Register Address required
            break;

        case TW_SR_DATA_ACK:    // 0x80: data received, ACK returned
            if (i2c_state == 0) {
                regAddr = TWDR; // Save data to the register address
                i2c_state = 1;
            } else {
                regData = TWDR; // Save to the register data
                i2c_state = 2;
            }
            break;

        case TW_SR_STOP:    // 0xA0: stop or repeated start condition received while selected
            if (i2c_state == 2) {
            	I2CSlaveAction(1);    // Call Write I2C Action (rw_status = 1)
                i2c_state = 0;      // Reset I2C State
            }
            break;

        case TW_ST_SLA_ACK: // 0xA8: SLA+R received, ACK returned
        case TW_ST_DATA_ACK:    // 0xB8: data transmitted, ACK received
            if (i2c_state == 1) {
            	I2CSlaveAction(0);    // Call Read I2C Action (rw_status = 0)
                TWDR = regData;     // Store data in TWDR register
                i2c_state = 0;      // Reset I2C State
            }
            break;

        case TW_ST_DATA_NACK:   // 0xC0: data transmitted, NACK received
        case TW_ST_LAST_DATA:   // 0xC8: last data byte transmitted, ACK received
        case TW_BUS_ERROR:  // 0x00: illegal start or stop condition
        default:
            i2c_state = 0;  // Back to the Begining State
    }

    // Clear TWINT Flag
    TWCR |= (1<<TWINT);
    // Enable Global Interrupt
    sei();
}

ISR (TIMER1_COMPA_vect)
{ // Прерывание по совпадению
	//cli(); //запрет прерываний

	uint8_t LenghtPPM = servoNextOCR[servoState]; 	// Берем следующий интервал

	if (servoState)		// Если не нулевое состояние
	{
		SERVO_PORT &= ~servoPortState[servoState];	// Сбрасываем биты в порту, в соответствии с маской в массиве масок.

		if (LenghtPPM == 0xFF) // Если значение сравнения равно FF значит это заглушка
		{					// И мы достигли конца таблицы. И пора обнулить автомат
			servoState = 0;	// Выставляем нулевое состояние автомата.

			OCR1A = 20*Tick_1ms; 		//ставим задержку на 15мс

			if (servoNeedUpdate)		// Если поступил приказ обновить таблицы автомата
			{
					ServoUpd();			// Обновляем таблицы.
					servoNeedUpdate = 0;	// Сбрасываем сигнал обновления.
			}
		}
		else
		{
			OCR1A = Tick_1ms + LenghtPPM;	// В регистр сравнения кладем следующий интервал (1мс + интервал)
			servoState++;					// Увеличиваем состояние автомата
		}
	}
	else
	{
		TCNT1 = 0;		//сброс таймера

		OCR1A = Tick_1ms + LenghtPPM;	// В регистр сравнения кладем следующий интервал (1мс + интервал)
		SERVO_PORT |= 0b00001111;		// Выставялем все сервоканалы в 1 - начало импульса

		servoState++;					// Увеличиваем состояние автомата
	}

	//sei(); //разрешение прерываний
}

void I2CSlaveAction(unsigned char rwStatus)
{
	switch(regAddr) {

	// PORT
	case REG_WHY_IAM:
		if (rwStatus == 0)
			// read
			regData = 0x2A; //Ответ на главный вопрос жизни, вселенной и всего такого
		break;
	case REG_ONLINE:
		if (rwStatus)
		{
			if (onLine == 0) //если система в оффлайне
				onLine = 1;
			offLineCount = 0; //сбрасываем счетчик офлайна
		}
		break;
	case REG_SERVO0:
		if (rwStatus)
		{
			if (onLine)
				ServoSetPos(0, regData);
		}
		else
			regData = servo[0].position;
		break;
	case REG_SERVO1:
		if (rwStatus)
		{
			if (onLine)
				ServoSetPos(1, regData);
		}
		else
			regData = servo[1].position;
		break;
	case REG_SERVO2:
		if (rwStatus)
		{
			if (onLine)
				ServoSetPos(2, regData);
		}
		else
			regData = servo[2].position;
		break;
	case REG_SERVO3:
		if (rwStatus)
		{
			if (onLine)
				ServoSetPos(3, regData);
		}
		else
			regData = servo[3].position;
		break;
	case REG_DIR0:
		if (rwStatus)
		{
			if (onLine)
			{
				if (regData)
				{
					MOTOR0_INA_PORT |= (1 << MOTOR0_INA_BIT);
					MOTOR0_INB_PORT &= ~(1 << MOTOR0_INB_BIT);
					dirMotor0 = 1;
				}
				else
				{
					MOTOR0_INA_PORT &= ~(1 << MOTOR0_INA_BIT);
					MOTOR0_INB_PORT |= (1 << MOTOR0_INB_BIT);
					dirMotor0 = 0;
				}
			}
		}
		else
			regData = dirMotor0;
		break;
	case REG_PWM0:
		if (rwStatus)
		{
			if (onLine)
				MOTOR0_TIMER_REG = regData;
		}
		else
			regData = MOTOR0_TIMER_REG;
		break;
	case REG_DIR1:
		if (rwStatus)
		{
			if (onLine)
			{
				if (regData)
				{
					MOTOR1_INA_PORT |= (1 << MOTOR1_INA_BIT);
					MOTOR1_INB_PORT &= ~(1 << MOTOR1_INB_BIT);
					dirMotor1 = 1;
				}
				else
				{
					MOTOR1_INA_PORT &= ~(1 << MOTOR1_INA_BIT);
					MOTOR1_INB_PORT |= (1 << MOTOR1_INB_BIT);
					dirMotor1 = 0;
				}
			}
		}
		else
			regData = dirMotor1;
		break;
	case REG_PWM1:
		if (rwStatus)
		{
			if (onLine)
				MOTOR1_TIMER_REG = regData;
		}
		else
			regData = MOTOR1_TIMER_REG;
		break;
	case REG_BEEP:
		if (rwStatus)
		{
			buzzerTime = regData;
			TCCR2A |= (1 << COM2B0); //включаем ногу динамика
		}
		break;
	default:
		regData = 0x00;
	}
}

//Простейший алгоритм сортировки вставкой. Недалеко ушел от пузырька, но на столь малых количествах
// данных являетя наиболее эффективным.
void ServoSort(void)
{
	uint8_t i, k;
	SArray_def *tmp;

	// Сортируем массив указателей.
	for(i=1; i<MaxServo; i++) {
		for(k=i; ((k>0)&&(servoSorted[k]->position < servoSorted[k-1]->position)); k--) {
			tmp = servoSorted[k];					// Swap [k,k-1]
			servoSorted[k] = servoSorted[k-1];
			servoSorted[k-1] = tmp;
		}
	}
}

void ServoUpd(void)
{
	uint8_t i,j,k;

	for(i=0, k=0; i<MaxServo; i++, k++)
	{
		if(servoSorted[i]->position != servoSorted[i+1]->position)	//Если значения уникальные
		{
			servoNextOCR[k] = servoSorted[i]->position;			// Записываем их как есть
			servoPortState[k+1] = servoSorted[i]->bit;			// И битмаску туда же
		}
		else								// Но если совпадает со следующим
		{
			servoNextOCR[k] = servoSorted[i]->position;			// Позицию записываем
			servoPortState[k+1] = servoSorted[i]->bit;			// Записываем битмаску

			// И в цикле ищем все аналогичные позиции, склеивая их битмаски в одну.

			for(j=1; (servoSorted[i]->position == servoSorted[i+j]->position)&&(i+j < MaxServo); j++)
				servoPortState[k+1] |= servoSorted[i+j]->bit;
			i+=j-1;						// Перед выходом корректируем индекс
		}						// На глубину зарывания в повторы
	}
	servoNextOCR[k] = 0xFF;		// В последний элемент вписываем заглушку FF.
}

void ServoInit(void) {
	servoSorted[0] = &servo[0];
	servoSorted[1] = &servo[1];
	servoSorted[2] = &servo[2];
	servoSorted[3] = &servo[3];


	servo[0].bit = 0b00000001;
	servo[1].bit = 0b00000010;
	servo[2].bit = 0b00000100;
	servo[3].bit = 0b00001000;

	//ServoSetZeroPos(NULL);

	//ServoSort();
	ServoUpd(); //вызываем для заполнения массивов servoNextOCR и servoPortState

	//настройка таймера 1 для формирования PPM сигналов
	TCCR1A = 0; //простой режим работы таймера
	TCCR1B = (3 << CS10); //предделитель на 64, 8000000/64/1000 = 1мс = 125 тиков таймера
	TIMSK1 = (1 << OCIE1A); //прерывание таймера по совпадению с OCR1A
	TCNT1 = 0;		//сброс таймера
	OCR1A = 0;

	//printf_P(PSTR("Servo: %u\n"), MaxServo);
}

void ServoSetPos(uint8_t servoNum, uint8_t pos) { //задать положение сервопривода
	if (pos > Tick_1ms)
		servo[servoNum].position = Tick_1ms;
	else
		servo[servoNum].position = pos; //задаем позицию
	ServoSort();
	servoNeedUpdate = 1;
}

void SetAllServoMiddlePos(void) { //выставляем все сервоприводы в среднее положение
	for ( uint8_t i = 0; i < MaxServo ; i++ )
	{
		servo[i].position = Tick_1ms >> 1;
	}
	ServoSort();
	servoNeedUpdate = 1;
}

void OffLineAction(void)
{
	//стоп моторы
	MOTOR0_TIMER_REG = 0;
	MOTOR1_TIMER_REG = 0;

	SetAllServoMiddlePos(); //все сервоприводы в среднее положение
}


int main(void)
{
	// поддяжка TWI Pull UP
	PORTC |= (1 << PC4) | (1 << PC5);

	LED_DDR |= (1<<LED_BIT); //информационный светодиод

	BUZZER_DDR |= (1<<BUZZER_BIT); //пищалка 2400 гц, порт на выход

	//таймер 2, режим CTC, подключен выход
	TCCR2A = (0 << COM2B0) | (1 << WGM21);
	//предделитель 8 8000000/8 = 1000000Гц
	TCCR2B = (2 << CS20);
	TCNT2 = 0;
	OCR2A = 208; //4800Гц /2 =2400Гц частота пищалки


	//моторы
	MOTOR0_PWM_DDR |= (1 << MOTOR0_PWM_BIT);
	MOTOR0_INA_DDR |= (1 << MOTOR0_INA_BIT);
	MOTOR0_INB_DDR |= (1 << MOTOR0_INB_BIT);

	MOTOR1_PWM_DDR |= (1 << MOTOR1_PWM_BIT);
	MOTOR1_INA_DDR |= (1 << MOTOR1_INA_BIT);
	MOTOR1_INB_DDR |= (1 << MOTOR1_INB_BIT);

	//таймер 0, шим на моторы FastPWM
	TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
	//предделитель 64 8000000/256/8 = 3906Гц
	TCCR0B = (2 << CS00);

	//серво
	SERVO_DDR |= 0b00001111;	// Все выходы серво назначаем как выход
	ServoInit(); //инициализация структур и прерываний для формирования сигналов управления сервоприводами
	SetAllServoMiddlePos(); //все сервоприводы в среднее положение

	// Initial I2C Slave
	TWAR = (I2C_SLAVE_ADDR << 1) & 0xFE;    // Set I2C Address, Ignore I2C General Address 0x00
	TWDR = 0x00;            // Default Initial Value

	// Start Slave Listening: Clear TWINT Flag, Enable ACK, Enable TWI, TWI Interrupt Enable
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);

	// Initial Variable Used
	regAddr = 0;
	regData = 0;

	buzzerTime = 3; //бибикаем на старте
	TCCR2A |= (1 << COM2B0); //включаем ногу динамика

	sei();

	while(1)
	{
		offLineCount++; //увеличиваем счетчик оффлайна
		if (offLineCount > 30) //если досчитал до 30 (3 секунды), значит сыстема верхнего уровня "отвалилась"
		{
			offLineCount = 0;
			if (onLine)
			{
				onLine = 0; //система в оффлайне
				OffLineAction(); //выполняем действия при оффлайне системы
			}

		}

		if (onLine)
			LED_PORT ^= (1<<LED_BIT); //инвертируем светодиод
		else
		{
			offLineLedCount++; //счетчик пропусков для режима оффлайн, чтобы светодиод мигал медленнее
			if (offLineLedCount > 10)
			{
				offLineLedCount = 0;
				LED_PORT ^= (1<<LED_BIT); //инвертируем светодиод
			}
		}

		if (buzzerTime)
			buzzerTime--;
		else
			TCCR2A &= ~(1 << COM2B0); //выключаем ногу динамика


		_delay_ms(100);
	}
}
