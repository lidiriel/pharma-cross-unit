#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ModbusRTUSlave.h>

#define DEBUG false
#define Serial if(DEBUG)Serial

#define CROSS_A // define CROSS_A or CROSS_B

#define pin_rs485_de 2
#define pin_rx 3
#define pin_tx 4
SoftwareSerial my_serial(pin_rx, pin_tx);
#define modbus_serial my_serial
#define modbus_baud 38400
#define modbus_config SERIAL_8N1

#ifdef CROSS_A
#define modbus_unit_id 1
#define led_shift 0
#else
#define modbus_unit_id 2
#define led_shift 8
#endif

const int pin_led_user = LED_BUILTIN;
// output pin for controlling LEDs (4 blues, 4 green)
const int pin_leds[8] = {5, 6, 7, 8, 9, 10, 11, 12};

const int pin_switch = 13;

bool ledOn = false;

#define register_led 0 // register used to change the state of the leds
uint8_t led_state = 0;
uint8_t new_state = 0;

const uint8_t numHoldingRegisters = 1;
ModbusRTUSlave modbus(modbus_serial, pin_rs485_de);
uint16_t holdingRegisters[numHoldingRegisters];

unsigned long lastUpdate;
unsigned long updateInterval = 250;


void setup()
{
	pinMode(pin_switch, INPUT_PULLUP);
	Serial.begin(9600);
	Serial.println("start setup");
	// configure and check outputs LEDs
	for(int i=0; i<8; i++){
		int pin = pin_leds[i];
		pinMode(pin, OUTPUT);
		digitalWrite(pin, HIGH);
		delay(250);
		digitalWrite(pin, LOW);
		delay(250);
	}
	modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
	modbus_serial.begin(modbus_baud);
	modbus.begin(modbus_unit_id, modbus_baud, modbus_config);
	// Timer0 is already used for millis() - we'll just interrupt somewhere
	// in the middle and call the "Compare A" function below
	OCR0A = 0xAF;
	TIMSK0 |= _BV(OCIE0A);
	Serial.println("end setup");
}

// interruption shutdown led
SIGNAL(TIMER0_COMPA_vect)
{
	if(ledOn){
		unsigned long currentMillis = millis();
		if((currentMillis - lastUpdate) > updateInterval)  // time to update
		{
			update_leds(0);
			ledOn = false;
		}
	}
}

void update_leds(uint8_t new_state) {
	if (new_state != led_state) {
		led_state = new_state;
		for (int i = 0; i < 8; i++) {
			uint8_t value = (new_state & 1) ? HIGH : LOW;
			digitalWrite(pin_leds[i], value);
			new_state = new_state >> 1;
		}
	}
	ledOn = true;
}

int count_set_bits(unsigned int num)
{
	int count = 0;
	while (num) {
		count += num & 1; // Add the least significant bit
		num >>= 1; // Right shift the number
	}
	return count;
}

void loop()
{
	if(digitalRead(pin_switch) == LOW){
		// random mode
		delay(500);
		int count = 0;
		do{
			new_state = random(0, 255);
			count = count_set_bits(new_state);
		}while(count>5);
		update_leds(new_state);
	}else{
		if(modbus.poll()){ // check message on modbus
			uint16_t reg = holdingRegisters[register_led];
			new_state = (uint8_t) (reg >> led_shift);
			update_leds(new_state);
			lastUpdate = millis();
		}
	}
}
