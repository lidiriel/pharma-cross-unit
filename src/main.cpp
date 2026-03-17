#include <Arduino.h>
#include <SoftwareSerial.h>

#define DEBUG false
#define Serial if(DEBUG)Serial

#define CROSS_B // define CROSS_A or CROSS_B

#define pin_rs485_de 2
#define pin_rx 3
#define pin_tx 4
SoftwareSerial rs485Serial(pin_rx, pin_tx);

// baud rate 9600 19200 38400 57600 76800 115200
#define com_baud 115200

#ifdef CROSS_A
#define unit_id 1
#define led_shift 0
#else
#define unit_id 2
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

unsigned long lastUpdate;
unsigned long updateInterval = 500;

byte crc8(byte *data, byte len) {
  byte crc = 0x00;
  for (byte i = 0; i < len; i++) {
    crc ^= data[i];
    for (byte j = 0; j < 8; j++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }
  return crc;
}

void setup()
{
	pinMode(pin_switch, INPUT_PULLUP);
	pinMode(pin_rs485_de, OUTPUT);
	digitalWrite(pin_rs485_de, LOW);
	Serial.begin(9600);
	Serial.println("start setup");
	rs485Serial.begin(com_baud);
	// configure and check outputs LEDs
	for(int i=0; i<8; i++){
		int pin = pin_leds[i];
		pinMode(pin, OUTPUT);
		digitalWrite(pin, HIGH);
		delay(250);
		digitalWrite(pin, LOW);
		delay(250);
	}
	// Timer0 is already used for millis() - we'll just interrupt somewhere
	// in the middle and call the "Compare A" function below
	OCR0A = 0xAF;
	TIMSK0 |= _BV(OCIE0A);
	Serial.println("end setup");
}

void update_leds(uint8_t new_state) {
	if (new_state != led_state) {
		led_state = new_state;
		for (int i = 0; i < 8; i++) {
			uint8_t value = (new_state & 0x01) == 1 ? HIGH : LOW;
			digitalWrite(pin_leds[i], value);
			new_state = new_state >> 1;
		}
	}
	ledOn = true;
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
	static enum {WAIT_START, BYTE1, BYTE2, CRC, WAIT_END} state = WAIT_START;
	static byte data1, data2, received_crc;
	if(digitalRead(pin_switch) == LOW){
		// random mode
		delay(500);
		int count = 0;
		do{
			new_state = random(0, 255);
			count = count_set_bits(new_state);
		}while(count>5);
		update_leds(new_state);
		lastUpdate = millis();
	}else{
		if (rs485Serial.available()) {
		    byte b = rs485Serial.read();

		    switch (state) {
		      case WAIT_START:
		        if (b == 0xAA) state = BYTE1;
		        break;
		      case BYTE1:
		        data1 = b;
		        state = BYTE2;
		        break;
		      case BYTE2:
		        data2 = b;
		        state = CRC;
		        break;
		      case CRC:
		    	received_crc = b;
		    	state = WAIT_END;
		    	break;
		      case WAIT_END:
		        if (b == 0x55) {
		        	byte buf[2] = {data1, data2};
		        	byte calc_crc = crc8(buf, 2);
		        	if (calc_crc == received_crc) {
						// data ok
						if(unit_id == 1){
							new_state = (uint8_t) data1;
						}else{
							new_state = (uint8_t) data2;
						}
						update_leds(new_state);
						lastUpdate = millis();
		        	} else {
		        		Serial.println("CRC invalid");
		        	}
		        } else {
		        	Serial.println("stop byte invalid");
		        }
		        state = WAIT_START;
		        break;
		    }
		}
	}
}
