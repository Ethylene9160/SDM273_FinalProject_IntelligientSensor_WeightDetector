#ifndef MY_DIGITAL_TUBE
#ifndef C_TEST
//#include<arduino.h>
#endif
#define MY_DIGITAL_TUBE 4

extern unsigned char tubeNums[];
class MyDigitalTube {
private:
	int latchPin, clockPin, dataPin;

public:

	MyDigitalTube(int latchPin, int colckPin, int dataPin);

	void light(int i);

	void begin();
};


#endif
