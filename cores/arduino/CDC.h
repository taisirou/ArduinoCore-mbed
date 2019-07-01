#include "Arduino.h"

#ifdef __cplusplus

#ifndef __ARDUINO_CDC_IMPLEMENTATION__
#define __ARDUINO_CDC_IMPLEMENTATION__

#ifdef SERIAL_CDC
#include "USBSerial.h"

static void usbPortChanged(int baud, int bits, int parity, int stop) {
  if (baud == 1200) {
    _ontouch1200bps_();
  }
}

namespace arduino {

extern USBSerial _serial;

class CDC : public HardwareSerial {
	public:
		CDC() {}
		void begin(unsigned long) {
			_serial.connect();
  			_serial.attach(usbPortChanged);
		}
		void begin(unsigned long baudrate, uint16_t config) {
			begin(baudrate);
		}
		void end() {}
		int available(void) {
			return _serial.available();
		}
		int peek(void) {
			return 0;
		}
		int read(void) {
			return _serial._getc();
		}
		void flush(void) {}
		size_t write(uint8_t c) {
			_serial._putc(c);
		}
		//size_t write(const uint8_t*, size_t);
		using Print::write; // pull in write(str) and write(buf, size) from Print
		operator bool() {
			return _serial.connected();
		}		
};
}

extern arduino::CDC SerialUSB;

#endif
#endif
#endif
