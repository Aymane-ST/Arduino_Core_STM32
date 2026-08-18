// USB Virtual Serial Receive Speed Benchmark
//
// This program receives data as rapidly as possible
// using Serial.readBytes() to read 500 bytes at a time.


// use one of these to define
// the USB virual serial name
//
#if defined(USE_TINYUSB)
  #include "TinyUSB.h"
#endif

#if defined(USE_TINYUSB)
  #define USBSERIAL SerialTinyUSB      // Arduino Leonardo, Teensy, Fubarino
#elif defined (USBCON) && defined(USBD_USE_CDC)
  #define USBSERIAL SerialUSB      // Arduino Due, Maple
#endif
//#define USBSERIAL SerialUSB   // Arduino Due, Maple


void setup() {
  USBSERIAL.begin(115200);
  //USBSERIAL.begin();  // for Maple
  pinMode(2, OUTPUT);  // frequency is kbytes/sec
}

byte pinstate=LOW;

void loop() {
  char buf[500];
  int count=0;
  int n;

  // receive 500 bytes
  for (count=0; count < 500; count++) {
    while (!USBSERIAL.available()) ;
    buf[count] = USBSERIAL.read();
  }

  // toggle pin 2, so the frequency is kbytes/sec
  if (pinstate == LOW) {
    digitalWrite(2, HIGH);
    pinstate = HIGH;
  } else {
    digitalWrite(2, LOW);
    pinstate = LOW;
  }
}
