#include <elapsedMillis.h>
#include <RingBuf.h>
#include "modbus_rtu.h"
#include "crc16_ccitt.h"
//#include <DueFlashStorage.h>

//#define DEBUG (1)
#undef DEBUG

#define ATX_ON_DURATION             (100)  // ms
#define ATX_OFF_EXPIRATION          (60000) // ms
#define ATX_OFF_DURATION            (5000) // ms
#define DEFAULT_TRIGGER_DELAY       (100) // ms
#define DEFAULT_CAMERA_PIN_DURATION (10) // ms
#define DEFAULT_LIGHT_DURATION      (100) // ms
#define DEFAULT_THRESHOLD           (127)

//DueFlashStorage dueFlashStorage;

// Pin definition
int triggerPin = 25;
int powerStatusPin = 24;
int powerButtonPin = 6;
int lightPin = 5;
int cameraPin = 3;

int shutterCount = 0;
int shutterPeriod = 10; // ms

// State Diagram for ATX Powering
enum PowerOpState {
  WAITING,
  BEGIN_ON,
  STAY_ON,
  BEGIN_OFF,
  POLLING_OFF,
  FORCE_OFF,
  STAY_OFF
} powerOpState = WAITING;
int powerStatus = LOW;
elapsedMillis powerOpElapsed;

void processAtxPower(enum PowerOpState *opState, int pinStatus)
{
  switch (*opState) {
  case WAITING: break;
  case BEGIN_ON:
#ifdef DEBUG
    Serial.println("Power on:");
#endif
    digitalWrite(powerButtonPin, HIGH);
    powerOpElapsed = 0;
    *opState = STAY_ON;
    break;
  case STAY_ON:
    if (powerOpElapsed >= ATX_ON_DURATION) {
      digitalWrite(powerButtonPin, LOW);
      // TODO: reinitialize all the staff
      *opState = WAITING;
    }
    break;
  case BEGIN_OFF:
#ifdef DEBUG
    Serial.println("Power Off:");
#endif
    // TODO: Sending the message agreed to turn off the vision computer.
    sendPowerOffCmd();
    *opState = POLLING_OFF;
    powerOpElapsed = 0;
    break;
  case POLLING_OFF:
    if (pinStatus == LOW) {
      *opState = WAITING;
    } else {
      if (powerOpElapsed >= ATX_OFF_EXPIRATION)
        *opState = FORCE_OFF;
    }
    break;
  case FORCE_OFF:
    digitalWrite(powerButtonPin, HIGH);
    powerOpElapsed = 0;
    *opState = STAY_OFF;
    break;
  case STAY_OFF:
    if (powerOpElapsed >= ATX_OFF_DURATION) {
      digitalWrite(powerButtonPin, LOW);
      *opState = WAITING;
    }
    break;
  }
}

// Variables for handling Triggers
int triggerStatus = LOW;
//boolean triggerPending = false;
//elapsedMillis triggerElapsed;
uint16_t triggerDelay = DEFAULT_TRIGGER_DELAY;
uint16_t triggerCount = 0;
uint16_t triggerOverlapped = 0;
RingBuf<elapsedMillis, 10> triggerRingBuffer;

// Variables for Camera Pin 
boolean cameraPinPending = false;
elapsedMillis cameraPinElapsed;

// Variables for Lighting
enum LightMode {
  LIGHT_ON = 0,
  LIGHT_OFF,
  LIGHT_AUTO
} lightMode = LIGHT_AUTO;
boolean lightPending = false;
uint16_t lightDuration = DEFAULT_LIGHT_DURATION;
elapsedMillis lightElapsed;

// Variables for Image Processing
uint16_t imageThreshold = DEFAULT_THRESHOLD; // 0..255 a <- a+128 where -127<=a<=127
uint16_t imageRatio = 0;
uint16_t imageRatioCount = 0;
uint16_t imageProcessTime = 0;
elapsedMillis imageProcessElapsed;

// For internal RS232, Async-framing
// sflag:u8, cmd:u8, len:u8, payload:len*u8, crc:u16
#define S_FLAG 0x7e
#define CMD_I_AM_READY     0x00
#define CMD_ATX_OFF        0x01
#define CMD_IMAGE_THRESHOLD     0x02
#define CMD_TRIGGER        0x03
#define CMD_IMAGE_RATIO     0x04
#define CMD_TRIGGER_DELAY  0x05
#define CMD_LIGHT_ON       0x06
#define CMD_LIGHT_OFF      0x07
#define CMD_LIGHT_AUTO     0x08
#define CMD_LIGHT_DURATION 0x09
#define CMD_RESET_COUNT    0x0a
#define CMD_END            0x0b

const unsigned char powerOffCommand[5] = { S_FLAG, CMD_ATX_OFF, 0, 0xf0, 0x38 };
unsigned char thresCommand[7] = { S_FLAG, CMD_IMAGE_THRESHOLD, 2, 0x00, 0x00, 0x00, 0x00 };
const unsigned char triggerCommand[5] = { S_FLAG, CMD_TRIGGER, 0, 0x96, 0x5a };
const unsigned char resetCommand[5] = { S_FLAG, CMD_RESET_COUNT, 0, 0x2c, 0xc2 };

#define MAX_BUFFER_SIZE (64)
RingBuf<uint8_t, MAX_BUFFER_SIZE> txRingBuffer;
uint8_t rxCount = 0;
uint8_t rxBuffer[MAX_BUFFER_SIZE];

inline void sendPowerOffCmd(void) {
  txRingBuffer.push(powerOffCommand[0]);
  txRingBuffer.push(powerOffCommand[1]);
  txRingBuffer.push(powerOffCommand[2]);
  txRingBuffer.push(powerOffCommand[3]);
  txRingBuffer.push(powerOffCommand[4]);
#ifdef DEBUG
  Serial.println("sendPowerOffCmd:");
#endif
}

inline void sendTriggerCmd(void) {
  txRingBuffer.push(triggerCommand[0]);
  txRingBuffer.push(triggerCommand[1]);
  txRingBuffer.push(triggerCommand[2]);
  txRingBuffer.push(triggerCommand[3]);
  txRingBuffer.push(triggerCommand[4]);
#ifdef DEBUG
  //Serial.println("sendTriggerCmd:");
#endif
}

inline void sendThreshold(uint16_t threshold) {
  thresCommand[3] = highByte(threshold);
  thresCommand[4] = lowByte(threshold);
  uint16_t crc = crc16_ccitt(thresCommand, 5);
  txRingBuffer.push(thresCommand[0]);
  txRingBuffer.push(thresCommand[1]);
  txRingBuffer.push(thresCommand[2]);
  txRingBuffer.push(thresCommand[3]);
  txRingBuffer.push(thresCommand[4]);
  txRingBuffer.push(highByte(crc));
  txRingBuffer.push(lowByte(crc));
#ifdef DEBUG
  Serial.print("sendThresdhold:"); Serial.println(threshold, HEX);
#endif
}

inline void sendResetCmd(void) {
  txRingBuffer.push(resetCommand[0]);
  txRingBuffer.push(resetCommand[1]);
  txRingBuffer.push(resetCommand[2]);
  txRingBuffer.push(resetCommand[3]);
  txRingBuffer.push(resetCommand[4]);
#ifdef DEBUG
  Serial.println("sendResetCmd:");
#endif
}

inline void flushTxCmd(void) {

  uint8_t ch;
  for (int i = 0; i < 3; ++i) {
    if (txRingBuffer.pop(ch)) Serial2.write(ch);
    else break;
  }
}

//////////// Modbus
#define MODBUS_ID              (1)

#define REG_ATX_ON             (0)
#define REG_ATX_OFF            (1)
#define REG_COUNT_RESET        (2)
#define REG_IMAGE_THRESHOLD    (3)
#define REG_TRIGGER_DELAY      (4) // ms
#define REG_ATX_STATE          (5)
#define REG_IMAGE_RATIO        (6)
#define REG_COUNT              (7)
#define REG_TRIGGER_COUNT      (8)
#define REG_TRIGGER_OVERLAP    (9)
#define REG_TIME_OF_PROCESS    (10)
#define NUM_REG                (11)

uint16_t modbusRegisters[NUM_REG];
ModbusRTU<UARTClass> modbusServer(MODBUS_ID, &Serial1, 23);

// ISR for handling trigger pin
static elapsedMillis triggerMummy;
void processTrigger(void) {
  ++triggerCount;
  /*
  if (triggerPending) {
    ++triggerOverlapped;
  } else {
    triggerPending = true;
    triggerElapsed = 0;
  }
  */
  if (triggerRingBuffer.size() > 0) ++triggerOverlapped;
  triggerMummy = 0;
  triggerRingBuffer.push(triggerMummy);
}

void parseRxCommand(uint8_t *command, uint8_t len) {
  uint16_t crc = word(command[len-2], rxBuffer[len-1]);
  if (crc == crc16_ccitt(command, len-2)) {
    switch (command[1]) {
    case CMD_I_AM_READY:
      // TODO: Setup the param. of Vision PC via RS232 transferration.
      //sendThreshold(imageThreshold);
      Serial.println("Hi there!");
      break;
    case CMD_IMAGE_RATIO:
      imageRatio = command[3];
      ++imageRatioCount;
      imageProcessTime = imageProcessElapsed;
#ifdef DEBUG
      //Serial.print("imageRatio: "); Serial.print(imageRatio);
      //Serial.print(", imageRatioCount: "); Serial.println(imageRatioCount);
#endif
      break;
    case CMD_TRIGGER_DELAY:
      triggerDelay = command[3]<<8 | command[4];
      //dueFlashStorage.write(2, highByte(triggerDelay));
      //dueFlashStorage.write(3, lowByte(triggerDelay));
#ifdef DEBUG
      Serial.print("triggerDelay: "); Serial.println(triggerDelay);
#endif
      break;
    case CMD_LIGHT_ON:
      if (lightMode != LIGHT_ON) {
        digitalWrite(lightPin, HIGH);
        lightPending = false;
        lightMode = LIGHT_ON;
#ifdef DEBUG
        Serial.println("Light on");
#endif
      }
      break;
    case CMD_LIGHT_OFF:
      if (lightMode != LIGHT_OFF) {
        digitalWrite(lightPin, LOW);
        lightPending = false;
        lightMode = LIGHT_OFF;
#ifdef DEBUG
        Serial.println("Light off");
#endif
      }
      break;
    case CMD_LIGHT_AUTO:
      if (lightMode != LIGHT_AUTO) {
        digitalWrite(lightPin, LOW);
        lightPending = false;
        lightMode = LIGHT_AUTO;
#ifdef DEBUG
        Serial.println("Light auto");
#endif
      }
      break;
    case CMD_LIGHT_DURATION:
      lightDuration = command[3]<<8 | command[4];
      //dueFlashStorage.write(4, highByte(lightDuration));
      //dueFlashStorage.write(5, lowByte(lightDuration));
#ifdef DEBUG
      Serial.print("Light Duration: "); Serial.println(lightDuration);
#endif
      break;
    }
  }
}

void serialEvent2() {
  if (Serial2.available()) {
    uint8_t ch = Serial2.read();
    //Serial.write(ch);
    if (rxCount == 0) {
      if (ch == S_FLAG) rxBuffer[rxCount++] = S_FLAG;
    } else if (rxCount == 1) { // Command
      if (ch < CMD_END) rxBuffer[rxCount++] = ch;
      else rxCount = 0;
    } else {
      rxBuffer[rxCount++] = ch;      
      if (rxCount >= 5) { // Sflag + Cmd + Datalen(0) + hiCRC + loCRC
        uint8_t datalen = rxBuffer[2];
        if (datalen > 2) {
          rxCount = 0;
        } else if (rxCount == datalen+5) {
          parseRxCommand(rxBuffer, rxCount);
          rxCount = 0;
        }
      }
    }
  }
}

void setup() {
  // Serial I/O Setup
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Hello World!");

  modbusServer.begin(115200);

  Serial2.begin(115200);
  while (!Serial2);

  // GPIO Pin setup
  pinMode(triggerPin, INPUT); // INPUT_PULLUP);
  triggerStatus = digitalRead(triggerPin);
  attachInterrupt(digitalPinToInterrupt(triggerPin), processTrigger, FALLING);
  
  pinMode(powerStatusPin, INPUT);
  powerStatus = digitalRead(powerStatusPin);
  
  pinMode(powerButtonPin, OUTPUT);
  digitalWrite(powerButtonPin, LOW);

  pinMode(lightPin, OUTPUT);
  digitalWrite(lightPin, LOW);

  pinMode(cameraPin, OUTPUT);
  digitalWrite(cameraPin, LOW);

  //imageThreshold = dueFlashStorage.read(0) << 8 | dueFlashStorage.read(1);
  //triggerDelay = dueFlashStorage.read(2) << 8 | dueFlashStorage.read(3);
  //lightDuration = dueFlashStorage.read(4) << 8 | dueFlashStorage.read(5);

  Serial.print("triggerStatus: "); Serial.println(triggerStatus);
  Serial.print("powerStatus: "); Serial.println(powerStatus);
  Serial.print("imageThreshold: 0x"); Serial.println(imageThreshold, HEX);
  Serial.print("triggerDelay: "); Serial.println(triggerDelay, DEC);
  Serial.print("lightDuration: "); Serial.println(lightDuration, DEC);
}

#ifdef DEBUG
void serialEvent() {
  if (Serial.available()) {
    int ch = Serial.read();
    switch (ch) {
      case 'a': // Triggering
        ++triggerCount;
        /*
        triggerPending = true;
        triggerElapsed = 0;
        */
        triggerMummy = 0;
        triggerRingBuffer.push(triggerMummy);
        break;
      case 'b':
        ++imageRatioCount;
        break;
      case 'c':
        powerOpElapsed = BEGIN_OFF;
        break;
    }
  }
}
#endif

// the loop routine runs over and over again forever:
void loop() {
  // TODO: processing of Modbus RTU
  modbusRegisters[REG_ATX_ON] = 0;
  modbusRegisters[REG_ATX_OFF] = 0;
  modbusRegisters[REG_COUNT_RESET] = 0;
  modbusRegisters[REG_IMAGE_THRESHOLD] = imageThreshold;
  modbusRegisters[REG_TRIGGER_DELAY] = triggerDelay;
  modbusRegisters[REG_ATX_STATE] = powerStatus;
  modbusRegisters[REG_IMAGE_RATIO] = imageRatio;
  modbusRegisters[REG_COUNT] = imageRatioCount;
  modbusRegisters[REG_TRIGGER_COUNT] = triggerCount;
  modbusRegisters[REG_TRIGGER_OVERLAP] = triggerOverlapped;
  modbusRegisters[REG_TIME_OF_PROCESS] = imageProcessTime;

  int res = modbusServer.poll(modbusRegisters, NUM_REG);
  if (res > 4) {
    if (modbusRegisters[REG_ATX_ON] != 0) {
      if (powerOpState == WAITING && powerStatus == 0) {
        //Serial.println("ATX_ON");
        powerOpState = BEGIN_ON;
      }
      modbusRegisters[REG_ATX_ON] = 0;
    }
    
    if (modbusRegisters[REG_ATX_OFF] != 0) {
      if (powerOpState == WAITING && powerStatus == 1) {
        //Serial.println("ATX_OFF");
        powerOpState = BEGIN_OFF;
      }
      modbusRegisters[REG_ATX_OFF] = 0;
    }
    
    if (modbusRegisters[REG_IMAGE_THRESHOLD] != imageThreshold) {
#ifdef DEBUG      
      Serial.print("imageThreshold: 0x"); Serial.println(modbusRegisters[REG_IMAGE_THRESHOLD], HEX);
#endif
      // TODO: send the threshold of A-channel
      imageThreshold = modbusRegisters[REG_IMAGE_THRESHOLD];
      sendThreshold(imageThreshold);
      //dueFlashStorage.write(0, highByte(imageThreshold));
      //dueFlashStorage.write(1, lowByte(imageThreshold));
    }

    if (modbusRegisters[REG_TRIGGER_DELAY] != triggerDelay) {
#ifdef DEBUG
      Serial.print("TrigerDelay:"); Serial.println(modbusRegisters[REG_TRIGGER_DELAY]);
#endif
      triggerDelay = modbusRegisters[REG_TRIGGER_DELAY];
      //dueFlashStorage.write(2, highByte(triggerDelay));
      //dueFlashStorage.write(3, lowByte(triggerDelay));
    }
    
    if (modbusRegisters[REG_COUNT_RESET] != 0) {
#ifdef DEBUG
      Serial.println("Reset Count");
#endif
      sendResetCmd();
      imageRatioCount = 0;
      triggerCount = 0;
      triggerOverlapped = 0;
      rxCount = 0;
      modbusRegisters[REG_COUNT_RESET] = 0;
    }
  }

  // Triggering through internal serial lane.
  /*
  if (triggerPending) {
    if (triggerElapsed >= triggerDelay) {
      if (lightMode == LIGHT_AUTO) digitalWrite(lightPin, HIGH);
      //lightPending = true;
      ++lightPending;
      lightElapsed = 0;
      
      // TODO: send the trigger that should grep the image
      //Serial.println("Triggering");
      sendTriggerCmd();
      digitalWrite(cameraPin, HIGH);
      imageProcessElapsed = 0;

      triggerPending = false;
    }
  }
  */
  if (triggerRingBuffer.size() > 0) {
    if (triggerRingBuffer[0] >= triggerDelay) {
      triggerRingBuffer.pop(triggerMummy);
      if (lightMode == LIGHT_AUTO) digitalWrite(lightPin, HIGH);
      lightPending = true;
      lightElapsed = 0;
      // TODO: send the trigger that should grep the image
      //Serial.println("Triggering");
      sendTriggerCmd();
      digitalWrite(cameraPin, HIGH);
      cameraPinPending = true;
      cameraPinElapsed = 0;
      imageProcessElapsed = 0;
    }
  }

  if (cameraPinPending) {
    if (cameraPinElapsed >= DEFAULT_CAMERA_PIN_DURATION) {
      // TODO: in the case of overlapped trigger, handling to cameraPin.
      digitalWrite(cameraPin, LOW);
      cameraPinPending = false;
    }
  }
  
  if (lightPending > 0) {
    if (lightElapsed >= lightDuration) {
      if (lightMode == LIGHT_AUTO) digitalWrite(lightPin, LOW);
      lightPending = false;
    }
  }
  
  // read the input pin:
  int triggerCurrent = digitalRead(triggerPin);
#ifdef DEBUG
  if (triggerCurrent == HIGH && triggerStatus == LOW) {
    Serial.println("Sense");
  } else if (triggerCurrent == LOW && triggerStatus == HIGH) {
    Serial.println("Desense");
  }
#endif
  triggerStatus = triggerCurrent;

  int powerCurrent = digitalRead(powerStatusPin);
#ifdef DEBUG
  if (powerCurrent == HIGH && powerStatus == LOW) {
    Serial.println("Power on");
  } else if (powerCurrent == LOW && powerStatus == HIGH) {
    Serial.println("Power off");
  }
#endif
  powerStatus = powerCurrent;
  processAtxPower(&powerOpState, powerStatus);
  
  flushTxCmd();
}
