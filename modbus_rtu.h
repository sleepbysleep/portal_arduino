/**
 * @file 	ModbusRtu.h
 * @version     1.21
 * @date        2016.02.21
 * @author 	Samuel Marco i Armengol
 * @contact     sammarcoarmengol@gmail.com
 * @contribution Helium6072
 *
 * @description
 *  Arduino library for communicating with Modbus devices
 *  over RS232/USB/485 via RTU protocol.
 *
 *  Further information:
 *  http://modbus.org/
 *  http://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
 *
 * @license
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; version
 *  2.1 of the License.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * @defgroup setup Modbus Object Instantiation/Initialization
 * @defgroup loop Modbus Object Management
 * @defgroup buffer Modbus Buffer Management
 * @defgroup discrete Modbus Function Codes for Discrete Coils/Inputs
 * @defgroup register Modbus Function Codes for Holding/Input Registers
 *
 */
#ifndef __MODBUS_RTU_H__
#define __MODBUS_RTU_H__

#include <inttypes.h>
#include <Arduino.h>
#include <Print.h>
//#include <SoftwareSerial.h>

#define T35         (5)
#define MAX_BUFFER  (64)	//!< maximum size for the communication buffer in bytes

/**
 * @struct modbus_t
 * @brief
 * Master query structure:
 * This includes all the necessary fields to make the Master generate a Modbus query.
 * A Master may keep several of these structures and send them cyclically or
 * use them according to program needs.
 */
typedef struct {
  uint8_t u8id;          /*!< Slave address between 1 and 247. 0 means broadcast */
  uint8_t u8fct;         /*!< Function code: 1, 2, 3, 4, 5, 6, 15 or 16 */
  uint16_t u16RegAdd;    /*!< Address of the first register to access at slave/s */
  uint16_t u16CoilsNo;   /*!< Number of coils or registers to access */
  uint16_t *au16reg;     /*!< Pointer to memory image in master */
} modbus_t;

enum {
  RESPONSE_SIZE = 6,
  EXCEPTION_SIZE = 3,
  CHECKSUM_SIZE = 2
};

/**
 * @enum MESSAGE
 * @brief
 * Indexes to telegram frame positions
 */
enum MESSAGE {
  ID = 0, //!< ID field
  FUNC, //!< Function code position
  ADD_HI, //!< Address high byte
  ADD_LO, //!< Address low byte
  NB_HI, //!< Number of coils or registers high byte
  NB_LO, //!< Number of coils or registers low byte
  BYTE_CNT  //!< byte counter
};

/**
 * @enum MB_FC
 * @brief
 * Modbus function codes summary.
 * These are the implement function codes either for Master or for Slave.
 *
 * @see also fctsupported
 * @see also modbus_t
 */
enum MB_FC {
  MB_FC_NONE                     = 0,   /*!< null operator */
  MB_FC_READ_COILS               = 1,	/*!< FCT=1 -> read coils or digital outputs */
  MB_FC_READ_DISCRETE_INPUT      = 2,	/*!< FCT=2 -> read digital inputs */
  MB_FC_READ_REGISTERS           = 3,	/*!< FCT=3 -> read registers or analog outputs */
  MB_FC_READ_INPUT_REGISTER      = 4,	/*!< FCT=4 -> read analog inputs */
  MB_FC_WRITE_COIL               = 5,	/*!< FCT=5 -> write single coil or output */
  MB_FC_WRITE_REGISTER           = 6,	/*!< FCT=6 -> write single register */
  MB_FC_WRITE_MULTIPLE_COILS     = 15,	/*!< FCT=15 -> write multiple coils or outputs */
  MB_FC_WRITE_MULTIPLE_REGISTERS = 16	/*!< FCT=16 -> write multiple registers */
};

enum COM_STATES {
  COM_IDLE                     = 0,
  COM_WAITING                  = 1
};

enum ERR_LIST {
  ERR_NOT_MASTER                = -1,
  ERR_POLLING                   = -2,
  ERR_BUFF_OVERFLOW             = -3,
  ERR_BAD_CRC                   = -4,
  ERR_EXCEPTION                 = -5
};

enum {
  NO_REPLY = 255,
  EXC_FUNC_CODE = 1,
  EXC_ADDR_RANGE = 2,
  EXC_REGS_QUANT = 3,
  EXC_EXECUTE = 4
};

const unsigned char fctsupported[] = {
  MB_FC_READ_COILS,
  MB_FC_READ_DISCRETE_INPUT,
  MB_FC_READ_REGISTERS,
  MB_FC_READ_INPUT_REGISTER,
  MB_FC_WRITE_COIL,
  MB_FC_WRITE_REGISTER,
  MB_FC_WRITE_MULTIPLE_COILS,
  MB_FC_WRITE_MULTIPLE_REGISTERS
};

static constexpr uint16_t crc16_modbus_table[256] = {
    0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301,
    0x03c0, 0x0280, 0xc241, 0xc601, 0x06c0,
    0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481,
    0x0440, 0xcc01, 0x0cc0, 0x0d80, 0xcd41,
    0x0f00, 0xcfc1, 0xce81, 0x0e40, 0x0a00,
    0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0,
    0x0880, 0xc841, 0xd801, 0x18c0, 0x1980,
    0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
    0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01,
    0x1dc0, 0x1c80, 0xdc41, 0x1400, 0xd4c1,
    0xd581, 0x1540, 0xd701, 0x17c0, 0x1680,
    0xd641, 0xd201, 0x12c0, 0x1380, 0xd341,
    0x1100, 0xd1c1, 0xd081, 0x1040, 0xf001,
    0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1,
    0xf281, 0x3240, 0x3600, 0xf6c1, 0xf781,
    0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
    0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01,
    0x3fc0, 0x3e80, 0xfe41, 0xfa01, 0x3ac0,
    0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881,
    0x3840, 0x2800, 0xe8c1, 0xe981, 0x2940,
    0xeb01, 0x2bc0, 0x2a80, 0xea41, 0xee01,
    0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1,
    0xec81, 0x2c40, 0xe401, 0x24c0, 0x2580,
    0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
    0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101,
    0x21c0, 0x2080, 0xe041, 0xa001, 0x60c0,
    0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281,
    0x6240, 0x6600, 0xa6c1, 0xa781, 0x6740,
    0xa501, 0x65c0, 0x6480, 0xa441, 0x6c00,
    0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0,
    0x6e80, 0xae41, 0xaa01, 0x6ac0, 0x6b80,
    0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
    0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01,
    0x7bc0, 0x7a80, 0xba41, 0xbe01, 0x7ec0,
    0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81,
    0x7c40, 0xb401, 0x74c0, 0x7580, 0xb541,
    0x7700, 0xb7c1, 0xb681, 0x7640, 0x7200,
    0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0,
    0x7080, 0xb041, 0x5000, 0x90c1, 0x9181,
    0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
    0x9601, 0x56c0, 0x5780, 0x9741, 0x5500,
    0x95c1, 0x9481, 0x5440, 0x9c01, 0x5cc0,
    0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81,
    0x5e40, 0x5a00, 0x9ac1, 0x9b81, 0x5b40,
    0x9901, 0x59c0, 0x5880, 0x9841, 0x8801,
    0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1,
    0x8a81, 0x4a40, 0x4e00, 0x8ec1, 0x8f81,
    0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
    0x4400, 0x84c1, 0x8581, 0x4540, 0x8701,
    0x47c0, 0x4680, 0x8641, 0x8201, 0x42c0,
    0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040 };

template <typename serialport_t>
class ModbusRTU {
 private:
  serialport_t *serialPort = NULL;
  
  uint8_t modbusId; //!< 0=master, 1..247=slave number
  int txEnablePin; //!< flow control pin: 0=USB or RS-232 mode, >0=RS-485 mode
  uint16_t timeOut;
  uint32_t startTime, endTime;
  uint16_t incomingCount, outgoingCount, errorCount;
  uint8_t lastRecord, bufferSize;

  uint8_t currentState, lastError;
  uint8_t byteBuffer[MAX_BUFFER];

  uint16_t *registers;
  uint8_t registerCount;

  void init(uint8_t id, int tx_en_pin) {
    this->modbusId = id;
    this->txEnablePin = tx_en_pin;
    this->timeOut = 1000;
    this->incomingCount = this->outgoingCount = this->errorCount = 0;
    this->lastRecord = this->bufferSize = 0;

    if (tx_en_pin >= 0) {
      pinMode(tx_en_pin, OUTPUT);
      digitalWrite(tx_en_pin, LOW);
    }
  }

  /**
   * @brief
   * This method moves Serial buffer data to the Modbus au8Buffer.
   *
   * @return buffer size if OK, ERR_BUFF_OVERFLOW if u8BufferSize >= MAX_BUFFER
   * @ingroup buffer
   */
  int8_t getRxBuffer() {
    boolean bBufferOverflow = false;

    if (this->txEnablePin >= 0) digitalWrite(this->txEnablePin, LOW);

    this->bufferSize = 0;
    while (this->serialPort->available()) {
      this->byteBuffer[this->bufferSize++] = this->serialPort->read();
      if (this->bufferSize >= MAX_BUFFER) bBufferOverflow = true;
    }
      
    ++this->incomingCount;

    if (bBufferOverflow) {
      ++this->errorCount;
      return ERR_BUFF_OVERFLOW;
    }
    
    return this->bufferSize;
  }
  
  /**
   * @brief
   * This method transmits au8Buffer to Serial line.
   * Only if u8txenpin != 0, there is a flow handling in order to keep
   * the RS485 transceiver in output state as long as the message is being sent.
   * This is done with UCSRxA register.
   * The CRC is appended to the buffer before starting to send it.
   *
   * @param nothing
   * @return nothing
   * @ingroup buffer
   */
  void sendTxBuffer() {
    //uint8_t i = 0; //unused
    // append CRC to message
    uint16_t u16crc = calcCRC(this->bufferSize);
    this->byteBuffer[this->bufferSize++] = u16crc >> 8;
    this->byteBuffer[this->bufferSize++] = u16crc & 0x00ff;

    // set RS485 transceiver to transmit mode
    if (this->txEnablePin >= 0) {
      digitalWrite(this->txEnablePin, HIGH);
      delayMicroseconds(50);
    }

    // transfer buffer to serial line
    this->serialPort->write(this->byteBuffer, this->bufferSize);
    
    // keep RS485 transceiver in transmit mode as long as sending
    if (this->txEnablePin >= 0) {
      delayMicroseconds(50);
      digitalWrite(this->txEnablePin, LOW);
    }
    
    // transfer buffer to serial line
    while (this->serialPort->read() >= 0);
    /*
    for (int i = 0; i < this->bufferSize; ++i) {
      Serial2.print(this->byteBuffer[i], HEX);
      Serial2.print(", ");
    }
    */
    this->bufferSize = 0;

    // set time-out for master
    this->endTime = millis() + (unsigned long)this->timeOut;

    // increase message counter
    ++this->outgoingCount;
  }

  /**
   * @brief
   * This method calculates CRC
   *
   * @return uint16_t calculated CRC value for the message
   * @ingroup buffer
   */
#if 0
  uint16_t calcCRC(uint8_t u8length) {
    unsigned int temp, temp2, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < u8length; ++i) {
      temp = temp ^ this->byteBuffer[i];
      for (unsigned char j = 1; j <= 8; ++j) {
	flag = temp & 0x0001;
	temp >>=1;
	if (flag) temp ^= 0xA001;
      }
    }
    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;
    // the returned value is already swapped
    // crcLo byte is first & crcHi byte is last
    return temp;
  }
#else
  
  uint16_t calcCRC(uint8_t u8length) {
    uint8_t temp;
    uint16_t crc16 = 0xffff;

    for (unsigned char i = 0; i < u8length; ++i) {
      temp = this->byteBuffer[i] ^ crc16;
      crc16 >>= 8;
      crc16 ^= crc16_modbus_table[temp];
    }
    return (crc16 << 8) | (crc16 >> 8);
  }
#endif
  
  /**
   * @brief
   * This method validates slave incoming messages
   *
   * @return 0 if OK, EXCEPTION if anything fails
   * @ingroup buffer
   */
  uint8_t validateRequest() {
    // check message crc vs calculated crc
    uint16_t u16MsgCRC = ((this->byteBuffer[this->bufferSize - 2] << 8) |
			  this->byteBuffer[this->bufferSize - 1]); // combine the crc Low & High bytes
    
    if (calcCRC(this->bufferSize-2) != u16MsgCRC) {
      ++this->errorCount;
      return NO_REPLY;
    }

    // check fct code
    boolean isSupported = false;
    for (uint8_t i = 0; i< sizeof(fctsupported); ++i) {
      if (fctsupported[i] == this->byteBuffer[FUNC]) {
	isSupported = true;
	break;
      }
    }
    if (!isSupported) {
      ++this->errorCount;
      return EXC_FUNC_CODE;
    }

    // check start address & nb range
    uint16_t u16regs = 0;
    uint8_t u8regs;
    switch (this->byteBuffer[FUNC]) {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
    case MB_FC_WRITE_MULTIPLE_COILS:
      u16regs = word(this->byteBuffer[ADD_HI], this->byteBuffer[ADD_LO]) / 16;
      u16regs += word(this->byteBuffer[NB_HI], this->byteBuffer[NB_LO]) /16;
      u8regs = (uint8_t)u16regs;
      if (u8regs > this->registerCount) return EXC_ADDR_RANGE;
      break;
    case MB_FC_WRITE_COIL:
      u16regs = word(this->byteBuffer[ADD_HI], this->byteBuffer[ADD_LO]) / 16;
      u8regs = (uint8_t)u16regs;
      if (u8regs > this->registerCount) return EXC_ADDR_RANGE;
      break;
    case MB_FC_WRITE_REGISTER:
      u16regs = word(this->byteBuffer[ADD_HI], this->byteBuffer[ADD_LO]);
      u8regs = (uint8_t)u16regs;
      if (u8regs > this->registerCount) return EXC_ADDR_RANGE;
      break;
    case MB_FC_READ_REGISTERS:
    case MB_FC_READ_INPUT_REGISTER:
    case MB_FC_WRITE_MULTIPLE_REGISTERS:
      u16regs = word(this->byteBuffer[ADD_HI], this->byteBuffer[ADD_LO]);
      u16regs += word(this->byteBuffer[NB_HI], this->byteBuffer[NB_LO]);
      u8regs = (uint8_t)u16regs;
      if (u8regs > this->registerCount) return EXC_ADDR_RANGE;
      break;
    }
    return 0; // OK, no exception code thrown
  }

  /**
   * @brief
   * This method validates master incoming messages
   *
   * @return 0 if OK, EXCEPTION if anything fails
   * @ingroup buffer
   */
  uint8_t validateAnswer() {
    // check message crc vs calculated crc
    uint16_t u16MsgCRC = ((this->byteBuffer[this->bufferSize - 2] << 8) |
			  this->byteBuffer[this->bufferSize - 1]); // combine the crc Low & High bytes
    
    if (calcCRC(this->bufferSize-2) != u16MsgCRC) {
      ++this->errorCount;
      return NO_REPLY;
    }

    // check exception
    if ((this->byteBuffer[FUNC] & 0x80) != 0) {
      ++this->errorCount;
      return ERR_EXCEPTION;
    }

    // check fct code
    boolean isSupported = false;
    for (uint8_t i = 0; i< sizeof(fctsupported); ++i) {
      if (fctsupported[i] == this->byteBuffer[FUNC]) {
	isSupported = true;
	break;
      }
    }
    
    if (!isSupported) {
      ++this->errorCount;
      return EXC_FUNC_CODE;
    }

    return 0; // OK, no exception code thrown
  }

  /**
   * @brief
   * This method builds an exception message
   *
   * @ingroup buffer
   */
  void buildException(uint8_t u8exception) {
    uint8_t u8func = this->byteBuffer[FUNC];  // get the original FUNC code

    this->byteBuffer[ID] = this->modbusId;
    this->byteBuffer[FUNC] = u8func + 0x80;
    this->byteBuffer[2] = u8exception;
    this->bufferSize = EXCEPTION_SIZE;
  }

  /**
   * This method processes functions 1 & 2 (for master)
   * This method puts the slave answer into master data buffer
   *
   * @ingroup register
   * TODO: finish its implementation
   */
  void get_FC1() {
    //uint8_t u8byte, i;
    //u8byte = 0;

    //  for (i=0; i< au8Buffer[ 2 ] /2; i++) {
    //    au16regs[ i ] = word(
    //    au8Buffer[ u8byte ],
    //    au8Buffer[ u8byte +1 ]);
    //    u8byte += 2;
    //  }
  }

  /**
   * This method processes functions 3 & 4 (for master)
   * This method puts the slave answer into master data buffer
   *
   * @ingroup register
   */
  void get_FC3() {
    uint8_t u8byte, i;
    u8byte = 3;

    for (i = 0; i< this->byteBuffer[2]/2; ++i) {
      this->registers[i] = word(this->byteBuffer[u8byte], this->byteBuffer[u8byte + 1]);
      u8byte += 2;
    }
  }

  /**
   * @brief
   * This method processes functions 1 & 2
   * This method reads a bit array and transfers it to the master
   *
   * @return u8BufferSize Response to master length
   * @ingroup discrete
   */
  int8_t process_FC1(uint16_t *regs, uint8_t u8size) {
    uint8_t u8currentRegister, u8currentBit, u8bytesno, u8bitsno;
    uint8_t u8CopyBufferSize;
    uint16_t u16currentCoil, u16coil;

    // get the first and last coil from the message
    uint16_t u16StartCoil = word(this->byteBuffer[ADD_HI], this->byteBuffer[ADD_LO]);
    uint16_t u16Coilno = word(this->byteBuffer[NB_HI], this->byteBuffer[NB_LO]);

    // put the number of bytes in the outcoming message
    u8bytesno = (uint8_t) (u16Coilno / 8);
    if (u16Coilno % 8 != 0) ++u8bytesno;
    this->byteBuffer[ADD_HI] = u8bytesno;
    this->bufferSize = ADD_LO;

    // read each coil from the register map and put its value inside the outcoming message
    u8bitsno = 0;

    for (u16currentCoil = 0; u16currentCoil < u16Coilno; ++u16currentCoil) {
      u16coil = u16StartCoil + u16currentCoil;
      u8currentRegister = (uint8_t)(u16coil / 16);
      u8currentBit = (uint8_t)(u16coil % 16);

      bitWrite(this->byteBuffer[this->bufferSize],
	       u8bitsno,
	       bitRead(regs[u8currentRegister], u8currentBit));
      ++u8bitsno;

      if (u8bitsno > 7) {
	u8bitsno = 0;
	++this->bufferSize;
      }
    }

    // send outcoming message
    if (u16Coilno % 8 != 0) ++this->bufferSize;
    u8CopyBufferSize = this->bufferSize +2;
    this->sendTxBuffer();
    return u8CopyBufferSize;
  }

  /**
   * @brief
   * This method processes functions 3 & 4
   * This method reads a word array and transfers it to the master
   *
   * @return u8BufferSize Response to master length
   * @ingroup register
   */
  int8_t process_FC3(uint16_t *regs, uint8_t u8size) {
    uint8_t u8StartAdd = word(this->byteBuffer[ADD_HI], this->byteBuffer[ADD_LO]);
    uint8_t u8regsno = word(this->byteBuffer[NB_HI], this->byteBuffer[NB_LO]);
    uint8_t u8CopyBufferSize;
    uint8_t i;

    this->byteBuffer[2] = u8regsno * 2;
    this->bufferSize = 3;

    for (i = u8StartAdd; i < u8StartAdd + u8regsno; ++i) {
      this->byteBuffer[this->bufferSize++] = highByte(regs[i]);
      this->byteBuffer[this->bufferSize++] = lowByte(regs[i]);
    }
    u8CopyBufferSize = this->bufferSize +2;
    this->sendTxBuffer();

    return u8CopyBufferSize;
  }

  /**
   * @brief
   * This method processes function 5
   * This method writes a value assigned by the master to a single bit
   *
   * @return u8BufferSize Response to master length
   * @ingroup discrete
   */
  int8_t process_FC5(uint16_t *regs, uint8_t u8size) {
    uint8_t u8currentRegister, u8currentBit;
    uint8_t u8CopyBufferSize;
    uint16_t u16coil = word(this->byteBuffer[ADD_HI], this->byteBuffer[ADD_LO]);

    // point to the register and its bit
    u8currentRegister = (uint8_t) (u16coil / 16);
    u8currentBit = (uint8_t) (u16coil % 16);

    // write to coil
    bitWrite(regs[u8currentRegister],
	     u8currentBit,
	     this->byteBuffer[NB_HI] == 0xff);


    // send answer to master
    this->bufferSize = 6;
    u8CopyBufferSize = this->bufferSize +2;
    this->sendTxBuffer();

    return u8CopyBufferSize;
  }

  /**
   * @brief
   * This method processes function 6
   * This method writes a value assigned by the master to a single word
   *
   * @return u8BufferSize Response to master length
   * @ingroup register
   */
  int8_t process_FC6(uint16_t *regs, uint8_t u8size) {
    uint8_t u8add = word(this->byteBuffer[ADD_HI], this->byteBuffer[ADD_LO]);
    uint8_t u8CopyBufferSize;
    uint16_t u16val = word(this->byteBuffer[NB_HI], this->byteBuffer[NB_LO]);

    regs[u8add] = u16val;

    // keep the same header
    this->bufferSize = RESPONSE_SIZE;

    u8CopyBufferSize = this->bufferSize +2;
    this->sendTxBuffer();

    return u8CopyBufferSize;
  }

  /**
   * @brief
   * This method processes function 15
   * This method writes a bit array assigned by the master
   *
   * @return u8BufferSize Response to master length
   * @ingroup discrete
   */
  int8_t process_FC15(uint16_t *regs, uint8_t u8size) {
    uint8_t u8currentRegister, u8currentBit, u8frameByte, u8bitsno;
    uint8_t u8CopyBufferSize;
    uint16_t u16currentCoil, u16coil;
    boolean bTemp;

    // get the first and last coil from the message
    uint16_t u16StartCoil = word(this->byteBuffer[ADD_HI], this->byteBuffer[ADD_LO]);
    uint16_t u16Coilno = word(this->byteBuffer[NB_HI], this->byteBuffer[NB_LO]);


    // read each coil from the register map and put its value inside the outcoming message
    u8bitsno = 0;
    u8frameByte = 7;
    for (u16currentCoil = 0; u16currentCoil < u16Coilno; ++u16currentCoil) {
      u16coil = u16StartCoil + u16currentCoil;
      u8currentRegister = (uint8_t) (u16coil / 16);
      u8currentBit = (uint8_t) (u16coil % 16);

      bTemp = bitRead(this->byteBuffer[u8frameByte], u8bitsno);
      bitWrite(regs[u8currentRegister], u8currentBit, bTemp);
      ++u8bitsno;

      if (u8bitsno > 7) {
	u8bitsno = 0;
	++u8frameByte;
      }
    }

    // send outcoming message
    // it's just a copy of the incomping frame until 6th byte
    this->bufferSize = 6;
    u8CopyBufferSize = this->bufferSize + 2;
    this->sendTxBuffer();
    
    return u8CopyBufferSize;
  }

  /**
   * @brief
   * This method processes function 16
   * This method writes a word array assigned by the master
   *
   * @return u8BufferSize Response to master length
   * @ingroup register
   */
  int8_t process_FC16(uint16_t *regs, uint8_t u8size) {
    //uint8_t u8func = this->byteBuffer[ FUNC ];  // get the original FUNC code
    uint8_t u8StartAdd = this->byteBuffer[ADD_HI] << 8 | this->byteBuffer[ADD_LO];
    uint8_t u8regsno = this->byteBuffer[NB_HI] << 8 | this->byteBuffer[NB_LO];
    uint8_t u8CopyBufferSize;
    uint8_t i;
    uint16_t temp;

    // build header
    this->byteBuffer[NB_HI] = 0;
    this->byteBuffer[NB_LO] = u8regsno;
    this->bufferSize = RESPONSE_SIZE;

    // write registers
    for (i = 0; i < u8regsno; ++i) {
      temp = word(this->byteBuffer[(BYTE_CNT + 1) + i * 2],
		  this->byteBuffer[(BYTE_CNT + 2) + i * 2]);

      regs[u8StartAdd + i] = temp;
    }
    u8CopyBufferSize = this->bufferSize + 2;
    this->sendTxBuffer();

    return u8CopyBufferSize;
  }

 public:
  ModbusRTU(uint8_t id, serialport_t *serial_port, int tx_en_pin=-1) {
    init(id, tx_en_pin);
    this->serialPort = serial_port;

    //while (this->serialPort->read() >= 0);
  }
  
  /**
   * Start the Modbus RTU client with the specified parameters
   *
   * @param baudrate Baud rate to use
   * @param config serial config. to use defaults to SERIAL_8N1
   *
   * Return 1 on success, 0 on failure
   */
  int begin(unsigned long baudrate) {
    this->serialPort->begin(baudrate);

    if (this->txEnablePin >= 0) {
      digitalWrite(this->txEnablePin, LOW);
    }

    while (this->serialPort->read() >= 0);

    this->lastRecord = 0;
    this->bufferSize = 0;
    this->incomingCount = 0;
    this->outgoingCount = 0;
    this->errorCount = 0;
    
    return 0;
  }
  
  /**
   * @brief
   * Method to write a new slave ID address
   *
   * @param 	u8id	new slave address between 1 and 247
   * @ingroup setup
   */
  void setId(uint8_t id) { if (id != 0 && id <= 247) this->modbusId = id; }
  
  /**
   * @brief
   * Method to read current slave ID address
   *
   * @return u8id	current slave address between 1 and 247
   * @ingroup setup
   */
  uint8_t getId() { return this->modbusId; }
  
  /**
   * @brief
   * Initialize time-out parameter
   *
   * Call once class has been instantiated, typically within setup().
   * The time-out timer is reset each time that there is a successful communication
   * between Master and Slave. It works for both.
   *
   * @param time-out value (ms)
   * @ingroup setup
   */
  void setTimeOut(uint16_t timeout) { this->timeOut = timeout; }

  //!<get communication watch-dog timer value  
  uint16_t getTimeOut() { return this->timeOut; }

  /**
   * @brief
   * Return communication Watchdog state.
   * It could be usefull to reset outputs if the watchdog is fired.
   *
   * @return TRUE if millis() > u32timeOut
   * @ingroup loop
   */
  boolean getTimeOutState() { return (millis() > this->endTime); }

  /**
   * @brief
   * Get input messages counter value
   * This can be useful to diagnose communication
   *
   * @return input messages counter
   * @ingroup buffer
   */
  uint16_t getInCnt() { return this->incomingCount; }

  /**
   * @brief
   * Get transmitted messages counter value
   * This can be useful to diagnose communication
   *
   * @return transmitted messages counter
   * @ingroup buffer
   */
  uint16_t getOutCnt() { return this->outgoingCount; }

  /**
   * @brief
   * Get errors counter value
   * This can be useful to diagnose communication
   *
   * @return errors counter
   * @ingroup buffer
   */
  uint16_t getErrCnt() { return this->errorCount; }

  /**
   * Get modbus master state
   *
   * @return = 0 IDLE, = 1 WAITING FOR ANSWER
   * @ingroup buffer
   */  
  uint8_t getState() { return this->currentState; }

  /**
   * Get the last error in the protocol processor
   *
   * @returnreturn   NO_REPLY = 255      Time-out
   * @return   EXC_FUNC_CODE = 1   Function code not available
   * @return   EXC_ADDR_RANGE = 2  Address beyond available space for Modbus registers
   * @return   EXC_REGS_QUANT = 3  Coils or registers number beyond the available space
   * @ingroup buffer
   */
  uint8_t getLastError() { return this->lastError; }

  /**
   * @brief
   * *** Only Modbus Master ***
   * Generate a query to an slave with a modbus_t telegram structure
   * The Master must be in COM_IDLE mode. After it, its state would be COM_WAITING.
   * This method has to be called only in loop() section.
   *
   * @see modbus_t
   * @param modbus_t  modbus telegram structure (id, fct, ...)
   * @ingroup loop
   * @todo finish function 15
   */
  int8_t query(modbus_t telegram) {
    uint8_t u8regsno, u8bytesno;
    
    if (this->modbusId != 0) return -2;
    if (this->currentState != COM_IDLE) return -1;

    if (telegram.u8id == 0 || telegram.u8id > 247) return -3;

    this->registers = telegram.au16reg;

    // telegram header
    this->byteBuffer[ID]         = telegram.u8id;
    this->byteBuffer[FUNC]       = telegram.u8fct;
    this->byteBuffer[ADD_HI]     = highByte(telegram.u16RegAdd);
    this->byteBuffer[ADD_LO]     = lowByte(telegram.u16RegAdd);

    switch (telegram.u8fct) {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
    case MB_FC_READ_REGISTERS:
    case MB_FC_READ_INPUT_REGISTER:
      this->byteBuffer[NB_HI]    = highByte(telegram.u16CoilsNo);
      this->byteBuffer[NB_LO]    = lowByte(telegram.u16CoilsNo);
      this->bufferSize = 6;
      break;
    case MB_FC_WRITE_COIL:
      this->byteBuffer[NB_HI]    = ((this->registers[0] > 0) ? 0xff : 0);
      this->byteBuffer[NB_LO]    = 0;
      this->bufferSize = 6;
      break;
    case MB_FC_WRITE_REGISTER:
      this->byteBuffer[NB_HI]    = highByte(this->registers[0]);
      this->byteBuffer[NB_LO]    = lowByte(this->registers[0]);
      this->bufferSize = 6;
      break;
    case MB_FC_WRITE_MULTIPLE_COILS: // TODO: implement "sending coils"
      u8regsno = telegram.u16CoilsNo / 16;
      u8bytesno = u8regsno * 2;
      if ((telegram.u16CoilsNo % 16) != 0) {
	++u8bytesno;
	++u8regsno;
      }

      this->byteBuffer[NB_HI]    = highByte(telegram.u16CoilsNo);
      this->byteBuffer[NB_LO]    = lowByte(telegram.u16CoilsNo);
      this->byteBuffer[NB_LO+1]  = u8bytesno;
      this->bufferSize = 7;

      u8regsno = u8bytesno = 0; // now auxiliary registers
      for (uint16_t i = 0; i < telegram.u16CoilsNo; ++i) {
      }
      break;

    case MB_FC_WRITE_MULTIPLE_REGISTERS:
      this->byteBuffer[NB_HI]    = highByte(telegram.u16CoilsNo);
      this->byteBuffer[NB_LO]    = lowByte(telegram.u16CoilsNo);
      this->byteBuffer[NB_LO+1]  = (uint8_t)(telegram.u16CoilsNo * 2);
      this->bufferSize = 7;

      for (uint16_t i = 0; i< telegram.u16CoilsNo; ++i) {
	this->byteBuffer[this->bufferSize++] = highByte(this->registers[i]);
	this->byteBuffer[this->bufferSize++] = lowByte(this->registers[i]);
      }
      break;
    }

    this->sendTxBuffer();
    this->currentState = COM_WAITING;
    return 0;
  }

  /**
   * @brief *** Only for Modbus Master ***
   * This method checks if there is any incoming answer if pending.
   * If there is no answer, it would change Master state to COM_IDLE.
   * This method must be called only at loop section.
   * Avoid any delay() function.
   *
   * Any incoming data would be redirected to au16regs pointer,
   * as defined in its modbus_t query telegram.
   *
   * @params	nothing
   * @return errors counter
   * @ingroup loop
   */
  int8_t poll() {
    // check if there is any incoming frame
    uint8_t u8current = 0;

    u8current = this->serialPort->available();

    if (millis() > this->endTime) {
      this->currentState = COM_IDLE;
      this->lastError = NO_REPLY;
      ++this->errorCount;
      return 0;
    }

    if (u8current == 0) return 0;

    // check T35 after frame end or still no frame end
    if (u8current != this->lastRecord) {
      this->lastRecord = u8current;
      this->startTime = millis() + T35;
      return 0;
    }
    if (millis() < this->startTime) return 0;

    // transfer Serial buffer frame to auBuffer
    this->lastRecord = 0;
    int8_t i8state = this->getRxBuffer();
    if (i8state < 7) {
      this->currentState = COM_IDLE;
      ++this->errorCount;
      return i8state;
    }

    // validate message: id, CRC, FCT, exception
    uint8_t u8exception = this->validateAnswer();
    if (u8exception != 0) {
      this->currentState = COM_IDLE;
      return u8exception;
    }

    // process answer
    switch (this->byteBuffer[FUNC]) {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
      // call get_FC1 to transfer the incoming message to au16regs buffer
      this->get_FC1( );
      break;
    case MB_FC_READ_INPUT_REGISTER:
    case MB_FC_READ_REGISTERS :
      // call get_FC3 to transfer the incoming message to au16regs buffer
      this->get_FC3( );
      break;
    case MB_FC_WRITE_COIL:
    case MB_FC_WRITE_REGISTER :
    case MB_FC_WRITE_MULTIPLE_COILS:
    case MB_FC_WRITE_MULTIPLE_REGISTERS :
      // nothing to do
      break;
    default:
      break;
    }
    this->currentState = COM_IDLE;
    return this->bufferSize;
  }

  /**
   * @brief
   * *** Only for Modbus Slave ***
   * This method checks if there is any incoming query
   * Afterwards, it would shoot a validation routine plus a register query
   * Avoid any delay() function !!!!
   * After a successful frame between the Master and the Slave, the time-out timer is reset.
   *
   * @param *regs  register table for communication exchange
   * @param u8size  size of the register table
   * @return 0 if no query, 1..4 if communication error, >4 if correct query processed
   * @ingroup loop
   */
  int8_t poll(uint16_t *regs, uint8_t u8size) {
    uint8_t u8current = 0;
    
    this->registers = regs;
    this->registerCount = u8size;

    u8current = this->serialPort->available();
    if (u8current == 0) return 0;
    //Serial.println("available data!");

    // check T35 after frame end or still no frame end
    if (u8current != this->lastRecord) {
      this->lastRecord = u8current;
      this->startTime = millis() + T35;
      return 0;
    }
    if (millis() < this->startTime) return 0;

    this->lastRecord = 0;
    int8_t i8state = this->getRxBuffer();
    this->lastError = i8state;
    if (i8state < 7) return i8state;

    // check slave id
    if (this->byteBuffer[ID] != this->modbusId) return 0;

    // validate message: CRC, FCT, address and size
    uint8_t u8exception = this->validateRequest();
    if (u8exception > 0) {
      if (u8exception != NO_REPLY) {
	this->buildException(u8exception);
	this->sendTxBuffer();
      }
      this->lastError = u8exception;
      return u8exception;
    }

    this->endTime = millis() + long(this->timeOut);
    this->lastError = 0;

    // process message
    switch (this->byteBuffer[FUNC]) {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
      //return this->process_FC1(regs, u8size);
      break;
    case MB_FC_READ_INPUT_REGISTER:
    case MB_FC_READ_REGISTERS :
      return this->process_FC3(regs, u8size);
      break;
    case MB_FC_WRITE_COIL:
      //return this->process_FC5(regs, u8size);
      break;
    case MB_FC_WRITE_REGISTER :
      return this->process_FC6(regs, u8size);
      break;
    case MB_FC_WRITE_MULTIPLE_COILS:
      //return this->process_FC15(regs, u8size);
      break;
    case MB_FC_WRITE_MULTIPLE_REGISTERS :
      return this->process_FC16(regs, u8size);
      break;
    default:
      break;
    }
    return i8state;
  }
  
  //void end(); //!<finish any communication and release serial communication port
};

#endif /* __MODBUS_ARDUINO_H__ */
