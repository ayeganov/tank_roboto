/*
*  Originally Written by Matthew Richardson
*  matthewrichardson37<at>gmail.com  *  http://mattallen37.wordpress.com/
*
*  Updated by Jaikrishna T S  *  t.s.jaikrishna<at>gmail.com
*
*  Updated by John Cole * Dexter Industries.   
*
*  Initial date: June 4, 2013
*  Last updated: Feb 17, 2014
*
* These files have been made available online through a Creative Commons Attribution-ShareAlike 3.0  license.
* (http://creativecommons.org/licenses/by-sa/3.0/)
*
*  This is a library of functions for the RPi to communicate with the BrickPi.
*/
/*
##################################################################################################################
* Debugging:
* - NOTE THAT DEBUGGING ERROR MESSAGES ARE TURNED OFF BY DEFAULT.  To debug,
*   just take the comment out of Line 38.
* 
* If you #define DEBUG in the program, the BrickPi.h drivers will print debug
* messages to the terminal. One common message is "BrickPiRx error: -2", in
* function BrickPiUpdateValues(). This is caused by an error in the
* communication with one of the microcontrollers on the BrickPi. When this
* happens, the drivers automatically re-try the communication several times
* before the function gives up and returns -1 (unsuccessful) to the
* user-program.

* Function BrickPiUpdateValues() will either return 0 (success), or -1 (error
* that could not automatically be resolved, even after re-trying several
* times). We have rarely had BrickPiUpdateValues() retry more than once before
* the communication was successful.  A known cause for "BrickPiRx error: -2" is
* the RPi splitting the UART message. Sometimes the RPi will send e.g. 3 bytes,
* wait a while, and then send 4 more, when it should have just sent 7 at once.
* During the pause between the packs of bytes, the BrickPi microcontrollers
* will think the transmission is complete, realize the message doesn't make
* sense, throw it away, and not return a message to the RPi. The RPi will then
* fail to receive a message in the specified amount of time, timeout, and then
* retry the communication.
*
* Some more in-depth debugging information:
*    BrickPiRx error: -4; // Buffer is too small to be anything. Maybe you only got one byte back?
*    BrickPiRx error: -6; // Buffer incomplete.  Again, the buffer is smaller than expected.
*    BrickPiRx error: -5;  // Checksum is wrong.  The checksum position was
*                             calculated wrong, something was dropped in the serial communication or
*                             didn't make it into the buffer.
*
*/

#ifndef __BrickPi_h_
#define __BrickPi_h_

// #define DEBUG        // Define this if you want debugging messages on.

#include <stdio.h>
#include <unistd.h>         //Used for UART
#include <fcntl.h>          //Used for UART
#include <termios.h>        //Used for UART

static const int PORT_A = 0;
static const int PORT_B = 1;
static const int PORT_C = 2;
static const int PORT_D = 3;

static const int PORT_1 = 0;
static const int PORT_2 = 1;
static const int PORT_3 = 2;
static const int PORT_4 = 3;

static const int MASK_D0_M = 0x01;
static const int MASK_D1_M = 0x02;
static const int MASK_9V = 0x04;
static const int MASK_D0_S = 0x08;
static const int MASK_D1_S = 0x10;

static const int BYTE_MSG_TYPE = 0; // MSG_TYPE is the first byte.
static const int MSG_TYPE_CHANGE_ADDR = 1; // Change the UART address.
static const int MSG_TYPE_SENSOR_TYPE = 2; // Change/set the sensor type.
static const int MSG_TYPE_VALUES = 3; // Set the motor speed and direction, and return the sesnors and encoders.
static const int MSG_TYPE_E_STOP = 4; // Float motors immidately
static const int MSG_TYPE_TIMEOUT_SETTINGS = 5; // Set the timeout

// New UART address (MSG_TYPE_CHANGE_ADDR)
static const int BYTE_NEW_ADDRESS = 1;

// Sensor setup (MSG_TYPE_SENSOR_TYPE)
static const int BYTE_SENSOR_1_TYPE = 1;
static const int BYTE_SENSOR_2_TYPE = 2;

// Timeout setup (MSG_TYPE_TIMEOUT_SETTINGS)
static const int BYTE_TIMEOUT = 1;

static const int TYPE_MOTOR_PWM = 0;
static const int TYPE_MOTOR_SPEED = 1;
static const int TYPE_MOTOR_POSITION = 2;

static const int TYPE_SENSOR_RAW = 0; // - 31
static const int TYPE_SENSOR_LIGHT_OFF = 0;
static const int TYPE_SENSOR_LIGHT_ON = (MASK_D0_M | MASK_D0_S);
static const int TYPE_SENSOR_TOUCH = 32;
static const int TYPE_SENSOR_ULTRASONIC_CONT = 33;
static const int TYPE_SENSOR_ULTRASONIC_SS = 34;
static const int TYPE_SENSOR_RCX_LIGHT = 35; // tested minimally
static const int TYPE_SENSOR_COLOR_FULL = 36;
static const int TYPE_SENSOR_COLOR_RED = 37;
static const int TYPE_SENSOR_COLOR_GREEN = 38;
static const int TYPE_SENSOR_COLOR_BLUE = 39;
static const int TYPE_SENSOR_COLOR_NONE = 40;
static const int TYPE_SENSOR_I2C = 41;
static const int TYPE_SENSOR_I2C_9V = 42;


static const int TYPE_SENSOR_EV3_US_M0 = 43;
static const int TYPE_SENSOR_EV3_US_M1 = 44;
static const int TYPE_SENSOR_EV3_US_M2 = 45;
static const int TYPE_SENSOR_EV3_US_M3 = 46;
static const int TYPE_SENSOR_EV3_US_M4 = 47;
static const int TYPE_SENSOR_EV3_US_M5 = 48;
static const int TYPE_SENSOR_EV3_US_M6 = 49;

static const int TYPE_SENSOR_EV3_COLOR_M0 = 50;
static const int TYPE_SENSOR_EV3_COLOR_M1 = 51;
static const int TYPE_SENSOR_EV3_COLOR_M2 = 52;
static const int TYPE_SENSOR_EV3_COLOR_M3 = 53;
static const int TYPE_SENSOR_EV3_COLOR_M4 = 54;
static const int TYPE_SENSOR_EV3_COLOR_M5 = 55;

static const int TYPE_SENSOR_EV3_GYRO_M0 = 56;
static const int TYPE_SENSOR_EV3_GYRO_M1 = 57;
static const int TYPE_SENSOR_EV3_GYRO_M2 = 58;
static const int TYPE_SENSOR_EV3_GYRO_M3 = 59;
static const int TYPE_SENSOR_EV3_GYRO_M4 = 60;

static const int TYPE_SENSOR_EV3_INFRARED_M0 = 61;
static const int TYPE_SENSOR_EV3_INFRARED_M1 = 62;
static const int TYPE_SENSOR_EV3_INFRARED_M2 = 63;
static const int TYPE_SENSOR_EV3_INFRARED_M3 = 64;
static const int TYPE_SENSOR_EV3_INFRARED_M4 = 65;
static const int TYPE_SENSOR_EV3_INFRARED_M5 = 66;

static const int TYPE_SENSOR_EV3_TOUCH_0 = 67;

static const int BIT_I2C_MID = 0x01;  // Do one of those funny clock pulses between writing and reading. defined for each device.
static const int BIT_I2C_SAME = 0x02;  // The transmit data, and the number of bytes to read and write isn't going to change. defined for each device.

static const int INDEX_RED = 0;
static const int INDEX_GREEN = 1;
static const int INDEX_BLUE = 2;
static const int INDEX_BLANK = 3;

//US Fix starts
static const int US_I2C_SPEED = 10; //#tweak this value 
static const int US_I2C_IDX = 0;
static const int LEGO_US_I2C_ADDR = 0x02;
static const int LEGO_US_I2C_DATA_REG = 0x42;
//US Fix ends

//Setup USART Stream 0
static int uart0_filestream = -1;



extern unsigned long CurrentTickUs();

struct BrickPiStruct{
  unsigned char Address[2];     // Communication addresses
  unsigned long Timeout;     // Communication timeout (how long in ms since the last valid communication before floating the motors). 0 disables the timeout.

/*
  Motors
*/
  int           MotorSpeed     [4];     // Motor speeds, from -255 to 255
  unsigned char MotorEnable    [4];     // Motor enable/disable

/*
  Encoders
*/
  long          EncoderOffset  [4];     // Encoder offsets (not yet implemented)
  long          Encoder        [4];     // Encoder values

/*
  Sensors
*/
  long          Sensor         [4];     // Primary sensor values
  long          SensorArray    [4][4];  // For more sensor values for the sensor (e.g. for color sensor FULL mode).
  unsigned char SensorType     [4];     // Sensor types
  unsigned char SensorSettings [4][8];  // Sensor settings, used for specifying I2C settings.

/*
  I2C
*/
  unsigned char SensorI2CDevices[4];        // How many I2C devices are on each bus (1 - 8).
  unsigned char SensorI2CSpeed  [4];        // The I2C speed.
  unsigned char SensorI2CAddr   [4][8];     // The I2C address of each device on each bus.  
  unsigned char SensorI2CWrite  [4][8];     // How many bytes to write
  unsigned char SensorI2CRead   [4][8];     // How many bytes to read
  unsigned char SensorI2COut    [4][8][16]; // The I2C bytes to write
  unsigned char SensorI2CIn     [4][8][16]; // The I2C input buffers
};
static struct BrickPiStruct BrickPi;
BrickPiStruct& get_brick();


//Store the button's of the MINDSENSORS PSP Controller
struct button
{
    unsigned char   l1;
    unsigned char   l2;
    unsigned char   r1;
    unsigned char   r2;
    unsigned char   a;
    unsigned char   b;
    unsigned char   c;
    unsigned char   d;
    unsigned char   tri;
    unsigned char   sqr;
    unsigned char   cir;
    unsigned char   cro;
    unsigned char   ljb;  // joystick button state
    unsigned char   rjb;  // joystick button state

    int   ljx;   // analog value of joystick scaled from -127 to 127
    int   ljy;   // analog value of joystick scaled from -127 to 127
    int   rjx;   // analog value of joystick scaled from -127 to 127
    int   rjy;   // analog value of joystick scaled from -127 to 127
};
//Initialize all the buttons to 0 of the MINDSENSORS PSP controller
struct button init_psp (struct button b);

//Update all the buttons of the MINDSENSORS PSP controller
//For all buttons:
//  0:  Unpressed
//  1:  Pressed
//
//  Left and right joystick: -127 to 127 
static struct button upd(struct button b,int port);

//Show button values of the MINDSENSORS PSP controller
static void show_val(struct button b);

int BrickPiChangeAddress(unsigned char OldAddr, unsigned char NewAddr);

int BrickPiSetTimeout();


void AddBits(unsigned char byte_offset, unsigned char bit_offset, unsigned char bits, unsigned long value);
unsigned long GetBits(unsigned char byte_offset, unsigned char bit_offset, unsigned char bits);
unsigned char BitsNeeded(unsigned long value);
int BrickPiSetupSensors();
int BrickPiUpdateValues();
int BrickPiSetup();

void BrickPiTx(unsigned char dest, unsigned char ByteCount, unsigned char OutArray[]);
int BrickPiRx(unsigned char *InBytes, unsigned char *InArray, long timeout);

#endif
