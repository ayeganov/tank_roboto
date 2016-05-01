#include <cstring>
#include <cerrno>
#include <iostream>
#include "BrickPi.h"

static long gotten_bits = 0;
//Setup USART Stream 0
static int uart0_filestream = -1;
static unsigned char Array[256];
static unsigned char BytesReceived;
static unsigned int Bit_Offset = 0;
static unsigned char Retried = 0; // For re-trying a failed update.

BrickPiStruct& get_brick() { return BrickPi; }
struct button init_psp (struct button b)
{
    b.l1 = 0;
    b.l2 = 0;
    b.r1 = 0;
    b.r2 = 0;
    b.a = 0;
    b.b = 0;
    b.c = 0;
    b.d = 0;
    b.tri = 0;
    b.sqr = 0;
    b.cir = 0;
    b.cro = 0;
    b.ljb = 0;
    b.rjb = 0;
    b.ljx = 0;
    b.ljy = 0;
    b.rjx = 0;
    b.rjy = 0;
    return b;
}


struct button upd(struct button b,int port)
{
    //Left and right joystick button press
    b.ljb = ~(BrickPi.SensorI2CIn[port][0][0] >> 1) & 0x01;
    b.rjb = ~(BrickPi.SensorI2CIn[port][0][0] >> 2) & 0x01;

    //For buttons a,b,c,d
    b.d = ~(BrickPi.SensorI2CIn[port][0][0] >> 4) & 0x01;
    b.c = ~(BrickPi.SensorI2CIn[port][0][0] >> 5) & 0x01;
    b.b = ~(BrickPi.SensorI2CIn[port][0][0] >> 6) & 0x01;
    b.a = ~(BrickPi.SensorI2CIn[port][0][0] >> 7) & 0x01;

    //For buttons l1,l2,r1,r2
    b.l2    = ~(BrickPi.SensorI2CIn[port][0][1] ) & 0x01;
    b.r2    = ~(BrickPi.SensorI2CIn[port][0][1] >> 1) & 0x01;
    b.l1    = ~(BrickPi.SensorI2CIn[port][0][1] >> 2) & 0x01;
    b.r1    = ~(BrickPi.SensorI2CIn[port][0][1] >> 3) & 0x01;

    //For buttons square,triangle,cross,circle
    b.tri    = ~(BrickPi.SensorI2CIn[port][0][1] >> 4) & 0x01;
    b.cir    = ~(BrickPi.SensorI2CIn[port][0][1] >> 5) & 0x01;
    b.cro = ~(BrickPi.SensorI2CIn[port][0][1] >> 6) & 0x01;
    b.sqr    = ~(BrickPi.SensorI2CIn[port][0][1] >> 7) & 0x01;

   //Left joystick x and y , -127 to 127
    b.ljx = -1*((BrickPi.SensorI2CIn[port][0][5]&0xff) - 128);
    b.ljy = (BrickPi.SensorI2CIn[port][0][4]&0xff) - 128;

    //Right joystick x and y , -127 to 127
    b.rjx = -1*((BrickPi.SensorI2CIn[port][0][3]&0xff) - 128);
    b.rjy = (BrickPi.SensorI2CIn[port][0][2]&0xff) - 128;
    return b;
}


void show_val(struct button b)
{
    printf("ljb rjb d c b a l2 r2 l1 r1 tri cir cro sqr\tljx\tljy\trjx\trjy\n");
    printf("%d ",b.ljb);
    printf("  %d ",b.rjb);
    printf("  %d ",b.d);
    printf("%d ",b.c);
    printf("%d ",b.b);
    printf("%d ",b.a);

    printf("%d ",b.l2);
    printf(" %d ",b.r2);
    printf(" %d ",b.l1);
    printf(" %d ",b.r1);
    printf(" %d ",b.tri);
    printf("  %d ",b.cir);
    printf("  %d ",b.cro);
    printf("  %d ",b.sqr);
    printf("\t%d ",b.ljx);
    printf("\t%d ",b.rjx);
    printf("\t%d ",b.ljy);
    printf("\t%d\n\n",b.rjy);
}


int BrickPiChangeAddress(unsigned char OldAddr, unsigned char NewAddr)
{
  Array[BYTE_MSG_TYPE] = MSG_TYPE_CHANGE_ADDR;
  Array[BYTE_NEW_ADDRESS] = NewAddr;
  BrickPiTx(OldAddr, 2, Array);

  if(BrickPiRx(&BytesReceived, Array, 5000))
    return -1;
  if(!(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_CHANGE_ADDR))
    return -1;

  return 0;
}


int BrickPiSetTimeout()
{
  unsigned char i = 0;
  while(i < 2){
    Array[BYTE_MSG_TYPE] = MSG_TYPE_TIMEOUT_SETTINGS;
    Array[BYTE_TIMEOUT      ] = ( BrickPi.Timeout             & 0xFF);
    Array[(BYTE_TIMEOUT + 1)] = ((BrickPi.Timeout / 256     ) & 0xFF);
    Array[(BYTE_TIMEOUT + 2)] = ((BrickPi.Timeout / 65536   ) & 0xFF);
    Array[(BYTE_TIMEOUT + 3)] = ((BrickPi.Timeout / 16777216) & 0xFF);
    BrickPiTx(BrickPi.Address[i], 5, Array);
    if(BrickPiRx(&BytesReceived, Array, 2500))
      return -1;
    if(!(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_TIMEOUT_SETTINGS))
      return -1;
    i++;
  }
  return 0;
}


void AddBits(unsigned char byte_offset, unsigned char bit_offset, unsigned char bits, unsigned long value)
{
  unsigned char i = 0;
  while(i < bits){
    if(value & 0x01){
      Array[(byte_offset + ((bit_offset + Bit_Offset + i) / 8))] |= (0x01 << ((bit_offset + Bit_Offset + i) % 8));
    }
    value /= 2;
    i++;
  }
  Bit_Offset += bits;
}


unsigned long GetBits(unsigned char byte_offset, unsigned char bit_offset, unsigned char bits)
{
  unsigned long Result = 0;
  char i = bits;
  while(i){
    Result *= 2;
    Result |= ((Array[(byte_offset + ((bit_offset + Bit_Offset + (i - 1)) / 8))] >> ((bit_offset + Bit_Offset + (i - 1)) % 8)) & 0x01);    
    i--;
  }
  Bit_Offset += bits;
  return Result;
}


unsigned char BitsNeeded(unsigned long value)
{
  unsigned char i = 0;
  while(i < 32){
    if(!value)
      return i;
    value /= 2;
    i++;
  }
  return 31;
}


int BrickPiSetupSensors()
{
//    printf("Internal BrickPi: %p\n", &get_brick());
  unsigned char i = 0;
  while(i < 2){
    int ii = 0;
    while(ii < 256){
      Array[ii] = 0;
      ii++;
    }
    Bit_Offset = 0;
    Array[BYTE_MSG_TYPE] = MSG_TYPE_SENSOR_TYPE;
    Array[BYTE_SENSOR_1_TYPE] = BrickPi.SensorType[PORT_1 + (i * 2)];
    Array[BYTE_SENSOR_2_TYPE] = BrickPi.SensorType[PORT_2 + (i * 2)];
    ii = 0;
    while(ii < 2){
      unsigned char port = (i * 2) + ii;
  //US Fix starts
      if(Array[BYTE_SENSOR_1_TYPE + ii] == TYPE_SENSOR_ULTRASONIC_CONT)
      {
        Array[BYTE_SENSOR_1_TYPE + ii] = TYPE_SENSOR_I2C;
        BrickPi.SensorI2CSpeed[port] = US_I2C_SPEED;
        BrickPi.SensorI2CDevices[port] = 1;
        BrickPi.SensorSettings[port][US_I2C_IDX] = BIT_I2C_MID | BIT_I2C_SAME;
        BrickPi.SensorI2CAddr[port][US_I2C_IDX] = LEGO_US_I2C_ADDR;
        BrickPi.SensorI2CWrite [port][US_I2C_IDX]    = 1;
        BrickPi.SensorI2CRead  [port][US_I2C_IDX]    = 1;
        BrickPi.SensorI2COut   [port][US_I2C_IDX][0] = LEGO_US_I2C_DATA_REG;
      }
      //US Fix Ends
      if(Array[BYTE_SENSOR_1_TYPE + ii] == TYPE_SENSOR_I2C
      || Array[BYTE_SENSOR_1_TYPE + ii] == TYPE_SENSOR_I2C_9V){
        AddBits(3, 0, 8, BrickPi.SensorI2CSpeed[port]);

        if(BrickPi.SensorI2CDevices[port] > 8)
          BrickPi.SensorI2CDevices[port] = 8;

        if(BrickPi.SensorI2CDevices[port] == 0)
          BrickPi.SensorI2CDevices[port] = 1;

        AddBits(3, 0, 3, (BrickPi.SensorI2CDevices[port] - 1));

        unsigned char device = 0;
        while(device < BrickPi.SensorI2CDevices[port]){
          AddBits(3, 0, 7, (BrickPi.SensorI2CAddr[port][device] >> 1));
          AddBits(3, 0, 2, BrickPi.SensorSettings[port][device]);
          if(BrickPi.SensorSettings[port][device] & BIT_I2C_SAME)
          {
            AddBits(3, 0, 4, BrickPi.SensorI2CWrite[port][device]);
            AddBits(3, 0, 4, BrickPi.SensorI2CRead [port][device]);
            unsigned char out_byte = 0;
            while(out_byte < BrickPi.SensorI2CWrite[port][device]){
              AddBits(3, 0, 8, BrickPi.SensorI2COut[port][device][out_byte]);
              out_byte++;
            }
          }
          device++;
        }
      }
      ii++;
    }
    unsigned char UART_TX_BYTES = (((Bit_Offset + 7) / 8) + 3);
//    std::cout << "From setup sensors\n";
    BrickPiTx(BrickPi.Address[i], UART_TX_BYTES, Array);
    // printf("Sent!");
    if(BrickPiRx(&BytesReceived, Array, 5000000))
      return -1;
    if(!(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_SENSOR_TYPE))
      return -1;
    i++;
  }
  return 0;
}


int BrickPiUpdateValues()
{
  unsigned char i = 0;
  unsigned int ii = 0;
  while(i < 2){
    Retried = 0;

__RETRY_COMMUNICATION__:

    ii = 0;
    while(ii < 256){
      Array[ii] = 0;
      ii++;
    }

    Array[BYTE_MSG_TYPE] = MSG_TYPE_VALUES;

    Bit_Offset = 0;

//    AddBits(1, 0, 2, 0);     use this to disable encoder offset

    ii = 0;                 // use this for encoder offset support
    while(ii < 2){
      unsigned char port = (i * 2) + ii;
      if(BrickPi.EncoderOffset[port]){
        long Temp_Value = BrickPi.EncoderOffset[port];
        unsigned char Temp_ENC_DIR = 0;
        unsigned char Temp_BitsNeeded;

        AddBits(1, 0, 1, 1);
        if(Temp_Value < 0){
          Temp_ENC_DIR = 1;
          Temp_Value *= (-1);
        }
        Temp_BitsNeeded = (BitsNeeded(Temp_Value) + 1);
        AddBits(1, 0, 5, Temp_BitsNeeded);
        Temp_Value *= 2;
        Temp_Value |= Temp_ENC_DIR;
        AddBits(1, 0, Temp_BitsNeeded, Temp_Value);
      }
      else{
        AddBits(1, 0, 1, 0);
      }
      ii++;
    }

    int speed;
    unsigned char dir;
    ii = 0;
    while(ii < 2){
      unsigned char port = (i * 2) + ii;
      speed = BrickPi.MotorSpeed[port];
      dir = 0;
      if(speed < 0){
        dir = 1;
        speed *= (-1);
      }
      if(speed > 255){
        speed = 255;
      }
      AddBits(1, 0, 10, ((((speed & 0xFF) << 2) | (dir << 1) | (BrickPi.MotorEnable[port] & 0x01)) & 0x3FF));
      ii++;
    }

    ii = 0;
    while(ii < 2){
      unsigned char port = (i * 2) + ii;
      //if(BrickPi.SensorType[port] == TYPE_SENSOR_I2C || BrickPi.SensorType[port] == TYPE_SENSOR_I2C_9V){
      //US Fix starts
      if(BrickPi.SensorType[port] == TYPE_SENSOR_I2C || BrickPi.SensorType[port] == TYPE_SENSOR_I2C_9V|| BrickPi.SensorType[port] == TYPE_SENSOR_ULTRASONIC_CONT){
      //US Fix ends
        unsigned char device = 0;
        while(device < BrickPi.SensorI2CDevices[port]){
          if(!(BrickPi.SensorSettings[port][device] & BIT_I2C_SAME)){
            AddBits(1, 0, 4, BrickPi.SensorI2CWrite[port][device]);
            AddBits(1, 0, 4, BrickPi.SensorI2CRead [port][device]);
            unsigned char out_byte = 0;
            while(out_byte < BrickPi.SensorI2CWrite[port][device]){
              AddBits(1, 0, 8, BrickPi.SensorI2COut[port][device][out_byte]);
              out_byte++;
            }
          }
          device++;
        }
      }
      ii++;
    }
    
    unsigned char UART_TX_BYTES = (((Bit_Offset + 7) / 8) + 1);
    BrickPiTx(BrickPi.Address[i], UART_TX_BYTES, Array);
    
    int result = BrickPiRx(&BytesReceived, Array, 7500);
    
    if(result != -2){                            // -2 is the only error that indicates that the BrickPi uC did not properly receive the message
      BrickPi.EncoderOffset[((i * 2) + PORT_A)] = 0;
      BrickPi.EncoderOffset[((i * 2) + PORT_B)] = 0;
    }
    
    if(result || (Array[BYTE_MSG_TYPE] != MSG_TYPE_VALUES)){
#ifdef DEBUG
      printf("BrickPiRx error: %d\n", result);
#endif
      if(Retried < 2){
        Retried++;
        goto __RETRY_COMMUNICATION__;
      }
      else{
#ifdef DEBUG
        printf("Retry failed.\n");
#endif
        return -1;
      }      
    }
    
    Bit_Offset = 0;
    
    unsigned char Temp_BitsUsed[2] = {0, 0};         // Used for encoder values
    Temp_BitsUsed[0] = GetBits(1, 0, 5);
    Temp_BitsUsed[1] = GetBits(1, 0, 5);
    unsigned long Temp_EncoderVal;
    
    ii = 0;
    while(ii < 2){
      Temp_EncoderVal = GetBits(1, 0, Temp_BitsUsed[ii]);
      if(Temp_EncoderVal & 0x01){
        Temp_EncoderVal /= 2;
        BrickPi.Encoder[ii + (i * 2)] = Temp_EncoderVal * (-1);}
      else{
        BrickPi.Encoder[ii + (i * 2)] = (Temp_EncoderVal / 2);}      
      ii++;
    }

    ii = 0;
    while(ii < 2){
      unsigned char port = ii + (i * 2);
      switch(BrickPi.SensorType[port]){
        case TYPE_SENSOR_TOUCH:
          BrickPi.Sensor[port] = GetBits(1, 0, 1);
        break;
        //US Fix
        //case TYPE_SENSOR_ULTRASONIC_CONT:
        //case TYPE_SENSOR_ULTRASONIC_SS:
        case TYPE_SENSOR_ULTRASONIC_SS:
          BrickPi.Sensor[port] = GetBits(1, 0, 8);
        break;
        case TYPE_SENSOR_COLOR_FULL:
          BrickPi.Sensor[port] = GetBits(1, 0, 3);
          BrickPi.SensorArray[port][INDEX_BLANK] = GetBits(1, 0, 10);
          BrickPi.SensorArray[port][INDEX_RED  ] = GetBits(1, 0, 10);
          BrickPi.SensorArray[port][INDEX_GREEN] = GetBits(1, 0, 10);
          BrickPi.SensorArray[port][INDEX_BLUE ] = GetBits(1, 0, 10);
        break;
        case TYPE_SENSOR_I2C:
        case TYPE_SENSOR_I2C_9V:
        //US Fix Starts
        case TYPE_SENSOR_ULTRASONIC_CONT:
        {
        //US Fix Ends
          BrickPi.Sensor[port] = GetBits(1, 0, BrickPi.SensorI2CDevices[port]);
          unsigned char device = 0;
          while(device < BrickPi.SensorI2CDevices[port]){
            if(BrickPi.Sensor[port] & (0x01 << device)){
              unsigned char in_byte = 0;
              while(in_byte < BrickPi.SensorI2CRead[port][device]){
                BrickPi.SensorI2CIn[port][device][in_byte] = GetBits(1, 0, 8);
                in_byte++;
              }
            }
            device++;
          }
        }
        break;      
        case TYPE_SENSOR_EV3_US_M0       :
        case TYPE_SENSOR_EV3_US_M1       :
        case TYPE_SENSOR_EV3_US_M2       :
        case TYPE_SENSOR_EV3_US_M3       :
        case TYPE_SENSOR_EV3_US_M4       :
        case TYPE_SENSOR_EV3_US_M5       :
        case TYPE_SENSOR_EV3_US_M6       :
        case TYPE_SENSOR_EV3_COLOR_M0    :
        case TYPE_SENSOR_EV3_COLOR_M1    :
        case TYPE_SENSOR_EV3_COLOR_M2    :
        case TYPE_SENSOR_EV3_COLOR_M4    :
        case TYPE_SENSOR_EV3_COLOR_M5    :
        case TYPE_SENSOR_EV3_GYRO_M0     :
        case TYPE_SENSOR_EV3_GYRO_M1     :
        case TYPE_SENSOR_EV3_GYRO_M2     :
        case TYPE_SENSOR_EV3_GYRO_M4     :
        case TYPE_SENSOR_EV3_INFRARED_M0 :
        case TYPE_SENSOR_EV3_INFRARED_M1 :
        case TYPE_SENSOR_EV3_INFRARED_M3 :
        case TYPE_SENSOR_EV3_INFRARED_M4 :
        case TYPE_SENSOR_EV3_INFRARED_M5 :
        case TYPE_SENSOR_EV3_TOUCH_0:
          BrickPi.Sensor[port] = GetBits(1, 0, 16);
          // gotten_bits = GetBits(1, 0, 16);               // Just test code
          // printf("First Test: %d \n", gotten_bits );     // Just test code
        break;
        case TYPE_SENSOR_EV3_COLOR_M3    :
        case TYPE_SENSOR_EV3_GYRO_M3     :
        case TYPE_SENSOR_EV3_INFRARED_M2 :

            // Filter out negative values.
            // Test and shift.  Get it as a 32 bit number.  
            gotten_bits = GetBits(1,0,32);  
            /*if(gotten_bits > 10 && gotten_bits >= 0){
                gotten_bits = gotten_bits>>8;
                printf("First Test: %d \n", gotten_bits );  
            }*/
            // gotten_bits = GetBits(1,0,8);    
            // printf("Results: %d \n", gotten_bits );  
            // if(gotten_bits < 10 && gotten_bits >= 0){
                BrickPi.Sensor[port] = gotten_bits;
            //}
            break;
        case TYPE_SENSOR_LIGHT_OFF:
        case TYPE_SENSOR_LIGHT_ON:
        case TYPE_SENSOR_RCX_LIGHT:
        case TYPE_SENSOR_COLOR_RED:
        case TYPE_SENSOR_COLOR_GREEN:
        case TYPE_SENSOR_COLOR_BLUE:
        case TYPE_SENSOR_COLOR_NONE:
        default:
          BrickPi.Sensor[(ii + (i * 2))] = GetBits(1, 0, 10);
      }
      if (BrickPi.SensorType[port] == TYPE_SENSOR_ULTRASONIC_CONT)
      {
        if(BrickPi.Sensor[port] & ( 0x01 << US_I2C_IDX)) 
            BrickPi.Sensor[port] = BrickPi.SensorI2CIn[port][US_I2C_IDX][0];
        else
            BrickPi.Sensor[port] = -1;
      }
      ii++;
    }      
    i++;
  }
  return 0;
}


int BrickPiSetup()
{
    //OPEN THE UART
    //The flags (defined in fcntl.h):
    //  Access modes (use 1 of these):
    //      O_RDONLY - Open for reading only.
    //      O_RDWR - Open for reading and writing.
    //      O_WRONLY - Open for writing only.
    //
    //  O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //                                          if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //                                          immediately with a failure status if the output can't be written immediately.
    //
    //  O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
    uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);      //Open in non blocking read/write mode
    if (uart0_filestream == -1)
    {
        //ERROR - CAN'T OPEN SERIAL PORT
        printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
        return -1;
    }

    //CONFIGURE THE UART
    //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    //  Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //  CSIZE:- CS5, CS6, CS7, CS8
    //  CLOCAL - Ignore modem status lines
    //  CREAD - Enable receiver
    //  IGNPAR = Ignore characters with parity errors
    //  ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //  PARENB - Parity enable
    //  PARODD - Odd parity (else even)
    struct termios options;
    tcgetattr(uart0_filestream, &options);
    options.c_cflag = B500000 | CS8 | CLOCAL | CREAD;       //<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;

//    cfmakeraw(&options);
    tcflush(uart0_filestream, TCIFLUSH);
    tcsetattr(uart0_filestream, TCSANOW, &options);

  /*UART_file_descriptor = serialOpen("/dev/ttyAMA0", 500000);
  if(UART_file_descriptor == -1){
    return -1;
  }*/
  return 0;
}


void BrickPiTx(unsigned char dest, unsigned char ByteCount, unsigned char OutArray[])
{
  unsigned char tx_buffer[256];
  tx_buffer[0] = dest;
  tx_buffer[1] = dest + ByteCount;
  tx_buffer[2] = ByteCount;
  unsigned char i = 0;

  while(i < ByteCount)
  {
      tx_buffer[1] += OutArray[i];
      tx_buffer[i + 3] = OutArray[i];
      i++;
  }
  #ifdef DEBUG
//  std::cout << "Buffer: " << tx_buffer << '\n';
//  std::cout << "Buffer Size: " << ByteCount << '\n';
  #endif
  i = 0;
  while(i < (ByteCount + 3))
  {
      int n = write(uart0_filestream, &tx_buffer[i], 1);
//      printf("Tx buffer %u: %u\n", i, tx_buffer[i]);
      if(n < 0) std::strerror(errno);
//      printf("Bytes written: %d\n", n);
      i++;
  }
}


int BrickPiRx(unsigned char *InBytes, unsigned char *InArray, long timeout)
{  // timeout in uS, not mS
  unsigned char rx_buffer[256];
  unsigned char RxBytes = 0;
  unsigned char CheckSum = 0;
  unsigned char i = 0;
  unsigned long OriginalTick = CurrentTickUs();
  unsigned long print_tick = OriginalTick;

  // Check the buffer for values.  If we don't find anything, wait 0.1ms and check again.
  // If we timeout, exit and return -2. 

  int rx_length = -1;       //Filestream, buffer to store in, number of bytes to read (max)
  while(rx_length < 0){
    rx_length = read(uart0_filestream, (void*)rx_buffer, 255);
    if(rx_length < 0 && (CurrentTickUs() - print_tick) >= 1000000)
    {
        printf("Error: %s\n", std::strerror(errno));
        print_tick += 1000000;
    }
    usleep(100);
    if(timeout && ((CurrentTickUs() - OriginalTick) >= timeout)) return -2;  // Timeout
  }
  #ifdef DEBUG
  printf("Bytes read: %i\n", rx_length);
  #endif

  RxBytes = rx_length;

  if(RxBytes < 2)
    return -4; // Buffer is too small to be anything.

  if(RxBytes < (rx_buffer[1] + 2))
    return -6; // Buffer incomplete.

  CheckSum = rx_buffer[1];

  i = 0;
  while(i < (RxBytes - 2)){
    CheckSum += rx_buffer[i + 2];
    InArray[i] = rx_buffer[i + 2];
    i++;
  }

  if(CheckSum != rx_buffer[0])
    return -5;  // Checksum is wrong.

  *InBytes = (RxBytes - 2);

  return 0;
}
