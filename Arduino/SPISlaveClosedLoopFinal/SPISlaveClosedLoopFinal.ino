/*
Virtual Solar Boat Motion Simulator
Software for second SPI slave
Sends analog input values and kalman IMU data to Rpi master
and receives pwm microsecond values from master and writes them to each servos
By: Jezreel Saltar
*/

//Libraries for Servos
#include <Servo.h>

//Libraries for Kalman Filter
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

//AnalogInputs pins
#define ROLL_IN   A0   //pin location
#define PITCH_IN  A1   //pin location
#define YAW_IN    A2   //pin location
#define DELTAZ_IN A3   //pin location
#define THRUST_IN A6   //pin location

//Servo Pins, for Servo at Location ij
#define SERVO_11  10   //pin location
#define SERVO_12  9    //pin location
#define SERVO_21  8    //pin location
#define SERVO_22  7    //pin location
#define SERVO_31  6    //pin location
#define SERVO_32  5    //pin location
#define SERVO_YAW 4    //pin location

#define DEBUG_PRINT false  //set true to print servo pwm values

//Global Variables
//Single Initialization
int buf[40];           //buffer to hold incoming pwm bytes
char dat;              //hold handshake received character
byte marker = 0;       //mark positions in switch case
float pwmServo[7];     //microseconds to send to servos
Servo servos[7];       //Servo object array
//Servo output pins
int servoPins[7] = {SERVO_11,SERVO_12,SERVO_21,SERVO_22,SERVO_31,SERVO_32,SERVO_YAW}; 
bool lastByte = false;  //True when the last byte has been sent
int  counterDetach = 0; //Counter to detach servo pins
bool detached = false;  //True when servo pins are detached
bool slave2done = false;//True when timer for slave2 is done

union one               //Union is used to convert floats into bytes and 
  {                     //viceversa, to send and receive through SPI
  float f;
  char  b[4];
  } data1, data2, data3, data4, data5, data6, data7, data8,
  data9, data10, data11, data12, data13, data14;

//Kalman Variables
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
int16_t tempRaw;
float gyroXangle, gyroYangle; // Angle calculate using the gyro only
float compAngleX, compAngleY; // Calculated angle using a complementary filter
float kalAngleX, kalAngleY; // Calculated angle using a Kalman filter, X is pitch, Y is roll
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
float dt;

unsigned long lastReadTime, readPeriod = 1000000;

void setup (void)
{
  Serial.begin(115200);    //Debugging
  pinMode(MISO, OUTPUT);   //Define MISO pin as output (slave)
  SPCR |= _BV(SPE);        //set enable bit of the SPI register
  pinMode(2,OUTPUT);       //Set pin 2 as output, used to select slave2
  digitalWrite(2,HIGH);    //Set as high to disable slave2

  //Kalman Setup
    //kalmanSetup();
   
  //Servo Initialization, 7 times
    for (int i = 0; i < 7; i++)
    {
      servos[i].attach(servoPins[i]);
    }//end for 
    lastReadTime = micros();
}  

void loop (void)
{
 //polls SPI data register
 if((SPSR & (1 << SPIF)) != 0)
  {
    spiSwitch();
  }
  //Updates analog and kalman values every read period
  //or when the last byte has been sent
  if (micros()>lastReadTime + readPeriod || lastByte)
  {
    updateAnalogReads();           //Function to update analog inputs
    kalmanRun();                   //Run IMU kalman filter
    //delay(2.4);                  //To simulate kalman when not running it
    data13.f = kalAngleY;          //Roll angle from IMU
    data14.f = kalAngleX;          //Pitch angle from IMU
    lastReadTime = micros() + readPeriod;
    lastByte = false;              //Reset lastbyte
    marker = 0;                    //reset marker
  
    counterDetach++; //counter to detach servos
    if (counterDetach > 3) //if too much idle time detach servo pins
    {
      for (int i = 0; i < 7; i++)
     {
       servos[i].detach();
     }//end for 
     detached = true;
    }//end if to detach servo pins
  }//end if to update values
  
}//end main loop
  
void spiSwitch()
{
  switch (marker)     //Switch case to process each byte
  {
  case 0:             //does handshakes to receive a command and send a ready signal
    dat = SPDR;       //Store received command byte from data register
    lastReadTime = micros() + readPeriod;
    counterDetach = 0;          //Reset counter to detach servos
    
    if (slave2done)
      {
        digitalWrite(2,HIGH);   //Set as high to disable slave2
        pinMode(MISO, OUTPUT);  //Reset MISO pin to output
        slave2done = false;     //Reset slave2done bool
      } //end if slave done
     
    if (detached) //If servos pins are detached, attach them
      {
        //Servo Initialization, 7 times
        for (int i = 0; i < 7; i++)
        {
        servos[i].attach(servoPins[i]);
        }//end for  
        detached = false;
      }//end if detached
      
    if (dat == 'a')  //Handshake for pwm
    {
      SPDR = 'b';    //sends a b to tell the master it is ready
      marker = 1;      //case 1 next
    }
    if (dat == 'c')  //Handshake for inputs
    {
      SPDR = 'd';    //sends a b to tell the master it is ready
      marker = 30;   //Jump to case 30 next
    }
    if (dat == 'e')  //Handshake for kalman
    {
      SPDR = 'f';    //sends an f to tell the master it is ready
      marker = 51;   //Jump to case 51 next
    }
    if (dat == 'g')  //Handshake to select slave2
                     //for master to receive boat controller data
    {
      pinMode(MISO, INPUT); //Set SPI MISO to input to not interfere with slave2
      digitalWrite(2,LOW);  //set slave select pin for slave2 to high
      delay(1);             //Wait for slave2 to do its thing
      slave2done = true;    //Slave2 is done
      marker=0;             //Reset marker
    }
    break;
    
  //Cases for receiving bytes of servo microseconds    
  case 1:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 2:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 3:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 4:
    buf[marker-1] = SPDR;
    marker++;  
    break;
  case 5:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 6:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 7:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 8:
    buf[marker-1] = SPDR;
    marker++;   
    break;
  case 9:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 10:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 11:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 12:
    buf[marker-1] = SPDR;
    marker++;  
    break;
  case 13:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 14:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 15:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 16:
    buf[marker-1] = SPDR;
    marker++;   
    break;
  case 17:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 18:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 19:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 20:
    buf[marker-1] = SPDR;
    marker++;  
    break;
  case 21:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 22:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 23:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 24:
    buf[marker-1] = SPDR;
    marker++;    
    break;
  case 25:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 26:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 27:
    buf[marker-1] = SPDR;
    marker++;
    break;
  case 28:
    buf[marker-1] = SPDR;
    marker++;
    pwmExecute();
    lastByte = true;  
    marker=0;  
    break;
      
  //cases for sending analog input readings  
  case 30:
    marker++;
    SPDR = data8.b[0];    
    break;    
  case 31:
    marker++;
    SPDR = data8.b[1]; 
    break;
  case 32:
    marker++;
    SPDR = data8.b[2];    
    break;    
  case 33:
    marker++;
    SPDR = data8.b[3]; 
    break;
  case 34:
    marker++;
    SPDR = data9.b[0];    
    break;    
  case 35:
    marker++;
    SPDR = data9.b[1]; 
    break;
  case 36:
    marker++;
    SPDR = data9.b[2];    
    break;    
  case 37:
    marker++;
    SPDR = data9.b[3]; 
    break; 
  case 38:
    marker++;
    SPDR = data10.b[0];    
    break;    
  case 39:
    marker++;
    SPDR = data10.b[1]; 
    break;
  case 40:
    marker++;
    SPDR = data10.b[2];    
    break;    
  case 41:
    marker++;
    SPDR = data10.b[3]; 
    break; 
  case 42:
    marker++;
    SPDR = data11.b[0];    
    break;    
  case 43:
    marker++;
    SPDR = data11.b[1]; 
    break;
  case 44:
    marker++;
    SPDR = data11.b[2];    
    break;    
  case 45:
    marker++;
    SPDR = data11.b[3]; 
    break;      
  case 46:
    marker++;
    SPDR = data12.b[0];    
    break;    
  case 47:
    marker++;
    SPDR = data12.b[1]; 
    break;
  case 48:
    marker++;
    SPDR = data12.b[2];    
    break;    
  case 49:
    marker++;
    SPDR = data12.b[3];
    marker=0;
    break;  

  //Cases for sending kalman IMU values
  case 51:
    marker++;
    SPDR = data13.b[0];    
    break;    
  case 52:
    marker++;
    SPDR = data13.b[1]; 
    break;
  case 53:
    marker++;
    SPDR = data13.b[2];    
    break;    
  case 54:
    marker++;
    SPDR = data13.b[3]; 
    break;      
  case 55:
    marker++;
    SPDR = data14.b[0];    
    break;    
  case 56:
    marker++;
    SPDR = data14.b[1]; 
    break;
  case 57:
    marker++;
    SPDR = data14.b[2];    
    break;    
  case 58:
    marker++;
    SPDR = data14.b[3]; 
    marker=0;
    break;  
  }
}  
// function to save the received pwm's
void pwmExecute(void)
{
  int count;
  for (int i = 0; i < 4; i++)
  data1.b[i] = buf[count++];

  for (int i = 0; i < 4; i++)
  data2.b[i] = buf[count++];

  for (int i = 0; i < 4; i++)
  data3.b[i] = buf[count++];

  for (int i = 0; i < 4; i++)
  data4.b[i] = buf[count++];

  for (int i = 0; i < 4; i++)
  data5.b[i] = buf[count++];

  for (int i = 0; i < 4; i++)
  data6.b[i] = buf[count++];

  for (int i = 0; i < 4; i++)
  data7.b[i] = buf[count++];

  count = 0;// end buffer data

   //Store each servo microseconds in the pwmServo array
   pwmServo[0] = data1.f;
   pwmServo[1] = data2.f;
   pwmServo[2] = data3.f;
   pwmServo[3] = data4.f;
   pwmServo[4] = data5.f;
   pwmServo[5] = data6.f;
   pwmServo[6] = data7.f;

  //Send microseconds to servos
     //for (int i = 0; i < 6; i++)
     //servos[i].writeMicroseconds((int)pwmServo[i]); //end for microseconds to servos
  
     //servos[6].writeMicroseconds((int)pwmServo[6]);
     
      #if DEBUG_PRINT
      Serial.print("pwmServo11 =   ");Serial.println(pwmServo[0]);
      Serial.print("pwmServo12 =   ");Serial.println(pwmServo[1]);
      Serial.print("pwmServo21 =   ");Serial.println(pwmServo[2]);
      Serial.print("pwmServo22 =   ");Serial.println(pwmServo[3]);
      Serial.print("pwmServo31 =   ");Serial.println(pwmServo[4]);
      Serial.print("pwmServo32 =   ");Serial.println(pwmServo[5]);
      Serial.print("pwmServoY  =   ");Serial.println(pwmServo[6]);
      #endif
      
} //end pwmExecute();

//Function to update analog values from joystick and potentiometer inputs
void updateAnalogReads()
{
      data8.f  = (float)analogRead(ROLL_IN);
      data9.f  = (float)analogRead(PITCH_IN);
      data10.f = (float)analogRead(YAW_IN);
      data11.f = (float)analogRead(DELTAZ_IN);
      data12.f = (float)analogRead(THRUST_IN);
}//end updateAnalog()

