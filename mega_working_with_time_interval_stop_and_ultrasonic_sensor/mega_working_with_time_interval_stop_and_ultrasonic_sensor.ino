#include <Wire.h>
#include <Adafruit_GFX.h> 
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;   

#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSerif12pt7b.h>
#include <FreeDefaultFonts.h>


//define colors using for the display
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

//w=1, b=0 for white line on black board
#define W 1
#define B 0

unsigned long previousTime = 0;
byte seconds ;
byte minutes ;
byte hours ;


// defines variables
long duration = 0;
int distance = 0;
long distanceOld = 0;

// defines pins numbers
const int trigPin = 9;
const int echoPin = 10;
const int led = 12;
// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200;

#define Kp 27
#define Ki 0
#define Kd 15

#define leftCenterSensor   A5  //analog pin A5
#define leftNearSensor     A4  //analog pin A4
#define leftFarSensor      A3  //analog pin A3
#define rightCenterSensor  A2  //analog pin A6
#define rightNearSensor    A1  //analog pin A7
#define rightFarSensor     A0   //digital pin D2

#define I2C_ADDRESS1 0x02 >> 1 //I2C address is 2 for motor 1. arduino work on 7-bit I2C addressing, so the address value is shifted to right by 1 bit.
#define I2C_ADDRESS2 0x04 >> 1 //I2C address is 4 for motor 2. arduino work on 7-bit I2C addressing, so the address value is shifted to right by 1 bit.
#define MAX_SPEED 100  //set maximum speed (0-255)
#define CB_0 0   //command byte "0" to read/write motor max speed
#define CB_1 1   //command byte "1" to read/write motor speed and dirction


//Other peripherals pins
#define led  12
#define buz   8

short interval = 0; //For generelized Blink without delay function
bool psensor[6]; //The Sensor Array
bool ledState = HIGH;
short phase = 0;
unsigned long previousMillis = 0; //For blink without delay function
unsigned char mode;
char path[100];
short pathlen=0;
short Speed;
short error; //line following error, pid will always try to minimize this error
float p; //proportional term
float i; //integral term
float d; //differential term
float pid; //pid term
float prev_error; //previous error term
float prev_i; //previous integral term


//List of all function (Not Prototype... (prototypes not required/mandatory in Arduino))

//Main functions
void readSensor();
void blink_without_delay(short); //Generelized Blink (without delay) -- parameter -> on-off gap time
void Straight(short);
void correct(); //Correct the normal line following
void Stop(); //Stop all motors
void Leap();
void Left();
void Right();
void Yaw(char);  //Generalized Yaw function

void setup()
{ 
   //initialize setup
  tft.reset();
  uint16_t ID = tft.readID();
  tft.begin(ID);
  Serial.print("TFT ID = 0x");
    Serial.println(ID, HEX);
    Serial.println("Calibrate for your Touch Panel");
    if (ID == 0xD3D3) ID = 0x9486; // write-only shield
    tft.setRotation(0);  
  
  //setting background of the display
  //tft.setRotation(1);   //rotation of the display
  tft.fillScreen(CYAN);  //color of the display
  tft.setTextWrap(true); 
    tft.fillRect(40, 250, 160, 40, GREEN); 
  //Sensor pins Mode config
  pinMode(leftCenterSensor, INPUT);
  pinMode(leftNearSensor, INPUT);
  pinMode(leftFarSensor, INPUT);
  pinMode(rightCenterSensor, INPUT);
  pinMode(rightNearSensor, INPUT);
  pinMode(rightFarSensor, INPUT);

  Wire.begin();
  delay(100);
  Wire.beginTransmission(I2C_ADDRESS1);
  Wire.write(CB_0);
  Wire.write(MAX_SPEED); // LSB Byte of Maximum Speed Value
  Wire.write(0); // MSB Byte of Maximum Speed Value
  Wire.endTransmission();
  delay(100);

  Wire.beginTransmission(I2C_ADDRESS2);
  Wire.write(CB_0);
  Wire.write(MAX_SPEED); // LSB Byte of Maximum Speed Value
  Wire.write(0); // MSB Byte of Maximum Speed Value
  Wire.endTransmission();
  delay(100);

  pinMode(buz, OUTPUT);
  pinMode(led, OUTPUT);
  
  Serial.begin(115200); //For debugging only
  
  digitalWrite(led, HIGH);


}

//Setting up the Sensor Array
void readSensor()
{
  if (analogRead(leftFarSensor)>200)
    psensor[0] = 1;
  else 
    psensor[0] = 0;
 if (analogRead(leftNearSensor)>200)
    psensor[1] = 1;
  else 
    psensor[1] = 0;
 if (analogRead(leftCenterSensor)>200)
    psensor[2] = 1;
  else 
    psensor[2] = 0;
  if (analogRead(rightCenterSensor)>200)
    psensor[3] = 1;
  else 
    psensor[3] = 0;
  if (analogRead(rightNearSensor)>200)
    psensor[4] = 1;
  else 
    psensor[4] = 0;
 if (analogRead(rightFarSensor)>200)
    psensor[5] = 1;
  else 
    psensor[5] = 0;

  /*  0 0 1 1 0 0  == The Robo is on the line, perfectly alligned
   *  1 1 1 1 0 0  == The Robo is on an intersection => "Straight/Left" or "only Left"
   *  0 0 1 1 1 1  == ..................intersection => "Straight/Right" or "only Right"
   *  1 1 1 1 1 1  == ..................intersection => "T-intsersection" or "Cross-intersection" or "End of Maze"
   *  0 0 0 0 0 0  == ..................intersection => "Dead End"
   */
   
  if(psensor[0]==W)
  {
    if(path[pathlen] == 'R')
    {
      ++pathlen;
      path[pathlen] = 'L';
    }
    else
    {
      path[pathlen] = 'L';
    }
  }

  else if(psensor[5]==W)
  {
    if(path[pathlen] == 'L')
    {
      ++pathlen;
      path[pathlen] = 'R';
    }
    else
    {
      path[pathlen] = 'R';
    }
  }
  
  //Case: "0 0 1 1 0 0" or "0 1 1 0 0 0" or "0 0 0 1 1 0" or "0 0 1 1 1 0" or "0 1 1 1 0 0" 
  //for line : "1 1 0 0 0 0" or "1 1 1 0 0 0" or "1 0 0 0 0 0" or "0 0 0 0 0 1" or "0 0 0 0 1 1" or "0 0 0 1 1 1"
  if(psensor[0]==W && psensor[1]==B && psensor[2]==B && psensor[3]==B && psensor[4]==B && psensor[5]==B)
  {
    mode = 'O';
    error = 3;
  }
  
  else if(psensor[0]==W && psensor[1]==W && psensor[2]==B && psensor[3]==B && psensor[4]==B && psensor[5]==B)
  {
    mode = 'O';
    error = 3;
    if(path[pathlen] == 'R')
    {
      ++pathlen;
      path[pathlen] = 'L';
    }
    else
    {
      path[pathlen] = 'L';
    }
  }
  
  else if(psensor[0]==B && psensor[1]==W && psensor[2]==W && psensor[3]==B && psensor[4]==B && psensor[5]==B)
  {
    mode = 'O';
    error = 2;
  }
  
  else if(psensor[0]==B && psensor[1]==W && psensor[2]==W && psensor[3]==W && psensor[4]==B && psensor[5]==B)
  {
    mode = 'O';
    error = 1;
  }
  
  else if((psensor[0]==B && psensor[1]==B && psensor[2]==W && psensor[3]==W && psensor[4]==B && psensor[5]==B) ||
          (psensor[0]==B && psensor[1]==B && psensor[2]==B && psensor[3]==W && psensor[4]==B && psensor[5]==B) ||
          (psensor[0]==B && psensor[1]==B && psensor[2]==W && psensor[3]==B && psensor[4]==B && psensor[5]==B))
  {
    mode = 'O';
    error = 0;
  }

  else if(psensor[0]==B && psensor[1]==B && psensor[2]==W && psensor[3]==W && psensor[4]==W && psensor[5]==B)
  {
    mode = 'O';
    error = -1;
  }
  
  else if(psensor[0]==B && psensor[1]==B && psensor[2]==B && psensor[3]==W && psensor[4]==W && psensor[5]==B)
  {
    mode = 'O';
    error = -2;
  }
  
  else if(psensor[0]==B && psensor[1]==B && psensor[2]==B && psensor[3]==B && psensor[4]==W && psensor[5]==W)
  {
    mode = 'O';
    error = -3;
    if(path[pathlen] == 'L')
    {
      ++pathlen;
      path[pathlen] = 'R';
    }
    else
    {
      path[pathlen] = 'R';
    }
  }

  else if(psensor[0]==B && psensor[1]==B && psensor[2]==B && psensor[3]==B && psensor[4]==B && psensor[5]==W)
  {
    mode = 'O';
    error = -3;
    if(path[pathlen] == 'L')
    {
      ++pathlen;
      path[pathlen] = 'R';
    }
    else
    {
      path[pathlen] = 'R';
    }
  }

  //Case: "1 1 1 1 0 0" or "1 1 1 1 1 0" second case for (in case) correction error
  else if (psensor[0]==W && psensor[1]==W && psensor[2]==W && psensor[3]==W && psensor[4]==B && psensor[5]==B)         
  {
    mode = 'L';
    error = 100;
    if(path[pathlen] == 'R')
    {
      ++pathlen;
      path[pathlen] = 'L';
    }
    else
    {
      path[pathlen] = 'L';
    }
  }
  //Case: "0 0 1 1 1 1" or "0 1 1 1 1 1" second case for (in case) correction error
  else if (psensor[0]==B && psensor[1]==B && psensor[2]==W && psensor[3]==W && psensor[4]==W && psensor[5]==W)
  {
    mode = 'R';
    error = 101;
  }
  //Case: "1 1 1 1 1 1"
  else if (psensor[0]==W && psensor[1]==W && psensor[2]==W && psensor[3]==W && psensor[4]==W && psensor[5]==W)
  {
    mode = 'X'; //checkpoint detector
    error = 102;
  }
  //Case: "0 0 0 0 0 0"
  else if (psensor[0]==B && psensor[1]==B && psensor[2]==B && psensor[3]==B && psensor[4]==B && psensor[5]==B)
  {
    mode = 'D'; //Dead End
    error = 103;
  }

  
  Serial.print(psensor[0]);
  Serial.print(psensor[1]);
  Serial.print(psensor[2]);
  Serial.print(psensor[3]);
  Serial.print(psensor[4]);
  Serial.print(psensor[5]);
  Serial.print("  -> MODE : ");
  Serial.print(mode);
  Serial.print("  ERROR : ");
  Serial.println(error);
}
//Sensor array config done


void blink_without_delay(short interval)
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;
    if (ledState == LOW) 
    {
      ledState = HIGH;
    } 
    else 
    {
      ledState = LOW;
    }
    digitalWrite(led, ledState);
  }
}

void loop() 
{
  unsigned long cur_mil = millis();
  readSensor();

   //for Checkpoint detection Reset
  if (cur_mil - previousMillis >= 1000) 
  {
    digitalWrite(led, LOW);
  }
  
{
        Serial.print (":");
        Serial.println(seconds,DEC);
        Serial.print (minutes,DEC);
        Serial.print (":");
        Serial.print (hours, DEC);
        Serial.print (":");
   if (millis() >= (previousTime))
   {
     previousTime = previousTime + 1000;  // use 100000 for uS
     seconds = seconds +1;
     if (seconds == 60)
     {
        seconds = 0;
        minutes = minutes +1;
      
     }
         if (minutes == 60)
     {
        minutes = 0;
        hours = hours +1;
     }
      if (hours == 13)
     {
        hours = 1;
     }
   }
    if (byte (seconds) >= 5){
  Stop();
  }
   if (byte (seconds) > 10){
        cal_pid();
        forward_correct();
        readSensor();
  }
    if (byte (seconds) >= 15){
   Stop();
  }
     if (byte (seconds) > 20){
        cal_pid();
        forward_correct();
        readSensor();
  }
}
    switch(mode)
    {
      case 'O' :  //On-Line
      {
        Serial.println("on-line");
        cal_pid();
        forward_correct();
        //Go straight and on the same time correct position
        //Should not consist of any delay
        readSensor();
        break;
      }


      case 'L' :
      {
        Stop();
        delay(50);
        readSensor(); //new
         //motor1
    Wire.beginTransmission(I2C_ADDRESS1);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(255)-50;  //LSB byte of speed to be set
    Wire.write(0);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop

    Wire.beginTransmission(I2C_ADDRESS2);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(255)-50;  //LSB byte of speed to be set
    Wire.write(0);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop
        delay(150);
        Stop();
        //delay(50);
        Serial.println("turn left");
        Left();
        readSensor();
        break;
      }

      case 'R' :
      {
        Stop();
        delay(50);
        readSensor();
        
           //motor1
    Wire.beginTransmission(I2C_ADDRESS1);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(255)-50;  //LSB byte of speed to be set
    Wire.write(0);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop

    Wire.beginTransmission(I2C_ADDRESS2);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(255)-50;  //LSB byte of speed to be set
    Wire.write(0);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop
        delay(150);  
        Stop();
        //delay(50);
        Serial.println("turn right");
        Right();
        readSensor();
        break;
      }

      
      case 'X' :  //Cross-intersection (Checkpoint detector if 1 1 1 1 1 1)
      {
        Stop();
        Serial.println("++++++++++++++++ CHECKPOINT detected +++++++++++++++++++");
        //Flash the LED and buzz the buzzer
        
        //Right(); //giving particular turn the prioirity
        
        previousMillis = cur_mil;
        analogWrite(buz, 150);
        digitalWrite(led, HIGH);
        Serial.println(cur_mil);
        
        cal_pid();
        //forward_correct();
        
        readSensor();
        break;
      }
      
      case 'D' :  //Dead-End
      {
        Stop();     
        readSensor();
        Serial.println("CASE D!!!");
        Serial.print(path);
        Serial.println(" ");
        unsigned long timer = 0;
          do
          {
            timer++;
            Serial.print("Detecting Line :> ");
            Serial.println(timer);
              //motor1
    Wire.beginTransmission(I2C_ADDRESS1);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(255)-50;  //LSB byte of speed to be set
    Wire.write(0);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop

    Wire.beginTransmission(I2C_ADDRESS2);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(255)-50;  //LSB byte of speed to be set
    Wire.write(0);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop
            readSensor();
            if(timer > 30)
            {
              goto End;
            }
          }while(error>3 || error<-3);
          End:
          Stop();
          
        if(path[pathlen] == 'L')
        {
          Left();
        }
        else if(path[pathlen] == 'R')
        {
          Right();
        }
        pathlen=0;
        break;
      }
    }
}

void cal_pid()
{
  p = error;
  i = i + prev_i;
  d = error - (prev_error);
  pid = (Kp*p)+(Ki*i)+(Kd*d);
  prev_i = i;
  prev_error = error;
}

void forward_correct()
{
      //motor1
    Wire.beginTransmission(I2C_ADDRESS1);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(255)-pid;  //LSB byte of speed to be set
    Wire.write(0);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop

    Wire.beginTransmission(I2C_ADDRESS2);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(255)+pid;  //LSB byte of speed to be set
    Wire.write(0);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop
}

void Left()
{
 // Stop();
  unsigned long cur_mil = millis();

  Yaw('L');
  delay(10);
  Stop();
  do
  {
    Yaw('L');  //High Speed Left Yaw
    Serial.println("LEFT TURN");
    readSensor();
  }while(error > 2 || error < -2);
  Stop();
}

void Right()
{
  unsigned long cur_mil = millis();

  Yaw('R');
  delay(10);
  Stop();
  do
  {
    Yaw('R');  //High Speed Right Yaw
    Serial.println("RIGHT TURN");
    readSensor();
  }while(error > 2 || error < -2);
  Stop();
}

void Yaw(char direc)
{
  switch(direc)
  {
    case 'L' :
    {
      Serial.println("Left");
       //motor1
    Wire.beginTransmission(I2C_ADDRESS1);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(1);  //LSB byte of speed to be set
    Wire.write(255);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop

     //motor2
    Wire.beginTransmission(I2C_ADDRESS2);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(255);  //LSB byte of speed to be set
    Wire.write(0);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop
    delay(100);
      break;
    }
    case 'R' :
    {
      Serial.println("Right");
 //motor1
    Wire.beginTransmission(I2C_ADDRESS1);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(255);  //LSB byte of speed to be set
    Wire.write(0);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop

     //motor2
    Wire.beginTransmission(I2C_ADDRESS2);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(1);  //LSB byte of speed to be set
    Wire.write(255);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop
    delay(100);
      break;
    }
  }
}

void Stop()
{
 //motor1
    Wire.beginTransmission(I2C_ADDRESS1);  //send the slave address
    Wire.beginTransmission(I2C_ADDRESS2);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(0);  //LSB byte of speed to be set
    Wire.write(0);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop
}
