 #include <Wire.h>

#define I2C_ADDRESS1 0x02 >> 1 //I2C address is 2 for motor 1. arduino work on 7-bit I2C addressing, so the address value is shifted to right by 1 bit.
#define I2C_ADDRESS2 0x04 >> 1 //I2C address is 4 for motor 2. arduino work on 7-bit I2C addressing, so the address value is shifted to right by 1 bit.
#define MAX_SPEED 100  //set maximum speed (0-255)
#define CB_0 0   //command byte "0" to read/write motor max speed
#define CB_1 1   //command byte "1" to read/write motor speed and 


#include <ESP8266WiFi.h>
WiFiClient client;
WiFiServer server(80);

/* WIFI settings */
const char* ssid = "AndroidAP5F79";   //WIFI SSID //AndroidAP5F79
const char* password = "ufqg4888";    //WIFI PASSWORD //ufqg4888

/* data received from application */
String  data =""; 


/* define L298N or L293D motor control pins */
int Relay1 = 12;    //D6
int Relay2 = 16;    //D0


void setup()
{
  /* initialize motor control pins as output */
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT); 

  digitalWrite(Relay1,LOW);
  digitalWrite(Relay2,LOW);

  Wire.begin();
  delay(100);
  Wire.beginTransmission(I2C_ADDRESS1);
  Wire.beginTransmission(I2C_ADDRESS2);
  Wire.write(CB_0);
  Wire.write(MAX_SPEED);  //LSB byte of maximum speed value
  Wire.write(0);  //MSB byte of maximum speed value
  Wire.endTransmission();
  delay(100);
  
  /* start server communication */
  Serial.begin(115200);
  connectWiFi();
  server.begin();
}

void MotorForward(void){ 

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
  }

void MotorBackward(void){ 

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
  }

void TurnRight(void){ 

      //motor1
    Wire.beginTransmission(I2C_ADDRESS1);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(1);  //LSB byte of speed to be set
    Wire.write(255);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop

     //motor2
    Wire.beginTransmission(I2C_ADDRESS2);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(1);  //LSB byte of speed to be set
    Wire.write(255);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop
    delay(100);
  }

void TurnLeft(void){

       //motor1
    Wire.beginTransmission(I2C_ADDRESS1);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(255);  //LSB byte of speed to be set
    Wire.write(0);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop

     //motor2
    Wire.beginTransmission(I2C_ADDRESS2);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(255);  //LSB byte of speed to be set
    Wire.write(0);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop
    delay(100);;
  }

void MotorStop(void){  

      //motor1
    Wire.beginTransmission(I2C_ADDRESS1);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(0);  //LSB byte of speed to be set
    Wire.write(0);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop

     //motor2
    Wire.beginTransmission(I2C_ADDRESS2);  //send the slave address
    Wire.write(CB_1);  //send the command variable for speed
    Wire.write(0);  //LSB byte of speed to be set
    Wire.write(0);  //MSB byte of speed to be set 
    Wire.endTransmission();  //send I2C stop
    delay(100);
 }

void loop()
{
    /* If the server available, run the "checkClient" function */  
    client = server.available();
    if (!client) return; 
    data = checkClient ();
Serial.print(data);
/************************ Run function according to incoming data from application *************************/

    
    if (data == "Relay1ON")
    { 
      digitalWrite(Relay1,HIGH);
      Serial.println("relay will need to on");
      }
    
    else if (data == "Relay1OFF")
    {
      digitalWrite(Relay1,LOW);
      Serial.println("relay will need to off");
      }

    else if (data == "Relay2ON")
    {
      digitalWrite(Relay2,HIGH);
      Serial.println("relay will need to on");
      }
      
    else if (data == "Relay2OFF")
    {
      digitalWrite(Relay2,LOW);
      Serial.println("relay will need to on");
      }
    else if (data == "up") MotorForward();
    
    else if (data == "down") MotorBackward();
   
    else if (data == "left") TurnLeft();
   
    else if (data == "right") TurnRight();
    
    else if (data == "stop") MotorStop();
      
} 

void connectWiFi()
{
  Serial.println("Connecting to WIFI");
  WiFi.begin(ssid, password);
  while ((!(WiFi.status() == WL_CONNECTED)))
  {
    delay(300);
    Serial.print("..");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("NodeMCU Local IP is : ");
  Serial.print((WiFi.localIP()));
}
/********************************** RECEIVE DATA FROM the APP ******************************************/
String checkClient (void)
{
  while(!client.available()) delay(1); 
  String request = client.readStringUntil('\r');
  request.remove(0, 5);
  request.remove(request.length()-9,9);
  return request;
}
