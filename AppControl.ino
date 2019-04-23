/*********************************************************************
This is an example for our nRF8001 Bluetooth Low Energy Breakout

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1697

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Kevin Townsend/KTOWN  for Adafruit Industries.
MIT license, check LICENSE for more information
All text above, and the splash screen below must be included in any redistribution
* edited by Rafael Zasas *
*********************************************************************/

// This version uses the internal data queing so you can treat it like Serial (kinda)!

#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include <Servo.h>

Servo myservo;  // create servo object to control a servo


int pos;  // variable to store the servo position
int power; // variable to store the power delivered to the motor
int flex; // flex sensor 
int data; // variable to store output from flex sensor

// Connect CLK/MISO/MOSI to hardware SPI
#define CLK = 13
#define MISO = 12
#define MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9
#define Motor 6 // pinout for forward fan control 
#define MotorBack 5 // pinout for reverse fan control
#define RedLED 7 // pinout for Red LED - used to display when bluetooth advertising has started 
#define GreenLED 8  // pinout for Green LED - used to display when bluetooth app has connected to module 



Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/
void setup(void)
{ 
  flex= A0; // flex sensor connected to input pin A0
  data=0; 
  power=255; // Initialize motor to full power
  pos = 90; // initialize servo to 90 degree position 
  
  pinMode(RedLED, OUTPUT);
  pinMode(GreenLED, OUTPUT);
  pinMode(flex, INPUT);
  myservo.attach(4);
  myservo.write(pos); 
  Serial.begin(9600);
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001"));

    BTLEserial.setDeviceName("RafsTurbine"); 
    BTLEserial.begin();


}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
*/
/**************************************************************************/
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop()
{
  String result =""; // Used to recieve command from Bluefruit app. Initialized to nothing each iteration of the loop
  
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) { 
      
        digitalWrite(RedLED,HIGH); // Show that advertising has started by turning on Green LED
        Serial.println(F("* Advertising started"));
        
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
        digitalWrite(RedLED,LOW); // turn off Red LED
        digitalWrite(GreenLED,HIGH); // show that app has connected by turning on Green LED
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
        digitalWrite(GreenLED,LOW); // turn off Green LED to show that app has disconnected
    }
    // OK set the last status change to this one
    laststatus = status;
  }

 /* (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial.print(" *"); 
      Serial.print(BTLEserial.available()); 
      
      Serial.println(F(" bytes available from BTLE"));
      
    }*/
    
    // OK while we still have something to read, get a character and print it out

     // To piece together the characters that get sent back from the controller
         
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      //Serial.print(c);
     
      result+=c; // combine the characters to give a readable string that will be used to as a command to control the electronics 
        }

        
        
                  // FAN SPEED CONTROL ( Changes power delivered to fan depending on what button has been pressed.)
           
     if (result.equals("!B10;")){ // btn 1 pressed
      power=100;
      Serial.print(result+" ");
      Serial.print(power);
      result="";
      Serial.println();
     }
          if (result.equals("!B20:")){ // btn 2 pressed
      power=150;
      Serial.print(result+" ");
      Serial.print(power);
      result="";
      Serial.println();
     }
          if (result.equals("!B309")){ // btn 3 pressed
      power=200;
      Serial.print(result+" ");
      Serial.print(power);
      result="";
      Serial.println();
     }
          if (result.equals("!B408")){ // btn4 pressed
      power=255;
      Serial.print(result+" ");
      Serial.print(power);
      result="";
      Serial.println();
     }
                     // END FAN SPEED CONTROL 
     
                     // FAN FORWARD/BACKWARDS (Provides curreent to the motor to have it spin either clockwise or anticlockwise)

           
    if (result.equals("!B516")){ // if the forward arrow is pressed 
      Serial.print(result);
      analogWrite(Motor, power);
      result="";
      Serial.println();
       } 

    if (result.equals("!B507")){ // when the forward arrow is depressed
      Serial.print(result);     
      analogWrite(Motor, 0);
      result="";
      Serial.println();
      } 
      
    if (result.equals("!B615")){ // if the back arrow is pressed 
      Serial.print(result);
      analogWrite(MotorBack, power);
      result="";
      Serial.println();
       } 
    if (result.equals("!B606")){ // when the back arrow is depressed
      Serial.print(result);
      analogWrite(MotorBack, 0);
      result="";
      Serial.println();
      } 
                        // END FAN FORWARD/BACKWARDS


                         // DIRECTION COMMANDS (To change the orientation of the turbine)

            
   if (result.equals("!B714")){ // when the left arrow is pressed
      Serial.print(result);
             for(int i=0 ;i<11;i++){   // step only by 10 degrees each time the button is pressed
              pos++;
              if (pos<179){ // ensures that the servo isnt told to move past its end point
              myservo.write(pos);              // tell servo to go to position in variable 'pos'
              delay(15);  // waits 15ms for the servo to reach the position
              }
             }
      result="";
      Serial.println();
      }

      
   if (result.equals("!B813")){ // when the right arrow is pressed
      Serial.print(result);
             for(int i=0 ;i<11;i++){ // step only by 10 degrees each time the button is pressed
              
              pos--;
              if (pos>0){       // ensures that the servo isnt told to move past its end point
              myservo.write(pos);              // tell servo to go to position in variable 'pos'
              delay(15);  // waits 15ms for the servo to reach the position
              }
             }
             
      result="";
      Serial.println(); 
      }

                        // END DIRECTION COMMANDS


                        // BEGIN FLEX SENSOR READING
data =analogRead(flex);
if(millis()%1000==0){  // ensures that the data is only displayed every 1000 milliseconds instead of each iteration of the loop
  senseflex(data); // calls the senseflex method and passes the flex sensor data as a parameter 
}

                        // END FLEX SENSOR READING


                        
    // Next up, see if we have any data to get from the Serial console

    if (Serial.available()) {
      // Read a line from Serial
      Serial.setTimeout(100); // 100 millisecond timeout
      String s = Serial.readString();

      // We need to convert the line to bytes, no more than 20 at this time
      uint8_t sendbuffer[20];
      s.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, s.length());

      Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");

      // write the data
      BTLEserial.write(sendbuffer, sendbuffersize);
    }
  }


            // FLEX SENSOR OUTPUT METHOD

  void senseflex(int F){ 
    switch(F){ //  Assigns a level to the corresponding output from the flex sensor 

      /*
       !NOTE! 
       Flex sensor output data will change with different reisitor values 
      */
      
  case 15: Serial.println("0");
  break;
  case 14: Serial.println("1");
  break;
  case 13: Serial.println("2");
  break;
  case 12: Serial.println("3");
  break;
  case 11: Serial.println("4");
  break;
  case 10: Serial.println("5");
  break;
  case 9:  Serial.println("6");
  break;
  case 8: Serial.println("7");
  break;
  case 7: Serial.println("8");
  break;
  case 6: Serial.println("9");
  break;
  case 5: Serial.println("10");
  break;
  case 4: Serial.println("11");
  break;
  
}
    
  }
