/*
  Reading multiple RFID tags, simultaneously!
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 3rd, 2016
  https://github.com/sparkfun/Simultaneous_RFID_Tag_Reader

  Constantly reads and outputs any tags heard

  If using the Simultaneous RFID Tag Reader (SRTR) shield, make sure the serial slide
  The switch is in the 'SW-UART' position
*/

/* Digital Bag Toss
 * NMU Senior Project
 * Created  : February 1st 2021
 * Modified : April 17th 2021
 * 
 * Description: Tracks 8 RFID tags embedded in bag toss bags. The RSSI Values are attached to each bag. The RSSI
 * Values are then used to determine where the bag is located (On the board, Off the board, or in the hole). This
 * Score is then depicted on a 16 x 16 LED Matrix. Cancellation Scoring is used in the scoring function.
 * 
 * Authors: Sparkfun Electronics (Setup and Read Tags function), Kevin Darrah (LED Matrix Driver Code and XY Coordinate Code), 
 * and Matthew Alanskas and Evan Petroff (Main, part of Read Tags, and Scoreboard functions).
 */

#include <SoftwareSerial.h> //Used for transmitting to the device

SoftwareSerial softSerial(2, 3); //RX, TX

#include "SparkFun_UHF_RFID_Reader.h" //Library for controlling the M6E Nano module
RFID nano; //Create instance


int team1total = 0;                                   // initilizes team 1 score to 0
int team2total = 0;                                   // initilizes team 2 score to 0
unsigned long starttime = 0;
int EPCdata [8] = {0,0,0,0,0,0,0,0};                  // array to store and sort EPC data from tags
int rssidata [8] = {0,0,0,0,0,0,0,0};                 // array to store and sort rssi signal strength from tags

// double array to initilize tags
int tag[8][12] = 
{ 
{226, 00, 00, 27, 105, 9, 01, 132, 9, 80 , 185, 102},                  // team 1 bag 1
{226, 00, 00, 27, 105, 9, 01, 132, 9, 32, 191, 99  },                  // team 1 bag 1
{226, 00, 00, 27, 105, 9, 01, 132, 17, 80 ,164, 118},                  // team 2 bag 1
{226, 00, 00, 27, 105, 9, 01, 121, 32, 144, 64, 156},                  // team 2 bag 1
{226, 00, 00, 27, 105, 9, 01, 121, 33, 00 , 62, 143},                  // team 1 bag 2
{226, 00, 00, 27, 105, 9, 01, 121, 33, 16 , 60, 135},                  // team 1 bag 2
{226, 00, 00, 27, 105, 9, 01, 121, 33, 64, 58, 127},                   // team 2 bag 2
{226, 00, 00, 27, 105, 9, 01, 121, 33, 48 , 60, 131},                  // team 2 bag 2
};                                     

//VARIABLES AND DEFINES HERE - NEEDED BY THE WS2812 DRIVER CODE
#define numberOfLEDs 256// total number of RGB LEDs
byte RGB[768];//take your number of LEDs and multiply by 3

// FUNCTIONS HERE
void RGB_update(int LED, byte RED, byte GREEN, byte BLUE);//function to drive LEDs

void clearLEDs()
{
  memset(RGB, 0, sizeof(RGB));
}

void mapLEDXY(int y, int x, byte RED, byte  GREEN, byte BLUE) {
  int RGBlocation = 0;

  if (y % 2 == 0) { //even column
    RGBlocation = x + y * 16;

  } else { //odd column
    RGBlocation = 15 - x + y * 16;
  }

  RGB[RGBlocation * 3] = BLUE;
  RGB[RGBlocation * 3 + 1] = RED;
  RGB[RGBlocation * 3 + 2] = GREEN;
}

void readtags(void);                                   // function to read tags
void calculatescore(void);                             // function to calculate score
void scoreboard(void);                                 // function to output score on scoreboard
void clearscore(void);                                 // function to clear scoreboard after one team hits 21 points or if reset is needed

void setup()
{
  
  Serial.begin(115200);
  //while (!Serial); //Wait for the serial port to come online

  if (setupNano(38400) == false) //Configure nano to run at 38400bps
  {
    //Serial.println(F("Module failed to respond. Please check wiring."));
    while (1); //Freeze!
  }
  
  pinMode(8, OUTPUT);              // sets pin 8 to output data to led matrix
  
  // sets up scoreboard to show 0 - 0
  mapLEDXY(4,0,5,5,0);mapLEDXY(4,1,5,5,0);mapLEDXY(4,2,5,5,0);mapLEDXY(4,3,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(4,11,5,5,0);mapLEDXY(4,12,5,5,0);mapLEDXY(4,13,5,5,0);mapLEDXY(4,14,5,5,0);mapLEDXY(4,15,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,4,5,5,0);mapLEDXY(5,11,5,5,0);mapLEDXY(5,15,5,5,0);mapLEDXY(6,0,5,5,0);mapLEDXY(6,1,5,5,0);mapLEDXY(6,2,5,5,0);mapLEDXY(6,3,5,5,0);mapLEDXY(6,4,5,5,0);mapLEDXY(6,11,5,5,0);mapLEDXY(6,12,5,5,0);mapLEDXY(6,13,5,5,0);mapLEDXY(6,14,5,5,0);mapLEDXY(6,15,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);mapLEDXY(9,4,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);mapLEDXY(9,15,5,5,0);mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,1,5,5,0);mapLEDXY(11,4,5,5,0);mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);mapLEDXY(13,0,5,5,0);mapLEDXY(13,11,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,0,5,5,0);mapLEDXY(15,11,5,5,0);

  RGB_update(-1, 0, 0, 0);
  delay(100);
  RGB_update(-1, 0, 0, 0);
  delay(10);
  
  pinMode(7, INPUT_PULLUP);                // sets pin 7 as a push button input
  pinMode(9, INPUT_PULLUP);                // sets pin 9 as a push button input
  pinMode(10, INPUT_PULLUP);               // sets pin 10 as a push button input
  pinMode(11, INPUT_PULLUP);               // sets pin 11 as a push button input
  pinMode(12, INPUT_PULLUP);               // sets pin 12 as a push button input
  pinMode(13, INPUT_PULLUP);               // sets pin 13 as a push button input
  nano.setRegion(REGION_NORTHAMERICA);     // Set to North America

  nano.setReadPower(2200); //22.00 dBm. Higher values may caues USB port to brown out
  //Max Read TX Power is 27.00 dBm and may cause temperature-limit throttling
  starttime = millis();
  
}

void loop(void)
{
     // ints used to initiate push button commands
     int team1plus;
     int team1minus;
     int team2plus;
     int team2minus;
     int button_state;
     int hardreset;
     
     bool calculate = 0;
     bool score = 0;
     
     // sets up digital read for all push buttons
     button_state = digitalRead(12);  //triggers rfid reader to start reading (pin 12)
     hardreset = digitalRead(9);      //triggers rfid reader to start reading  (pin 9)
     team1plus = digitalRead(7);      // triger add one point to team 1 score  (pin 7)
     team1minus = digitalRead(11);    // trigger minus one point to team 1 score (pin 11)
     team2plus = digitalRead(13);     // trigger add one point to team 2 score  (pin 13)
     team2minus = digitalRead(3);     // trigger minus one point to team 2 score  (pin 3)
    

     // Actual Program
     
     if (button_state == 0)
       {
        nano.startReading();                  // Begin scanning for tags
          starttime = millis();
          while(millis() - starttime <= 2000) // Scan tags for 2 second period
          {
        
            readtags();                       // run read tags
          } 
          calculate = 1;
   
          if (calculate == 1)  
          {
           calculatescore();                  // calculate score
           calculate = 0;
           score = 1;
          }
          delay(200);
          if (score == 1)  
          {
           scoreboard();                      // run scoreboard function
           score = 0;
           for(int i = 0; i < 7; i++)
           {
           EPCdata[i] = 0;                    // reset array to store and sort EPC data from tags
           rssidata[i] = 0;                   // reset array to store and sort rssi signal strength from tags
           }
         
          }
       }
       // reset score
       if (hardreset == 0)
       {
          delay(1000);
          team1total = 0;
          team2total = 0;
          scoreboard();
         
       }

       // subtract one point from team 1
       if (team1minus == 0)
       {
          delay(500);     
          team1total--;   // decrements team1total
          scoreboard();   // prints score onto matrix
       }
       // subtract one point from team 2
       if (team2minus == 0)
       {
         delay(500);
          team2total--;
          scoreboard();
       }
       // add one point to team 1
       if (team1plus == 0)
       {
          delay(500);
          team1total++;
          scoreboard();
       }
       // add one point to team 2
       if (team2plus == 0)
       {
         delay(500);
          team2total++;
          scoreboard();
       }
       
       // Serial.println(("Press a key to begin scanning for tags."));
       //while (!Serial.available()); //Wait for user to send a character
       //Serial.read(); //Throw away the user's character
     /*bool tag = 1;
     if(tag == 1)
     {
       starttime = millis();  
       while(starttime - millis() <= 2000) // read tags for a period of two seconds
       {
            readtags();
       }

       bool score = 1;                          // variable used to post score to scoreboard
          if (score == 1)  
          {
           scoreboard();                           // run scoreboard function
           score = 0;
           int EPCdata [8] = {0,0,0,0,0,0,0,0};                  // array to store and sort EPC data from tags
           int rssidata [8] = {0,0,0,0,0,0,0,0};                 // array to store and sort rssi signal strength from tags
            Serial.read();              // Throw away the user's character
         
          }

      tag = 0;
     
     }
    */

    // troubleshooting
 
     /*
     if(team1total == 21)                       
     {
        Serial.println("Team 1 Wins!");         // Team 1 wins when reaching score of 21
        
     }
     else if(team2total == 21)
     {
        Serial.println("Team 2 Wins!");         // Team 2 wins when reaching score of 21
        
        
     }
     */
}



void readtags(void)
{
  
  if (nano.check() == true) //Check to see if any new data has come in from module
  {
    byte responseType = nano.parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp

    if (responseType == RESPONSE_IS_KEEPALIVE)
    {
      //Serial.println(F("Scanning"));
    }
    else if (responseType == RESPONSE_IS_TAGFOUND)
    {
      
      //If we have a full record we can pull out the fun bits
      int rssi = nano.getTagRSSI(); //Get the RSSI for this tag read

      long freq = nano.getTagFreq(); //Get the frequency this tag was detected at

      long timeStamp = nano.getTagTimestamp(); //Get the time this was read, (ms) since last keep-alive message

      byte tagEPCBytes = nano.getTagEPCBytes(); //Get the number of bytes of EPC from response
    
      //Print EPC bytes, this is a subsection of bytes from the response/msg array
      int EPC[12] = {0,0,0,0,0,0,0,0,0,0,0,0};    // tag data
        for (int x = 0; x < tagEPCBytes; x++)    // go through the EPC tag bytes 
        {
            EPC[x] = (nano.msg[31 + x]);          // nano.msg is a buffer used to skip other information when reading tag
        }
         char correcttag = 1;                    
        for (int x = 0; x < 8; x++)               // go through 8 tags read
        {
          correcttag = 1;                         // set boolean variable
              for (int y = 0; y < 12; y++)        // go through each byte for each tag
              {
                   if (EPC[y] != tag[x][y])       // ensure bytes match for each tag to initilized array
                   {
                        correcttag = 0;
                   }
              }
                   if (correcttag == 1)           // if EPC data matches, assign rssi data to tags in order in the array
                   {
                     
                     
                        EPCdata[x] = 1;           // used to keep data in order
                        rssidata[x] = rssi;
                      
                   }
        }

        /* print information for each tag on serial monitor for troubleshooting
        for (int x = 0; x < 8; x++)
        {
            if (EPCdata[x] == 1)
            {
                Serial.print("Tag ");
                Serial.println(x+1);
                Serial.print("Rssi:  ");
                Serial.println(rssidata[x]);  
            }
        }  */ 
  }
  else
  {
      //Unknown response
      //Serial.print("Unknown Error");
      
  }
 
 }
  
  return;
}

void calculatescore(void)
{
  int team1 = 0;                  // team 1 score for the round
  int team2 = 0;                  // team 2 score for the round
  int difference = 0;             // difference between scores
  int bagnum = 0;                 // holds bag number data
  int bagrssi = 0;                // holds bag rssi data
  
  // for loop used to assign rssi values to each bag and number each bag
  for (int i = 0; i < 8; i = i + 2)     
  {
      bagrssi = 0;
      // if both tags are read, bagrssi is assigned to the lower value of the two
      if ((EPCdata[i] == 1) & (EPCdata[i+1] == 1))
      {
        if (rssidata[i] < rssidata[i+1])
        {
          bagrssi = rssidata[i];
        }
        else
        {
          bagrssi = rssidata[i+1];
        }
      }
      // if only tag is read, this tags rssi value is assigned to bagrssi
      else if ((EPCdata[i] == 1) & (EPCdata[i+1] == 0))
      {
        bagrssi = rssidata[i];
      }
      else if ((EPCdata[i] == 0) & (EPCdata[i+1] == 1)){
        bagrssi = rssidata[i+1];
      }
      bagnum = ((i + 2)/2);
      //Serial.print("bag ");
      //Serial.println(bagnum);
      //Serial.print("RSSI: ");
      //Serial.println(bagrssi);

      if ((bagnum) % 2 == 0)            // even numbered bags are assigned to team 1
      {
        if (bagrssi == 0)               // if the signal strength is 0, the bag is off the board
        {
          team1 = team1;
        }
        
        else                            // else 1 point is added to team 1
        {
          team1 = team1 + 1;
        }
        
      }

      else                             // odd numbered bags are assigned to team 2
      {
        if (bagrssi == 0)              // if the signal strength is 0, the bag is off the board
        {
          team2 = team2;
        }
       
        else                           // else 1 point is added to team 2
        {
          team2 = team2 + 1;
        } 

        
        
      }

  }

  // if team 1 scores more points, the difference between team 1 and team 2 scores will be assigned to team 1's score
  if(team1 > team2)
  {
    difference = team1 - team2;
    team1total = team1total + difference;
      // if team 1 goes over 21, their score stays the same
      if(team1total > 21)          
      {
        team1total = team1total - difference;
      }
    team2total = team2total;
  }
  // if team 2 scores more points, the difference between team 2 and team 1 scores will be assigned to team 2's score
  else if(team2 > team1)
  {
    difference = team2 - team1;
    team2total = team2total + difference;
    // if team 2 goes over 21, their score stays the same
      if(team2total > 21)
      {
        team2total = team2total - difference;
      }  
    team1total = team1total;
  }
  // if the scores are equal, no points are awarded
  else
  {
    team1total = team1total;
    team2total = team2total;
  }
}
void scoreboard(void)
{
  clearLEDs();
  RGB_update(-1, 0, 0, 0);

  if(team1total == 0)
  {
    mapLEDXY(4,0,5,5,0);mapLEDXY(4,1,5,5,0);mapLEDXY(4,2,5,5,0);mapLEDXY(4,3,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,4,5,5,0);
    mapLEDXY(6,0,5,5,0);mapLEDXY(6,1,5,5,0);mapLEDXY(6,2,5,5,0);mapLEDXY(6,3,5,5,0);mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);
    mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);
    mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
  }
  else if(team1total == 1)
  {
    mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,1,5,5,0);mapLEDXY(5,2,5,5,0);mapLEDXY(5,3,5,5,0);mapLEDXY(5,4,5,5,0);mapLEDXY(6,1,5,5,0)
    ;mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);
    mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);
    mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
  }
  else if(team1total == 2)
  {
    mapLEDXY(4,0,5,5,0);mapLEDXY(4,1,5,5,0);mapLEDXY(4,2,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,2,5,5,0);mapLEDXY(5,4,5,5,0)
    ;mapLEDXY(6,0,5,5,0);mapLEDXY(6,2,5,5,0);mapLEDXY(6,3,5,5,0);mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);mapLEDXY(9,4,5,5,0)
    ;mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);mapLEDXY(11,4,5,5,0)
    ;mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
  }
  else if(team1total == 3)
  {
    mapLEDXY(4,0,5,5,0);mapLEDXY(4,1,5,5,0);mapLEDXY(4,2,5,5,0);mapLEDXY(4,3,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,2,5,5,0);
    mapLEDXY(5,4,5,5,0);mapLEDXY(6,0,5,5,0);mapLEDXY(6,2,5,5,0);mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);mapLEDXY(9,4,5,5,0);
    mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);mapLEDXY(11,4,5,5,0);
    mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
  }
  else if(team1total == 4)
  {
    mapLEDXY(4,0,5,5,0);mapLEDXY(4,1,5,5,0);mapLEDXY(4,2,5,5,0);mapLEDXY(4,3,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,2,5,5,0);mapLEDXY(6,0,5,5,0);mapLEDXY(6,1,5,5,0);
    mapLEDXY(6,2,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);
    mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);
    mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
  }
  else if(team1total == 5)
  {
    mapLEDXY(4,0,5,5,0);mapLEDXY(4,2,5,5,0);mapLEDXY(4,3,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,2,5,5,0);mapLEDXY(5,4,5,5,0);
    mapLEDXY(6,0,5,5,0);mapLEDXY(6,1,5,5,0);mapLEDXY(6,2,5,5,0);mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);mapLEDXY(9,4,5,5,0);
    mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);mapLEDXY(11,4,5,5,0);
    mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
  }
  else if(team1total == 6)
  {
    mapLEDXY(4,0,5,5,0);mapLEDXY(4,2,5,5,0);mapLEDXY(4,3,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,2,5,5,0);mapLEDXY(5,4,5,5,0);
    mapLEDXY(6,0,5,5,0);mapLEDXY(6,1,5,5,0);mapLEDXY(6,2,5,5,0);mapLEDXY(6,3,5,5,0);mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);
    mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);
    mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
  }
  else if(team1total == 7)
  {
    mapLEDXY(4,0,5,5,0);mapLEDXY(4,1,5,5,0);mapLEDXY(4,2,5,5,0);mapLEDXY(4,3,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(6,0,5,5,0);
    mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);
    mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);
    mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
  }
  else if(team1total == 8)
  {
    mapLEDXY(4,0,5,5,0);mapLEDXY(4,1,5,5,0);mapLEDXY(4,2,5,5,0);mapLEDXY(4,3,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,2,5,5,0);
    mapLEDXY(5,4,5,5,0);mapLEDXY(6,0,5,5,0);mapLEDXY(6,1,5,5,0);mapLEDXY(6,2,5,5,0);mapLEDXY(6,3,5,5,0);mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);
    mapLEDXY(8,3,5,5,0);mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);
    mapLEDXY(11,1,5,5,0);mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
  }
  else if(team1total == 9)
  {
    mapLEDXY(4,0,5,5,0);mapLEDXY(4,1,5,5,0);mapLEDXY(4,2,5,5,0);mapLEDXY(4,3,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,2,5,5,0);
    mapLEDXY(6,0,5,5,0);mapLEDXY(6,1,5,5,0);mapLEDXY(6,2,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);
    mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);
    mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
  }
  else if(team1total == 10)
  {
    mapLEDXY(0,0,5,5,0);mapLEDXY(0,1,5,5,0);mapLEDXY(0,2,5,5,0);mapLEDXY(0,3,5,5,0);mapLEDXY(0,4,5,5,0);mapLEDXY(1,0,5,5,0);mapLEDXY(1,4,5,5,0);
    mapLEDXY(2,0,5,5,0);mapLEDXY(2,1,5,5,0);mapLEDXY(2,2,5,5,0);mapLEDXY(2,3,5,5,0);mapLEDXY(2,4,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);
    mapLEDXY(5,1,5,5,0);mapLEDXY(5,2,5,5,0);mapLEDXY(5,3,5,5,0);mapLEDXY(5,4,5,5,0);mapLEDXY(6,1,5,5,0);mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);
    mapLEDXY(8,3,5,5,0);mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);
    mapLEDXY(11,1,5,5,0);mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
  }
  else if(team1total == 11)
  {
    mapLEDXY(0,4,5,5,0);mapLEDXY(1,0,5,5,0);mapLEDXY(1,1,5,5,0);mapLEDXY(1,2,5,5,0);mapLEDXY(1,3,5,5,0);mapLEDXY(1,4,5,5,0);mapLEDXY(2,1,5,5,0);
    mapLEDXY(2,4,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,1,5,5,0);mapLEDXY(5,2,5,5,0);mapLEDXY(5,3,5,5,0);mapLEDXY(5,4,5,5,0);
    mapLEDXY(6,1,5,5,0);mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);
    mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);
    mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
  }
  else if(team1total == 12)
  {
    mapLEDXY(0,0,5,5,0);mapLEDXY(0,1,5,5,0);mapLEDXY(0,2,5,5,0);mapLEDXY(0,4,5,5,0);mapLEDXY(1,0,5,5,0);mapLEDXY(1,2,5,5,0);mapLEDXY(1,4,5,5,0);
    mapLEDXY(2,0,5,5,0);mapLEDXY(2,2,5,5,0);mapLEDXY(2,3,5,5,0);mapLEDXY(2,4,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,1,5,5,0);
    mapLEDXY(5,2,5,5,0);mapLEDXY(5,3,5,5,0);mapLEDXY(5,4,5,5,0);mapLEDXY(6,1,5,5,0);mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);
    mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);
    mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
  }
  else if(team1total == 13)
  {
    mapLEDXY(0,0,5,5,0);mapLEDXY(0,1,5,5,0);mapLEDXY(0,2,5,5,0);mapLEDXY(0,3,5,5,0);mapLEDXY(0,4,5,5,0);mapLEDXY(1,0,5,5,0);mapLEDXY(1,2,5,5,0);
    mapLEDXY(1,4,5,5,0);mapLEDXY(2,0,5,5,0);mapLEDXY(2,2,5,5,0);mapLEDXY(2,4,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,1,5,5,0);
    mapLEDXY(5,2,5,5,0);mapLEDXY(5,3,5,5,0);mapLEDXY(5,4,5,5,0);mapLEDXY(6,1,5,5,0);mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);
    mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);
    mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
  }
  else if(team1total == 14)
  {
    mapLEDXY(0,0,5,5,0);mapLEDXY(0,1,5,5,0);mapLEDXY(0,2,5,5,0);mapLEDXY(0,3,5,5,0);mapLEDXY(0,4,5,5,0);mapLEDXY(1,2,5,5,0);mapLEDXY(2,0,5,5,0);
    mapLEDXY(2,1,5,5,0);mapLEDXY(2,2,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,1,5,5,0);mapLEDXY(5,2,5,5,0);mapLEDXY(5,3,5,5,0);
    mapLEDXY(5,4,5,5,0);mapLEDXY(6,1,5,5,0);mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);
    mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);
    mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
  }
  else if(team1total == 15)
  {
    mapLEDXY(0,0,5,5,0);mapLEDXY(0,2,5,5,0);mapLEDXY(0,3,5,5,0);mapLEDXY(0,4,5,5,0);mapLEDXY(1,0,5,5,0);mapLEDXY(1,2,5,5,0);mapLEDXY(1,4,5,5,0);
    mapLEDXY(2,0,5,5,0);mapLEDXY(2,1,5,5,0);mapLEDXY(2,2,5,5,0);mapLEDXY(2,4,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,1,5,5,0);
    mapLEDXY(5,2,5,5,0);mapLEDXY(5,3,5,5,0);mapLEDXY(5,4,5,5,0);mapLEDXY(6,1,5,5,0);mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);
    mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);
    mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
    //RGB_update(-1, 0, 0, 0);
  }
  else if(team1total == 16)
  {
    mapLEDXY(0,0,5,5,0);mapLEDXY(0,2,5,5,0);mapLEDXY(0,3,5,5,0);mapLEDXY(0,4,5,5,0);mapLEDXY(1,0,5,5,0);mapLEDXY(1,2,5,5,0);mapLEDXY(1,4,5,5,0);
    mapLEDXY(2,0,5,5,0);mapLEDXY(2,1,5,5,0);mapLEDXY(2,2,5,5,0);mapLEDXY(2,3,5,5,0);mapLEDXY(2,4,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);
    mapLEDXY(5,1,5,5,0);mapLEDXY(5,2,5,5,0);mapLEDXY(5,3,5,5,0);mapLEDXY(5,4,5,5,0);mapLEDXY(6,1,5,5,0);mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);
    mapLEDXY(8,3,5,5,0);mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);
    mapLEDXY(11,1,5,5,0);mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
     // RGB_update(-1, 0, 0, 0);
  }
  else if(team1total == 17)
  {
    mapLEDXY(0,0,5,5,0);mapLEDXY(0,1,5,5,0);mapLEDXY(0,2,5,5,0);mapLEDXY(0,3,5,5,0);mapLEDXY(0,4,5,5,0);mapLEDXY(1,0,5,5,0);mapLEDXY(2,0,5,5,0);
    mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,1,5,5,0);mapLEDXY(5,2,5,5,0);mapLEDXY(5,3,5,5,0);mapLEDXY(5,4,5,5,0);mapLEDXY(6,1,5,5,0);
    mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);
    mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);
    mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
    //RGB_update(-1, 0, 0, 0);
  }
  else if(team1total == 18)
  {
    mapLEDXY(0,0,5,5,0);mapLEDXY(0,1,5,5,0);mapLEDXY(0,2,5,5,0);mapLEDXY(0,3,5,5,0);mapLEDXY(0,4,5,5,0);mapLEDXY(1,0,5,5,0);mapLEDXY(1,2,5,5,0)
    ;mapLEDXY(1,4,5,5,0);mapLEDXY(2,0,5,5,0);mapLEDXY(2,1,5,5,0);mapLEDXY(2,2,5,5,0);mapLEDXY(2,3,5,5,0);mapLEDXY(2,4,5,5,0);mapLEDXY(4,4,5,5,0);
    mapLEDXY(5,0,5,5,0);mapLEDXY(5,1,5,5,0);mapLEDXY(5,2,5,5,0);mapLEDXY(5,3,5,5,0);mapLEDXY(5,4,5,5,0);mapLEDXY(6,1,5,5,0);mapLEDXY(6,4,5,5,0);
    mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);
    mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);
    mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
     //RGB_update(-1, 0, 0, 0);
  }
  else if(team1total == 19)
  {
    mapLEDXY(0,0,5,5,0);mapLEDXY(0,1,5,5,0);mapLEDXY(0,2,5,5,0);mapLEDXY(0,3,5,5,0);mapLEDXY(0,4,5,5,0);mapLEDXY(1,0,5,5,0);mapLEDXY(1,2,5,5,0);
    mapLEDXY(2,0,5,5,0);mapLEDXY(2,1,5,5,0);mapLEDXY(2,2,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,1,5,5,0);mapLEDXY(5,2,5,5,0);
    mapLEDXY(5,3,5,5,0);mapLEDXY(5,4,5,5,0);mapLEDXY(6,1,5,5,0);mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);mapLEDXY(9,4,5,5,0);
    mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);mapLEDXY(11,4,5,5,0);
    mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
    // RGB_update(-1, 0, 0, 0);
  }
  else if(team1total == 20)
  {
    mapLEDXY(0,0,5,5,0);mapLEDXY(0,1,5,5,0);mapLEDXY(0,2,5,5,0);mapLEDXY(0,3,5,5,0);mapLEDXY(0,4,5,5,0);mapLEDXY(1,0,5,5,0);mapLEDXY(1,4,5,5,0);
    mapLEDXY(2,0,5,5,0);mapLEDXY(2,1,5,5,0);mapLEDXY(2,2,5,5,0);mapLEDXY(2,3,5,5,0);mapLEDXY(2,4,5,5,0);mapLEDXY(4,0,5,5,0);mapLEDXY(4,1,5,5,0);
    mapLEDXY(4,2,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,2,5,5,0);mapLEDXY(5,4,5,5,0);mapLEDXY(6,0,5,5,0);mapLEDXY(6,2,5,5,0);
    mapLEDXY(6,3,5,5,0);mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);
    mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);
    mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
     //RGB_update(-1, 0, 0, 0);
  }
  else if(team1total == 21)
  {
    mapLEDXY(0,4,5,5,0);mapLEDXY(1,0,5,5,0);mapLEDXY(1,1,5,5,0);mapLEDXY(1,2,5,5,0);mapLEDXY(1,3,5,5,0);mapLEDXY(1,4,5,5,0);mapLEDXY(2,1,5,5,0);
    mapLEDXY(2,4,5,5,0);mapLEDXY(4,0,5,5,0);mapLEDXY(4,1,5,5,0);mapLEDXY(4,2,5,5,0);mapLEDXY(4,4,5,5,0);mapLEDXY(5,0,5,5,0);mapLEDXY(5,2,5,5,0);
    mapLEDXY(5,4,5,5,0);mapLEDXY(6,0,5,5,0);mapLEDXY(6,2,5,5,0);mapLEDXY(6,3,5,5,0);mapLEDXY(6,4,5,5,0);mapLEDXY(8,1,5,5,0);mapLEDXY(8,3,5,5,0);
    mapLEDXY(9,4,5,5,0);mapLEDXY(10,0,5,5,0);mapLEDXY(10,1,5,5,0);mapLEDXY(10,2,5,5,0);mapLEDXY(10,3,5,5,0);mapLEDXY(10,4,5,5,0);mapLEDXY(11,1,5,5,0);
    mapLEDXY(11,4,5,5,0);mapLEDXY(13,0,5,5,0);mapLEDXY(14,0,5,5,0);mapLEDXY(14,1,5,5,0);mapLEDXY(14,2,5,5,0);mapLEDXY(14,3,5,5,0);mapLEDXY(14,4,5,5,0);mapLEDXY(15,0,5,5,0);
    //RGB_update(-1, 0, 0, 0);
  }

  if(team2total == 0)
  {
    mapLEDXY(4,11,5,5,0);mapLEDXY(4,12,5,5,0);mapLEDXY(4,13,5,5,0);mapLEDXY(4,14,5,5,0);mapLEDXY(4,15,5,5,0);mapLEDXY(5,11,5,5,0);
    mapLEDXY(5,15,5,5,0);mapLEDXY(6,11,5,5,0);mapLEDXY(6,12,5,5,0);mapLEDXY(6,13,5,5,0);mapLEDXY(6,14,5,5,0);mapLEDXY(6,15,5,5,0);
    mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);mapLEDXY(9,15,5,5,0);
    mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);mapLEDXY(11,14,5,5,0);
    mapLEDXY(11,15,5,5,0);mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
    //RGB_update(-1, 0, 0, 0);
  }
   else if(team2total == 1)
  {
    mapLEDXY(4,15,5,5,0);mapLEDXY(5,11,5,5,0);mapLEDXY(5,12,5,5,0);mapLEDXY(5,13,5,5,0);mapLEDXY(5,14,5,5,0);mapLEDXY(5,15,5,5,0);
    mapLEDXY(6,12,5,5,0);mapLEDXY(6,15,5,5,0);mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);
    mapLEDXY(9,13,5,5,0);mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);
    mapLEDXY(11,13,5,5,0);mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);
    mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
    // RGB_update(-1, 0, 0, 0);
  }
  else if(team2total == 2)
  {
    mapLEDXY(4,11,5,5,0);mapLEDXY(4,12,5,5,0);mapLEDXY(4,13,5,5,0);mapLEDXY(4,15,5,5,0);mapLEDXY(5,11,5,5,0);mapLEDXY(5,13,5,5,0);
    mapLEDXY(5,15,5,5,0);mapLEDXY(6,11,5,5,0);mapLEDXY(6,13,5,5,0);mapLEDXY(6,14,5,5,0);mapLEDXY(6,15,5,5,0);mapLEDXY(8,12,5,5,0);
    mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);
    mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);
    mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
    // RGB_update(-1, 0, 0, 0);
  } 
  else if(team2total== 3)
  {
    mapLEDXY(4,11,5,5,0);mapLEDXY(4,12,5,5,0);mapLEDXY(4,13,5,5,0);mapLEDXY(4,14,5,5,0);mapLEDXY(4,15,5,5,0);mapLEDXY(5,11,5,5,0);
    mapLEDXY(5,13,5,5,0);mapLEDXY(5,15,5,5,0);mapLEDXY(6,11,5,5,0);mapLEDXY(6,13,5,5,0);mapLEDXY(6,15,5,5,0);mapLEDXY(8,12,5,5,0);
    mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);
    mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);
    mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
    // RGB_update(-1, 0, 0, 0);
  }
  else if(team2total == 4)
  {
    mapLEDXY(4,11,5,5,0);mapLEDXY(4,12,5,5,0);mapLEDXY(4,13,5,5,0);mapLEDXY(4,14,5,5,0);mapLEDXY(4,15,5,5,0);mapLEDXY(5,13,5,5,0);
    mapLEDXY(6,11,5,5,0);mapLEDXY(6,12,5,5,0);mapLEDXY(6,13,5,5,0);mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);
    mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);
    mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);
    mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
    //  RGB_update(-1, 0, 0, 0);
  }
  else if(team2total == 5)
  {
    mapLEDXY(4,11,5,5,0);mapLEDXY(4,13,5,5,0);mapLEDXY(4,14,5,5,0);mapLEDXY(4,15,5,5,0);mapLEDXY(5,11,5,5,0);mapLEDXY(5,13,5,5,0);
    mapLEDXY(5,15,5,5,0);mapLEDXY(6,11,5,5,0);mapLEDXY(6,12,5,5,0);mapLEDXY(6,13,5,5,0);mapLEDXY(6,15,5,5,0);mapLEDXY(8,12,5,5,0);
    mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);
    mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);
    mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
    // RGB_update(-1, 0, 0, 0);
  }
  else if(team2total == 6)
  {
    mapLEDXY(4,11,5,5,0);mapLEDXY(4,13,5,5,0);mapLEDXY(4,14,5,5,0);mapLEDXY(4,15,5,5,0);mapLEDXY(5,11,5,5,0);mapLEDXY(5,13,5,5,0);
    mapLEDXY(5,15,5,5,0);mapLEDXY(6,11,5,5,0);mapLEDXY(6,12,5,5,0);mapLEDXY(6,13,5,5,0);mapLEDXY(6,14,5,5,0);mapLEDXY(6,15,5,5,0);
    mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);mapLEDXY(9,15,5,5,0);
    mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);mapLEDXY(11,14,5,5,0);
    mapLEDXY(11,15,5,5,0);mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
   // RGB_update(-1, 0, 0, 0);
  } 
  else if(team2total == 7)
  {
    mapLEDXY(4,11,5,5,0);mapLEDXY(4,12,5,5,0);mapLEDXY(4,13,5,5,0);mapLEDXY(4,14,5,5,0);mapLEDXY(4,15,5,5,0);mapLEDXY(5,11,5,5,0);
    mapLEDXY(6,11,5,5,0);mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);
    mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);
    mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);
    mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
     // RGB_update(-1, 0, 0, 0);
  }
  else if(team2total == 8)
  {
    mapLEDXY(4,11,5,5,0);mapLEDXY(4,12,5,5,0);mapLEDXY(4,13,5,5,0);mapLEDXY(4,14,5,5,0);mapLEDXY(4,15,5,5,0);mapLEDXY(5,11,5,5,0);
    mapLEDXY(5,13,5,5,0);mapLEDXY(5,15,5,5,0);mapLEDXY(6,11,5,5,0);mapLEDXY(6,12,5,5,0);mapLEDXY(6,13,5,5,0);mapLEDXY(6,14,5,5,0);
    mapLEDXY(6,15,5,5,0);mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);
    mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);
    mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);
    mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
    // RGB_update(-1, 0, 0, 0);
  }
  else if(team2total == 9)
  {
    mapLEDXY(4,11,5,5,0);mapLEDXY(4,12,5,5,0);mapLEDXY(4,13,5,5,0);mapLEDXY(4,14,5,5,0);mapLEDXY(4,15,5,5,0);mapLEDXY(5,11,5,5,0);
    mapLEDXY(5,13,5,5,0);mapLEDXY(6,11,5,5,0);mapLEDXY(6,12,5,5,0);mapLEDXY(6,13,5,5,0);mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);
    mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);
    mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);mapLEDXY(13,11,5,5,0);
    mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
   // RGB_update(-1, 0, 0, 0);
  }
  else if(team2total == 10)
  {
    mapLEDXY(0,11,5,5,0);mapLEDXY(0,12,5,5,0);mapLEDXY(0,13,5,5,0);mapLEDXY(0,14,5,5,0);mapLEDXY(0,15,5,5,0);mapLEDXY(1,11,5,5,0);
    mapLEDXY(1,15,5,5,0);mapLEDXY(2,11,5,5,0);mapLEDXY(2,12,5,5,0);mapLEDXY(2,13,5,5,0);mapLEDXY(2,14,5,5,0);mapLEDXY(2,15,5,5,0);
    mapLEDXY(4,15,5,5,0);mapLEDXY(5,11,5,5,0);mapLEDXY(5,12,5,5,0);mapLEDXY(5,13,5,5,0);mapLEDXY(5,14,5,5,0);mapLEDXY(5,15,5,5,0);
    mapLEDXY(6,12,5,5,0);mapLEDXY(6,15,5,5,0);mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);
    mapLEDXY(9,13,5,5,0);mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);
    mapLEDXY(11,13,5,5,0);mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);
    mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
     // RGB_update(-1, 0, 0, 0);
  }
  else if(team2total == 11)
  {
    mapLEDXY(0,15,5,5,0);mapLEDXY(1,11,5,5,0);mapLEDXY(1,12,5,5,0);mapLEDXY(1,13,5,5,0);mapLEDXY(1,14,5,5,0);mapLEDXY(1,15,5,5,0);
    mapLEDXY(2,12,5,5,0);mapLEDXY(2,15,5,5,0);mapLEDXY(4,15,5,5,0);mapLEDXY(5,11,5,5,0);mapLEDXY(5,12,5,5,0);mapLEDXY(5,13,5,5,0);
    mapLEDXY(5,14,5,5,0);mapLEDXY(5,15,5,5,0);mapLEDXY(6,12,5,5,0);mapLEDXY(6,15,5,5,0);mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);
    mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);
    mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);mapLEDXY(13,11,5,5,0);
    mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
    // RGB_update(-1, 0, 0, 0);
  }
  else if(team2total == 12)
  {
    mapLEDXY(0,11,5,5,0);mapLEDXY(0,12,5,5,0);mapLEDXY(0,13,5,5,0);mapLEDXY(0,15,5,5,0);mapLEDXY(1,11,5,5,0);mapLEDXY(1,13,5,5,0);
    mapLEDXY(1,15,5,5,0);mapLEDXY(2,11,5,5,0);mapLEDXY(2,13,5,5,0);mapLEDXY(2,14,5,5,0);mapLEDXY(2,15,5,5,0);mapLEDXY(4,15,5,5,0);
    mapLEDXY(5,11,5,5,0);mapLEDXY(5,12,5,5,0);mapLEDXY(5,13,5,5,0);mapLEDXY(5,14,5,5,0);mapLEDXY(5,15,5,5,0);mapLEDXY(6,12,5,5,0);
    mapLEDXY(6,15,5,5,0);mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);
    mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);
    mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);
    mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
    // RGB_update(-1, 0, 0, 0);
  }
  else if(team2total == 13)
  {
    mapLEDXY(0,11,5,5,0);mapLEDXY(0,12,5,5,0);mapLEDXY(0,13,5,5,0);mapLEDXY(0,14,5,5,0);mapLEDXY(0,15,5,5,0);mapLEDXY(1,11,5,5,0);
    mapLEDXY(1,13,5,5,0);mapLEDXY(1,15,5,5,0);mapLEDXY(2,11,5,5,0);mapLEDXY(2,13,5,5,0);mapLEDXY(2,15,5,5,0);mapLEDXY(4,15,5,5,0);
    mapLEDXY(5,11,5,5,0);mapLEDXY(5,12,5,5,0);mapLEDXY(5,13,5,5,0);mapLEDXY(5,14,5,5,0);mapLEDXY(5,15,5,5,0);mapLEDXY(6,12,5,5,0);
    mapLEDXY(6,15,5,5,0);mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);
    mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);
    mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
  }
  else if(team2total == 14)
  {
    mapLEDXY(0,11,5,5,0);mapLEDXY(0,12,5,5,0);mapLEDXY(0,13,5,5,0);mapLEDXY(0,14,5,5,0);mapLEDXY(0,15,5,5,0);mapLEDXY(1,13,5,5,0);
    mapLEDXY(2,11,5,5,0);mapLEDXY(2,12,5,5,0);mapLEDXY(2,13,5,5,0);mapLEDXY(4,15,5,5,0);mapLEDXY(5,11,5,5,0);mapLEDXY(5,12,5,5,0);
    mapLEDXY(5,13,5,5,0);mapLEDXY(5,14,5,5,0);mapLEDXY(5,15,5,5,0);mapLEDXY(6,12,5,5,0);mapLEDXY(6,15,5,5,0);mapLEDXY(8,12,5,5,0);
    mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);
    mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);
    mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
  }
  else if(team2total == 15)
  {
    mapLEDXY(0,11,5,5,0);mapLEDXY(0,13,5,5,0);mapLEDXY(0,14,5,5,0);mapLEDXY(0,15,5,5,0);mapLEDXY(1,11,5,5,0);mapLEDXY(1,13,5,5,0);
    mapLEDXY(1,15,5,5,0);mapLEDXY(2,11,5,5,0);mapLEDXY(2,12,5,5,0);mapLEDXY(2,13,5,5,0);mapLEDXY(2,15,5,5,0);mapLEDXY(4,15,5,5,0);
    mapLEDXY(5,11,5,5,0);mapLEDXY(5,12,5,5,0);mapLEDXY(5,13,5,5,0);mapLEDXY(5,14,5,5,0);mapLEDXY(5,15,5,5,0);mapLEDXY(6,12,5,5,0);
    mapLEDXY(6,15,5,5,0);mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);
    mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);
    mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
  }
  else if(team2total == 16)
  {
    mapLEDXY(0,11,5,5,0);mapLEDXY(0,13,5,5,0);mapLEDXY(0,14,5,5,0);mapLEDXY(0,15,5,5,0);mapLEDXY(1,11,5,5,0);mapLEDXY(1,13,5,5,0);
    mapLEDXY(1,15,5,5,0);mapLEDXY(2,11,5,5,0);mapLEDXY(2,12,5,5,0);mapLEDXY(2,13,5,5,0);mapLEDXY(2,14,5,5,0);mapLEDXY(2,15,5,5,0);
    mapLEDXY(4,15,5,5,0);mapLEDXY(5,11,5,5,0);mapLEDXY(5,12,5,5,0);mapLEDXY(5,13,5,5,0);mapLEDXY(5,14,5,5,0);mapLEDXY(5,15,5,5,0);
    mapLEDXY(6,12,5,5,0);mapLEDXY(6,15,5,5,0);mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);
    mapLEDXY(9,13,5,5,0);mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);
    mapLEDXY(11,13,5,5,0);mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);
    mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
  }
  else if(team2total == 17)
  {
    mapLEDXY(0,11,5,5,0);mapLEDXY(0,12,5,5,0);mapLEDXY(0,13,5,5,0);mapLEDXY(0,14,5,5,0);mapLEDXY(0,15,5,5,0);mapLEDXY(1,11,5,5,0);
    mapLEDXY(2,11,5,5,0);mapLEDXY(4,15,5,5,0);mapLEDXY(5,11,5,5,0);mapLEDXY(5,12,5,5,0);mapLEDXY(5,13,5,5,0);mapLEDXY(5,14,5,5,0);
    mapLEDXY(5,15,5,5,0);mapLEDXY(6,12,5,5,0);mapLEDXY(6,15,5,5,0);mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);
    mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);
    mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);
    mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
  }
  else if(team2total == 18)
  {
    mapLEDXY(0,11,5,5,0);mapLEDXY(0,12,5,5,0);mapLEDXY(0,13,5,5,0);mapLEDXY(0,14,5,5,0);mapLEDXY(0,15,5,5,0);mapLEDXY(1,11,5,5,0);
    mapLEDXY(1,13,5,5,0);mapLEDXY(1,15,5,5,0);mapLEDXY(2,11,5,5,0);mapLEDXY(2,12,5,5,0);mapLEDXY(2,13,5,5,0);mapLEDXY(2,14,5,5,0);
    mapLEDXY(2,15,5,5,0);mapLEDXY(4,15,5,5,0);mapLEDXY(5,11,5,5,0);mapLEDXY(5,12,5,5,0);mapLEDXY(5,13,5,5,0);mapLEDXY(5,14,5,5,0);
    mapLEDXY(5,15,5,5,0);mapLEDXY(6,12,5,5,0);mapLEDXY(6,15,5,5,0);mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);
    mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);
    mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);
    mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
  }
  else if(team2total == 19)
  {
    mapLEDXY(0,11,5,5,0);mapLEDXY(0,12,5,5,0);mapLEDXY(0,13,5,5,0);mapLEDXY(0,14,5,5,0);mapLEDXY(0,15,5,5,0);mapLEDXY(1,11,5,5,0);
    mapLEDXY(1,13,5,5,0);mapLEDXY(2,11,5,5,0);mapLEDXY(2,12,5,5,0);mapLEDXY(2,13,5,5,0);mapLEDXY(4,15,5,5,0);mapLEDXY(5,11,5,5,0);
    mapLEDXY(5,12,5,5,0);mapLEDXY(5,13,5,5,0);mapLEDXY(5,14,5,5,0);mapLEDXY(5,15,5,5,0);mapLEDXY(6,12,5,5,0);mapLEDXY(6,15,5,5,0);
    mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);mapLEDXY(9,15,5,5,0);
    mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);mapLEDXY(11,14,5,5,0);
    mapLEDXY(11,15,5,5,0);mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
  }
  else if(team2total == 20)
  {
    mapLEDXY(0,11,5,5,0);mapLEDXY(0,12,5,5,0);mapLEDXY(0,13,5,5,0);mapLEDXY(0,14,5,5,0);mapLEDXY(0,15,5,5,0);mapLEDXY(1,11,5,5,0);
    mapLEDXY(1,15,5,5,0);mapLEDXY(2,11,5,5,0);mapLEDXY(2,12,5,5,0);mapLEDXY(2,13,5,5,0);mapLEDXY(2,14,5,5,0);mapLEDXY(2,15,5,5,0);
    mapLEDXY(4,11,5,5,0);mapLEDXY(4,12,5,5,0);mapLEDXY(4,13,5,5,0);mapLEDXY(4,15,5,5,0);mapLEDXY(5,11,5,5,0);mapLEDXY(5,13,5,5,0);
    mapLEDXY(5,15,5,5,0);mapLEDXY(6,11,5,5,0);mapLEDXY(6,13,5,5,0);mapLEDXY(6,14,5,5,0);mapLEDXY(6,15,5,5,0);mapLEDXY(8,12,5,5,0);
    mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);
    mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);
    mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
  }
  else if(team2total == 21)
  {
    mapLEDXY(0,15,5,5,0);mapLEDXY(1,11,5,5,0);mapLEDXY(1,12,5,5,0);mapLEDXY(1,13,5,5,0);mapLEDXY(1,14,5,5,0);mapLEDXY(1,15,5,5,0);
    mapLEDXY(2,12,5,5,0);mapLEDXY(2,15,5,5,0);mapLEDXY(4,11,5,5,0);mapLEDXY(4,12,5,5,0);mapLEDXY(4,13,5,5,0);mapLEDXY(4,15,5,5,0);
    mapLEDXY(5,11,5,5,0);mapLEDXY(5,13,5,5,0);mapLEDXY(5,15,5,5,0);mapLEDXY(6,11,5,5,0);mapLEDXY(6,13,5,5,0);mapLEDXY(6,14,5,5,0);
    mapLEDXY(6,15,5,5,0);mapLEDXY(8,12,5,5,0);mapLEDXY(8,14,5,5,0);mapLEDXY(9,11,5,5,0);mapLEDXY(9,12,5,5,0);mapLEDXY(9,13,5,5,0);
    mapLEDXY(9,15,5,5,0);mapLEDXY(10,11,5,5,0);mapLEDXY(10,13,5,5,0);mapLEDXY(10,15,5,5,0);mapLEDXY(11,11,5,5,0);mapLEDXY(11,13,5,5,0);
    mapLEDXY(11,14,5,5,0);mapLEDXY(11,15,5,5,0);mapLEDXY(13,11,5,5,0);mapLEDXY(14,11,5,5,0);mapLEDXY(14,12,5,5,0);mapLEDXY(14,13,5,5,0);
    mapLEDXY(14,14,5,5,0);mapLEDXY(14,15,5,5,0);mapLEDXY(15,11,5,5,0);
  }
  RGB_update(-1, 0, 0, 0);
  delay(100);
  RGB_update(-1, 0, 0, 0);
  delay(10);
}  

// Clears score due to hard reset
void clearscore(void)
{
  //Serial.println("Clear Score was started.");
  
  team1total = 0;
  team2total = 0;

  //Serial.print("Team 1: ");
  //erial.println(team1total);
 // Serial.print("Team 2: ");
 // Serial.println(team2total);
}

//Gracefully handles a reader that is already configured and already reading continuously
//Because Stream does not have a .begin() we have to do this outside the library
boolean setupNano(long baudRate)
{
  nano.begin(softSerial); //Tell the library to communicate over software serial port

  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered
  softSerial.begin(baudRate); //For this test, assume module is already at our desired baud rate
  while (softSerial.isListening() == false); //Wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while (softSerial.available()) softSerial.read();

  nano.getVersion();

  if (nano.msg[0] == ERROR_WRONG_OPCODE_RESPONSE)
  {
    //This happens if the baud rate is correct but the module is doing a ccontinuous read
    nano.stopReading();

    //Serial.println(F("Module continuously reading. Asking it to stop..."));

    delay(1500);
  }
  else
  {
    //The module did not respond so assume it's just been powered on and communicating at 115200bps
    softSerial.begin(115200); //Start software serial at 115200

    nano.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg

    softSerial.begin(baudRate); //Start the software serial port, this time at user's chosen baud rate

    delay(250);
  }

  //Test the connection
  nano.getVersion();
  if (nano.msg[0] != ALL_GOOD) return (false); //Something is not right

  //The M6E has these settings no matter what
  nano.setTagProtocol(); //Set protocol to GEN2

  nano.setAntennaPort(); //Set TX/RX antenna ports to 1

  return (true); //We are ready to rock
}

//WS2812 Driver Function
void RGB_update(int LED, byte RED, byte GREEN, byte BLUE) {
  // LED is the LED number starting with 0
  // RED, GREEN, BLUE is the brightness 0..255 setpoint for that LED
  byte ExistingPort, WS2812pinHIGH;//local variables here to speed up pinWrites

  if (LED >= 0) { //map the REG GREEN BLUE Values into the RGB[] array
    RGB[LED * 3] = GREEN;
    RGB[LED * 3 + 1] = RED;
    RGB[LED * 3 + 2] = BLUE;
  }

  noInterrupts();//kill the interrupts while we send the bit stream out...
  ExistingPort = PORTB; // save the status of the entire PORT B - let's us write to the entire port without messing up the other pins on that port
  WS2812pinHIGH = PORTB | 1; //this gives us a byte we can use to set the whole PORTB with the WS2812 pin HIGH
  int bitStream = numberOfLEDs * 3;//total bytes in the LED string

  //This for loop runs through all of the bits (8 at a time) to set the WS2812 pin ON/OFF times
  for (int i = bitStream - 1; i >= 0; i--) {

    PORTB = WS2812pinHIGH;//bit 7  first, set the pin HIGH - it always goes high regardless of a 0/1

    //here's the tricky part, check if the bit in the byte is high/low then right that status to the pin
    // (RGB[i] & B10000000) will strip away the other bits in RGB[i], so here we'll be left with B10000000 or B00000000
    // then it's easy to check if the bit is high or low by AND'ing that with the bit mask ""&& B10000000)"" this gives 1 or 0
    // if it's a 1, we'll OR that with the Existing port, thus keeping the pin HIGH, if 0 the pin is written LOW
    PORTB = ((RGB[i] & B10000000) && B10000000) | ExistingPort;
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");//these are NOPS - these let us delay clock cycles for more precise timing
    PORTB = ExistingPort;//okay, here we know we have to be LOW regardless of the 0/1 bit state
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");//minimum LOW time for pin regardless of 0/1 bit state

    // then do it again for the next bit and so on... see the last bit though for a slight change

    PORTB = WS2812pinHIGH;//bit 6
    PORTB = ((RGB[i] & B01000000) && B01000000) | ExistingPort;
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    PORTB = ExistingPort;
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");

    PORTB = WS2812pinHIGH;//bit 5
    PORTB = ((RGB[i] & B00100000) && B00100000) | ExistingPort;
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    PORTB = ExistingPort;
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");

    PORTB = WS2812pinHIGH;//bit 4
    PORTB = ((RGB[i] & B00010000) && B00010000) | ExistingPort;
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    PORTB = ExistingPort;
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");

    PORTB = WS2812pinHIGH;//bit 3
    PORTB = ((RGB[i] & B00001000) && B00001000) | ExistingPort;
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    PORTB = ExistingPort;
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");

    PORTB = WS2812pinHIGH;//bit 2
    PORTB = ((RGB[i] & B00000100) && B00000100) | ExistingPort;
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    PORTB = ExistingPort;
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");

    PORTB = WS2812pinHIGH;//bit 1
    PORTB = ((RGB[i] & B00000010) && B00000010) | ExistingPort;
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    PORTB = ExistingPort;
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");

    PORTB = WS2812pinHIGH;//bit 0
    __asm__("nop\n\t");//on this last bit, the check is much faster, so had to add a NOP here
    PORTB = ((RGB[i] & B00000001) && B00000001) | ExistingPort;
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    PORTB = ExistingPort;//note there are no NOPs after writing the pin LOW, this is because the FOR Loop uses clock cycles that we can use instead of the NOPS
  }//for loop


  interrupts();//enable the interrupts

  // all done!
}//void RGB_update
