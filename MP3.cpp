#include "ProjSettings.h"

#include "MP3.h"
#include "SoftwareSerial.h"

#define PIN_MP3RX  11
#define PIN_MP3TX  10
SoftwareSerial mySerial(PIN_MP3RX,PIN_MP3TX);


void MP3::Init()
{
	mNumberOfTracks = 9;
	mVolume = 10;
	mActive = false;
	mySerial.begin (9600);
}

void MP3::Run()
{
  if (mActive)
  {
    unsigned long nowMs = millis();
    unsigned long diff = nowMs-mPrevTimeMs;

    if( diff>1000)    // check once a second
    {
      if (readPlayer()==0x3D)  // finished 
      {
        mActive = false;
      }
      mPrevTimeMs = millis();
    }
  }
}

void MP3::Start()
{
  mPrevTimeMs = millis();
  byte lTrack = 1 + (mPrevTimeMs % 9);
  execute(0x03, lTrack);
//  execute(0x03, 1);
  mActive = true;
}
void MP3::Stop()
{
  execute(0x16,0); 
  mActive = false;
}

void MP3::Next()
{
  execute(0x01,0);
  mActive = true;
}

bool MP3::IsActive()
{
  return mActive;
}

void MP3::SetVolume(byte aVolume)
{
  if(aVolume<0x30)
  {
    mVolume = aVolume;
    execute(0x06, aVolume); // Set the volume (0x00~0x30)
  }
}

void MP3::IncreaseVolume(byte aVolume)
{
	mVolume += aVolume;
	SetVolume(mVolume);
}

void MP3::execute(byte CMD, uint16_t Par)
// Excecute the command and parameters
{
  #define Start_Byte 0x7E
  #define Version_Byte 0xFF
  #define Command_Length 0x06
  #define End_Byte 0xEF
  #define Acknowledge 0x00 //Returns info with command 0x41 [0x01: info, 0x00: no info]

  // Calculate the checksum (2 bytes)
  uint16_t checksum = -(Version_Byte + Command_Length + CMD + Acknowledge + highByte(Par) + lowByte(Par));
  // Build the command line
  byte Command_line[10] = { Start_Byte, Version_Byte, Command_Length, CMD, Acknowledge,
  highByte(Par), lowByte(Par), highByte(checksum), lowByte(checksum), End_Byte};

  //Send the command line to the module
  //Serial.print(F("Sending: "));
  for (byte k=0; k<10; k++)
  {
    mySerial.write( Command_line[k]);
    //Serial.print( Command_line[k],HEX);
    //Serial.print(" ");
  }
  //Serial.println();
  delay(100);
}

byte MP3::readPlayer()
{
  byte lResult = 0;

  byte n=0;
  
  mySerial.setTimeout(100);
  while(mySerial.available())
  {
    byte lData = mySerial.read();
    if (n==3)
    {
      lResult = lData;
    }
//    if (n>=5 && n<=6)
//    {
//      lResult <<= 8;
//      lResult |= lData;
//    }
    n++;
  }
  return lResult;
}
