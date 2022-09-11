//  Packed Channels for Serial communication on ATmega168/328
//  Original code by Paulo Costa August 2013
//  This is free software. You can redistribute it and/or modify it under
//  the terms of Creative Commons Attribution 3.0 United States License. 
//  To view a copy of this license, visit http://creativecommons.org/licenses/by/3.0
//  or send a letter to Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, USA.


int8_t frameState;
char curChannel;
char frameHexData[4];

 
void sendHexNibble(byte b)
{
  if (b < 10) {
    Serial.write('0' + b);
  } else if (b < 16) {
    Serial.write('A' + (b - 10));
  }
}

void sendHexByte(byte b)
{
  sendHexNibble(b >> 4); 
  sendHexNibble(b & 0x0F); 
}


void sendChannel(unsigned char c, int16_t v)
{
  Serial.write(c); 
  sendHexByte(v >> 8);
  sendHexByte(v & 0xFF);
}

byte isHexNibble(char c)
{
  if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F')) return 1;
  else return 0;
}


byte HexNibbleToByte(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  else if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  else return 0;
}


void initChannelsStateMachine(void)
{
  frameState = -1;  
}

void channelsStateMachine(byte b)
{
  if (frameState == -1) {
    if (b >= 'G' && b <= 'Z') {
      frameState = 0;
      curChannel = b;
    }
  } else {
    if (isHexNibble(b)) {
      frameHexData[frameState] = b;
      frameState++;
    } else {
      frameState = -1;
    }
    
    if (frameState == 4) {
      int value = (HexNibbleToByte(frameHexData[0]) << 12) +
                  (HexNibbleToByte(frameHexData[1]) << 8)  +
                  (HexNibbleToByte(frameHexData[2]) << 4)  +
                   HexNibbleToByte(frameHexData[3]);

      processInFrame(curChannel, value);
      //sendChannel(curChannel, value);
      frameState = -1;
    } 
  }

  if (b >= 'g' && b <= 'z') {
    //sendChannel(b, prepareOutFrame(b));
    sendOut(b);
  }

     
}


