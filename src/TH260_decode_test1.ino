#include <Arduino.h>

/*
Thermo sensor Sencor T260 reader via RXB6 receiver
Initial version for use with functions, preparation for develop T25 library
Output to serial console
Miklik, 23.10.2016
v0.0.1 - copy from T25 test app
*/
#define IRQ_PIN 2             // RF input with irq
#define SENSCOUNT 3           // Sensors count
#define START  8000           // start space time
//#define PAUSE  8000           // space between sentences
#define BIT1   4000           // 1
#define BIT0   2000           // 0
#define IMPULSE 500           // impulse
#define ITRESHOLD 50          // impulse treshold +- for duration
#define STRESHOLD 200          // space treshold +- for duration
#define INTERCEPT -68.5045250691  // number for decode temperature t = SLOPE * v + INTERCEPT
#define SLOPE 0.056013            // number for decode temperature
#define BUFF_ROW 3             // row in buffer for data
#define DATA_LONG 42           // all data bits
#define DATA_START 2           // start data bits position
#define CSUM_START 34          // first bit of checksum
uint32_t data[BUFF_ROW] = {0,0,0};  // data buffer
byte cdata[BUFF_ROW] = {0,0,0};     // data buffer
// masks for data decode
#define SID_MASK 0xFF000000    // mask sensore id bits 25-32
#define SID_SHIFT 24           // move SID bits to low bits
byte SID = 0;                  // sensore ID
#define BAT_MASK 0x00C00000    // mask for batery status bits 23-24
#define BAT_SHIFT 22           // move BAT bits to low bits
byte BATT = 0;                 // batery status
#define CHA_MASK 0x00300000    // mask for chanel bits 21-22
#define CHA_SHIFT 20           // move chanel bits
byte CHANEL = 0;               // chanel
#define TEM_MASK 0x000FFF00    // mask for temperature 12b 9-20
#define TEM_SHIFT 8            // move temperature bits
float TEMP = 0;                // float temperature
#define HUM_MASK 0x000000FF    // mask for humidity 1-8
#define HUM_SHIFT 0            // move humidity bits
byte HUMI = 0;                 // humidity
#define SUM_MASK 0x000000FF    // mask for checksum 0-8
#define SUM_SHIFT 0            // move checksum bits
byte CSUM = 0;                 // checksum

byte start = 0;             // start data reading flag
boolean space = LOW;             // space is starting
boolean bit0 = LOW;
boolean bit1 = LOW;
byte bits = 0;                   // bits counter in data word
byte cbits = 0;                  // bits counter from checksum
byte repeat = 0;                 // counter for repeated reading
byte count = 0;                  // count temperature sentences
unsigned long lastTime = 0;      // variable for last time point

void setup()
{
  pinMode(IRQ_PIN, INPUT_PULLUP);    // input for RF receiver
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), handler, CHANGE); // IRQ handler for RF receiver
  Serial.println("Start");
}

void loop()
{
  if (repeat >= BUFF_ROW)                              // buffer is full
  {
    detachInterrupt(digitalPinToInterrupt(IRQ_PIN));   // stop IRQ
    //Serial.println(uint32_t(data[0] >> 32),BIN);
    Serial.print(data[0],BIN);
    Serial.print("-");
    Serial.println(cdata[0],BIN);
    //Serial.print(data[0]);
    //Serial.print("-");
    //Serial.println(cdata[0]);
    Serial.print("SID-");
    SID = getSID(data[0]);
    Serial.print(SID,BIN); //sensor ID
    Serial.print("-B-");
    BATT = getBits(data[0], BAT_MASK, BAT_SHIFT);
    Serial.print(BATT,BIN); //Battery
    Serial.print("-CH-");
    CHANEL = getChanel(data[0]);
    Serial.print(CHANEL); //CH
    Serial.print("-TEMP ");
    TEMP = getTemp(data[0]);
    Serial.print(TEMP,1);
    Serial.print(" : ");
    Serial.print(getVal(data[0]));
    Serial.print(" HUMI- ");
    HUMI = getHumi(data[0]);
    Serial.print(HUMI);
    Serial.print(" SUM-");
    CSUM = getSum(cdata[0]);
    Serial.print(data[0],HEX);
    Serial.print(' ');
    Serial.print(CSUM,HEX);


    Serial.println();
    resetTWord ();                                      // reset reading
    count++;                                            // reading counter
    delay(1000);                                        // wait 1s
    attachInterrupt(digitalPinToInterrupt(IRQ_PIN), handler, CHANGE); // IRQ handler // start IRQ again
  }
}

// return specified bits from sensor word
unsigned int getBits(uint32_t data, unsigned long mask, byte shift )
{
    return ((data & mask) >> shift);
}

// get sensor ID from data word
byte getSID(unsigned long data)
{
  return getBits(data, SID_MASK, SID_SHIFT);
}

// get sensor channel from data word
byte getChanel(unsigned long data)
{
  return getBits(data, CHA_MASK, CHA_SHIFT) + 1;
}

// get temperature value from data word
uint16_t getVal(unsigned long data)
{
  uint16_t t = getBits(data, TEM_MASK, TEM_SHIFT);
  t = (t >> 8) + (t & 0x0F0) + ((t & 0xF) << 8);
  return t;
}

// get temperature from data word
float getTemp(unsigned long data)
{
  uint16_t v = getVal(data);
  return ((SLOPE * (float)v) + INTERCEPT);
}

// get humidity from data word
byte getHumi(unsigned long data)
{
  byte h = getBits(data, HUM_MASK, HUM_SHIFT);
  h = (h >> 4) + (h << 4);
  return h;
}

// get checksum from data word
byte getSum(byte data)
{
  return data;
}


// IRQ handler for decode bits
void handler()
{
  unsigned long currentTime = micros();        // shot time
  int duration = currentTime - lastTime;       // calculate duration
  lastTime = currentTime;                      // memory curent time
  boolean state = digitalRead(IRQ_PIN);        // read input status
  if (!state)                                  // goes from HIGH to LOW
  {
      space = isImpuls(duration - IMPULSE, ITRESHOLD);     // if impuls time is valid
  }
  else if (space)                              // goes from LOW to HIGH
  {
    if (start < 2)                                // start flag is false
    {
      if(isImpuls(duration - START, STRESHOLD))  // start temperature data it was pause signal
      {
        start++;
      }
      else
      {
        newTWord();
        //start = 0;
        //bits = 0;
      }
    }
    else                                       // start flag is true
    {
      bit0 = isImpuls(duration - BIT0, STRESHOLD);  // is it bit 0
      bit1 = isImpuls(duration - BIT1, STRESHOLD);  // is it bit 1
      if(bit0 ^ bit1)                               // XOR / only one is true
      {
          if(bits >= DATA_START)          // skip first two bits
          {
              if (bits < CSUM_START)        // record data bits
              {
                  bit1 && bitSet(data[repeat],DATA_LONG - 9 - bits);   // write bit 1
                  bit0 && bitClear(data[repeat],DATA_LONG - 9 - bits); // write bit 0
              }
              else                          // record checksum bits
              {
                   bit1 && bitSet(cdata[repeat],DATA_LONG - bits - 1);   // write bit 1
                   bit0 && bitClear(cdata[repeat],DATA_LONG - bits - 1); // write bit 0
              }
          }
          bits++;                                              // increment bit counter
      }
      else                                          // both or nothing bit
      {
        resetTWord ();                              // reset reading
      }
      if(bits >= DATA_LONG)                         // last bit was reading
      {
        if(repeat > 0)                              // is it temp sentence hienr then 0
        {
          if (data[repeat] != data[repeat-1])       // current temp word is differnt than previous
          {
            resetTWord ();                          // reset reading
          }
        }
        repeat++;                                   // increment temp word reading
        newTWord();                                 // reset counters and flags
      }
    }
  }
}

void resetTWord ()                                  // reset reading
{
  newTWord();                                       // reset counter and flags
  repeat = 0;                                       // reset temp word counter
}

void newTWord ()                                    // reset temp word counter
{
  bits = 0;                                         // bits counter
  space = LOW;                                      // space is measured
  start = 0;                                      // starting data reading
}

bool isImpuls(int duration, byte TRESHOLD)          // is it valid impulse
{
    return (abs(duration) <= TRESHOLD);
}
