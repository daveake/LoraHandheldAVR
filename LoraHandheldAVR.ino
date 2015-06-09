// LoRa reciver with LCD 2-line display

#include <string.h>
#include <ctype.h>
#include <SPI.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

/*---------------------------------------------------*\
|                                                     |
|               Arduino A0 - Switch (other side GND)  |
|               Arduino  2 - RFM DIO5                 |
|               Arduino  3 - RFM DIO0                 |
|               Arduino  4 - LCD D4                   |
|               Arduino  5 - LCD D5                   |
|               Arduino  6 - LCD D6                   |
|               Arduino  7 - LCD D7                   |
|               Arduino  8 - LCD EN                   |
|               Arduino  9 - LCD RS                   |
|               Arduino 10 - RFM NSS                  |
|               Arduino 11 - RFM MOSI                 |
|               Arduino 12 - RFM MISO                 |
|               Arduino 13 - RFM CLK                  |
|                                                     |
\*---------------------------------------------------*/

const double Pi = 3.1415926;

// RFM98
int _slaveSelectPin = 10; 
String content = "";
char character;
int dio0 = 3;
int dio5 = 2;
byte currentMode = 0x81;
unsigned long LastPacketAt=0;
unsigned long UpdateTimeAt=0;
unsigned long UpdateRSSIAt=0;


#define REG_FIFO                    0x00
#define REG_FIFO_ADDR_PTR           0x0D 
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_OPMODE                  0x01
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_RSSI_PACKET             0x1A
#define REG_RSSI_CURRENT            0x1B
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_HOP_PERIOD              0x24

// MODES
// MODES
#define RF96_MODE_RX_CONTINUOUS     0x85
#define RF96_MODE_SLEEP             0x80
#define RF96_MODE_STANDBY           0x81

#define PAYLOAD_LENGTH              80

// Modem Config 1
#define EXPLICIT_MODE               0x00
#define IMPLICIT_MODE               0x01

#define ERROR_CODING_4_5            0x02
#define ERROR_CODING_4_6            0x04
#define ERROR_CODING_4_7            0x06
#define ERROR_CODING_4_8            0x08

#define BANDWIDTH_7K8               0x00
#define BANDWIDTH_10K4              0x10
#define BANDWIDTH_15K6              0x20
#define BANDWIDTH_20K8              0x30
#define BANDWIDTH_31K25             0x40
#define BANDWIDTH_41K7              0x50
#define BANDWIDTH_62K5              0x60
#define BANDWIDTH_125K              0x70
#define BANDWIDTH_250K              0x80
#define BANDWIDTH_500K              0x90

// Modem Config 2

#define SPREADING_6                 0x60
#define SPREADING_7                 0x70
#define SPREADING_8                 0x80
#define SPREADING_9                 0x90
#define SPREADING_10                0xA0
#define SPREADING_11                0xB0
#define SPREADING_12                0xC0

#define CRC_OFF                     0x00
#define CRC_ON                      0x04


// POWER AMPLIFIER CONFIG
#define REG_PA_CONFIG               0x09
#define PA_MAX_BOOST                0x8F
#define PA_LOW_BOOST                0x81
#define PA_MED_BOOST                0x8A
#define PA_MAX_UK                   0x88
#define PA_OFF_BOOST                0x00
#define RFO_MIN                     0x00

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23  // 0010 0011
#define LNA_OFF_GAIN                0x00

// GPS:
byte GPSBuffer[82];
byte GPSIndex=0;

// GPS Variables
unsigned long SendGPSConfig;
char GPS_Time[9] = "00:00:00";
unsigned int GPS_Latitude_Minutes, GPS_Longitude_Minutes;
double GPS_Latitude_Seconds, GPS_Longitude_Seconds;
char *GPS_LatitudeSign="";
char *GPS_LongitudeSign="";
double GPS_Longitude, GPS_Latitude;
unsigned int GPS_Altitude=0, MaximumAltitude=0, MaxAltitudeThisSentence=0;
byte GotGPSThisSentence=0;
unsigned int PreviousAltitude=0;
unsigned int GPS_Satellites=0;
unsigned int GPS_Speed=0;
unsigned int GPS_Direction=0;
long HAB_Altitude;
double HAB_Latitude, HAB_Longitude;
char LatitudeString[16], LongitudeString[16], RSSIString[6];
unsigned long ButtonChangedAt=0;
int ButtonState=1;
int Editing=0;
int ScreenNumber=0;
uint8_t Channel=18;
uint8_t LoraMode=0;

char Hex[] = "0123456789ABCDEF";

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(9, 8, 4, 5, 6, 7);

void setup()
{
  Serial.begin(9600);

  Serial.println("");
  Serial.println("LoRa Handheld Tracker");
  Serial.println("");
    
  pinMode(A0, INPUT);
  digitalWrite(A0, HIGH);

  pinMode( _slaveSelectPin, OUTPUT);
  pinMode(dio0, INPUT);
  pinMode(dio5, INPUT);
  
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);

  DisplayScreen();
 
  if ((EEPROM.read(0) == 'D') && (EEPROM.read(1) == 'A'))
  {
    Channel = EEPROM.read(2);
    LoraMode = EEPROM.read(3);
  }
  
  setupRFM98();
}

void loop()
{ 
  int NewButtonState;
  
  CheckGPS();
  
  CheckRx();
  
  UpdateDisplay();
  
  NewButtonState = digitalRead(A0);
  
  if (NewButtonState == ButtonState)
  {
    if (!NewButtonState && ButtonChangedAt)
    {
      // Button released.  How long was it pressed for?

      if ((millis() - ButtonChangedAt) >= 300)
      {
        // Long press.  Change value / mode within this screen
        ButtonChangedAt = 0;    // Disable any further processing of this press
        LongPress();
      }
    }
  }
  else
  {
    // Button position changed
    if (NewButtonState && ButtonChangedAt)
    {
      // Button released.  How long was it pressed for?

      if (((millis() - ButtonChangedAt) < 300) && ((millis() - ButtonChangedAt) > 50))
      {
        // Short press.  Changes screen or (if editing) current value
        ShortPress();
      }
    }
    
    ButtonState = NewButtonState;
    ButtonChangedAt = millis();
  }
}

void UpdateDisplay(void)
{
  switch(ScreenNumber)
  {
    case 1:  UpdateTelemetryScreen();    break;
  }
}
  
void UpdateTelemetryScreen(void)
{
  if (millis() >= UpdateRSSIAt)
  {
    int Bars, Bar;
    
    lcd.setCursor(0, 1);
    Bars = (readRegister(REG_RSSI_CURRENT) - 37) / 8;
    for (Bar=1; Bar<=6; Bar++)
    {
      if (Bar <= Bars)
      {
        lcd.print("\377");
      }
      else
      {
        lcd.print(" ");
      }
    }
    
    UpdateRSSIAt = millis() + 200;
  }
  
  if (LastPacketAt)
  {
    if (millis() >= UpdateTimeAt)
    {
       char Time[6];
       unsigned long TimeSincePacket;
       
       UpdateTimeAt = millis() + 1000;

       TimeSincePacket = millis() - LastPacketAt;    // ms
       TimeSincePacket /= 1000;                      // secs
       
       lcd.setCursor(0, 0);
       
       if (TimeSincePacket < 6000)
       {
         snprintf(Time, sizeof(Time), "%02u:%02u", (unsigned int)(TimeSincePacket / 60), (unsigned int)(TimeSincePacket % 60));
         lcd.print(Time);
       }
       else
       {
         lcd.print("**:**");
       }
    }
  }
}


void CheckGPS()
{
  int inByte;         // incoming serial byte
  
  // Check for GPS data
  while (Serial.available() > 0)
  {
    inByte = Serial.read();
    
    if (inByte != '$')
    {
      Serial.write(inByte);
    }

    if ((inByte =='$') || (GPSIndex >= 80))
    {
      GPSIndex = 0;
    }
    
    if (inByte != '\r')
    {
      GPSBuffer[GPSIndex++] = inByte;
    }
    
    if (inByte == '\n')
    {
      ProcessGPSLine();
      GPSIndex = 0;
    }
  }
}

byte GPSChecksumOK()
{
  byte XOR, i, c;
  
  XOR = 0;
  for (i = 1; i < (GPSIndex-4); i++)
  {
    c = GPSBuffer[i];
    XOR ^= c;
  }
  
  return (GPSBuffer[GPSIndex-4] == '*') && (GPSBuffer[GPSIndex-3] == Hex[XOR >> 4]) && (GPSBuffer[GPSIndex-2] == Hex[XOR & 15]);
}

void ProcessGPSLine()
{
  if (GPSChecksumOK())
  {
    if ((GPSBuffer[1] == 'G') && (GPSBuffer[2] == 'P') && (GPSBuffer[3] == 'R') && (GPSBuffer[4] == 'M') && (GPSBuffer[5] == 'C'))
    {
      ProcessGPRMCCommand();
    }
    else if ((GPSBuffer[1] == 'G') && (GPSBuffer[2] == 'P') && (GPSBuffer[3] == 'G') && (GPSBuffer[4] == 'G') && (GPSBuffer[5] == 'A'))
    {
      ProcessGPGGACommand();
      switch (ScreenNumber)
      {
        case 0:  DisplayGPSScreen();       break;
        case 2:  DisplayDirectionScreen(); break;
      }
    }
  }
}

void ProcessGPRMCCommand()
{
  int i, j, k, IntegerPart;
  double Divider;

  // $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
 
  Divider = 1;
  
  for (i=7, j=0, k=0; (i<GPSIndex) && (j<8); i++)
  {
    if (GPSBuffer[i] == ',')
    {
      j++;    // Segment index
      k=0;    // Index into target variable
      IntegerPart = 1;
    }
    else
    {
      switch (j)
      {
        case 0:
          // UTC Time
          if (k < 8)
          {
            GPS_Time[k++] = GPSBuffer[i];
            if ((k==2) || (k==5))
            {
              GPS_Time[k++] = ':';
            }
            GPS_Time[k] = 0;
          }
          break;  // Start bit
          
        case 1:
          // Validity
          if (GPSBuffer[i] == 'A')
          {
            // Message OK
            GPS_Latitude_Minutes = 0;
            GPS_Latitude_Seconds = 0;
            GPS_Longitude_Minutes = 0;
            GPS_Longitude_Seconds = 0;
            GPS_Speed = 0;
            GPS_Direction = 0;
            GPS_LongitudeSign = "-";  // new
          }
          else
          {
            exit;
          }  
          break;

        case 2:
          // Latitude
          if (k <= 1)
          {
            GPS_Latitude_Minutes = GPS_Latitude_Minutes * 10 + (int)(GPSBuffer[i] - '0');
            Divider = 1;
          }
          else if ( k != 4)
          {
            Divider = Divider * 10;
            GPS_Latitude_Seconds = GPS_Latitude_Seconds  + (double)(GPSBuffer[i] - '0') / Divider;
          }
          /*
          if (k < 9)
          {
            GPS_Latitude[k++] = GPSBuffer[i];
            GPS_Latitude[k] = 0;
          }
          */
          k++;
          break;  // Start bit

        case 3:
          // N or S
          if (k < 1)
          {
            // Latitude = GPS_Latitude_Minutes + GPS_Latitude_Seconds * 5 / 30000;
            if (GPSBuffer[i] == 'S')
            {
              GPS_LatitudeSign = "-";
            }
            else
            {
              GPS_LatitudeSign = "";
            }
          }
          break;  // Start bit

        case 4:
          // Longitude
          if (k <= 2)
          {
            GPS_Longitude_Minutes = GPS_Longitude_Minutes * 10 + (int)(GPSBuffer[i] - '0');
            Divider = 1;
          }
          else if ( k != 5)
          {
            Divider = Divider * 10;
            GPS_Longitude_Seconds = GPS_Longitude_Seconds + (double)(GPSBuffer[i] - '0') / Divider;
          }
          /*
          if (k < 10)
          {
            GPS_Longitude[k++] = GPSBuffer[i];
            GPS_Longitude[k] = 0;
          }
          */
          k++;
          break;  // Start bit

        case 5:
          // E or W
          // if (k < 1)
          {
            if (GPSBuffer[i] == 'E')
            {
              GPS_LongitudeSign = "";
            }
          }
          break;  // Start bit

        case 6:
          // Speed
          if (IntegerPart)
          {
            if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9'))
            {
              GPS_Speed = GPS_Speed * 10;
              GPS_Speed += (unsigned int)(GPSBuffer[i] - '0');
            }
            else
            {
              IntegerPart = 0;
            }
          }
          break;  // Start bit
          
        case 7:
          // Direction
          if (IntegerPart)
          {
            if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9'))
            {
              GPS_Direction = GPS_Direction * 10;
              GPS_Direction += (unsigned int)(GPSBuffer[i] - '0');
            }
            else
            {
              IntegerPart = 0;
            }
          }
          break;  // Start bit
          
        default:
          break;  
      }
    }
  }
  
  GPS_Latitude = GPS_Latitude_Minutes + GPS_Latitude_Seconds * 5 / 3;
  if (*GPS_LatitudeSign == '-') GPS_Latitude = -GPS_Latitude;
  GPS_Longitude = GPS_Longitude_Minutes + GPS_Longitude_Seconds * 5 / 3;
  if (*GPS_LongitudeSign == '-') GPS_Longitude = -GPS_Longitude;
}

void ProcessGPGGACommand()
{
  int i, j, k, IntegerPart;
  unsigned int Altitude;

  // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
  //                                               =====  <-- altitude in field 8
  
  IntegerPart = 1;
  GPS_Satellites = 0;
  Altitude = 0;
  
  for (i=7, j=0, k=0; (i<GPSIndex) && (j<9); i++)
  {
    if (GPSBuffer[i] == ',')
    {
      j++;    // Segment index
      k=0;    // Index into target variable
      IntegerPart = 1;
    }
    else
    {
      if (j == 6)
      {
        // Satellite Count
        if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9'))
        {
          GPS_Satellites = GPS_Satellites * 10;
          GPS_Satellites += (unsigned int)(GPSBuffer[i] - '0');
        }
      }
      else if (j == 8)
      {
        // Altitude
        if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9') && IntegerPart)
        {
          Altitude = Altitude * 10;
          Altitude += (unsigned int)(GPSBuffer[i] - '0');
        }
        else
        {
          IntegerPart = 0;
        }
      }
    }
  }

  if (GPS_Satellites >= 4)
  {
    if ((Altitude > 0) || (GPS_Altitude < 50))
    {
      GotGPSThisSentence = 1;
      
      if (Altitude > MaxAltitudeThisSentence)
      {
        MaxAltitudeThisSentence = Altitude;
      }
      
      GPS_Altitude = Altitude;
   
      if (GPS_Altitude > MaximumAltitude)
      {
        MaximumAltitude = GPS_Altitude;
      }
    }
  }
}

int receiveMessage(char *message)
{
  int i, Bytes, currentAddr;

  int x = readRegister(REG_IRQ_FLAGS);
  // printf("Message status = %02Xh\n", x);
  
  // clear the rxDone flag
  // writeRegister(REG_IRQ_FLAGS, 0x40); 
  writeRegister(REG_IRQ_FLAGS, 0xFF); 
   
  // check for payload crc issues (0x20 is the bit we are looking for
  if((x & 0x20) == 0x20)
  {
    // printf("CRC Failure %02Xh!!\n", x);
    // reset the crc flags
    writeRegister(REG_IRQ_FLAGS, 0x20); 
  }
  else
  {
    currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
    Bytes = readRegister(REG_RX_NB_BYTES);
    // printf ("%d bytes in packet\n", Bytes);

    // printf("RSSI = %d\n", readRegister(REG_RSSI) - 137);
	
    writeRegister(REG_FIFO_ADDR_PTR, currentAddr);   
    // now loop over the fifo getting the data
    for(i = 0; i < Bytes; i++)
    {
      message[i] = (unsigned char)readRegister(REG_FIFO);
    }
    message[Bytes] = '\0';
	
    // writeRegister(REG_FIFO_ADDR_PTR, 0);  // currentAddr);   
  } 
  
  return Bytes;
}

void CheckRx()
{
  if (digitalRead(dio0))
  {
    char Message[256], PayloadID[16], Time[16];
    int Bytes, SentenceCount;
    
    Bytes = receiveMessage(Message);
    
    sprintf(RSSIString, "%ddB", readRegister(REG_RSSI_PACKET) - 137);
    
    Serial.print("Packet size = "); Serial.println(Bytes);

    // Telemetry='$$LORA1,108,20:30:39,51.95027,-2.54445,00141,0,0,11*9B74
			
    if (sscanf(Message, "%[^,],%d,%[^,],%[^,],%[^,],%ld", PayloadID, &SentenceCount, Time, LatitudeString, LongitudeString, &HAB_Altitude) > 0)
    {
      LastPacketAt = millis();

      HAB_Latitude = atof(LatitudeString);
      HAB_Longitude = atof(LongitudeString);

      switch (ScreenNumber)
      {
        case 1:  DisplayTelemetryScreen();     break;
        case 2:  DisplayDirectionScreen();     break;
      }
      
      UpdateTimeAt = millis() + 10000;
      UpdateRSSIAt = millis() + 4000;
    }
  }
}

unsigned int CalculateCRC(char *Line)
{
  unsigned int CRC;
  unsigned int xPolynomial;
  int j;

  CRC = 0xffff;           // Seed
  xPolynomial = 0x1021;

  for (; *Line; Line++)
  {   // For speed, repeat calculation instead of looping for each bit
    CRC ^= (((unsigned int)*Line) << 8);
    for (j=0; j<8; j++)
    {
      if (CRC & 0x8000)
        CRC = (CRC << 1) ^ 0x1021;
      else
        CRC <<= 1;
    }
  } 

  return CRC;
}

/////////////////////////////////////
//    Method:   Change the mode
//////////////////////////////////////
void setMode(byte newMode)
{
  if(newMode == currentMode)
    return;  
  
  switch (newMode) 
  {
    case RF96_MODE_RX_CONTINUOUS:
      writeRegister(REG_PA_CONFIG, PA_OFF_BOOST);  // TURN PA OFF FOR RECIEVE??
      writeRegister(REG_LNA, LNA_MAX_GAIN);  // LNA_MAX_GAIN);  // MAX GAIN FOR RECIEVE
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      Serial.println("Changing to Receive Continuous Mode\n");
      break;
      
      break;
    case RF96_MODE_SLEEP:
      Serial.println("Changing to Sleep Mode"); 
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RF96_MODE_STANDBY:
      Serial.println("Changing to Standby Mode");
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    default: return;
  } 
  
  if(newMode != RF96_MODE_SLEEP)
  {
    Serial.println("Waiting for mode switch");
    while(digitalRead(dio5) == 0)
    {
    } 
    Serial.println("Mode switch done");
  }
   
  return;
}


/////////////////////////////////////
//    Method:   Read Register
//////////////////////////////////////

byte readRegister(byte addr)
{
  select();
  SPI.transfer(addr & 0x7F);
  byte regval = SPI.transfer(0);
  unselect();
  return regval;
}

/////////////////////////////////////
//    Method:   Write Register
//////////////////////////////////////

void writeRegister(byte addr, byte value)
{
  select();
  SPI.transfer(addr | 0x80); // OR address with 10000000 to indicate write enable;
  SPI.transfer(value);
  unselect();
}

/////////////////////////////////////
//    Method:   Select Transceiver
//////////////////////////////////////
void select() 
{
  digitalWrite(_slaveSelectPin, LOW);
}

/////////////////////////////////////
//    Method:   UNSelect Transceiver
//////////////////////////////////////
void unselect() 
{
  digitalWrite(_slaveSelectPin, HIGH);
}


void setLoRaMode()
{
  unsigned long FrequencyValue;
  double Frequency;

  Serial.println("Setting LoRa Mode");
  setMode(RF96_MODE_SLEEP);
  writeRegister(REG_OPMODE,0x80);
   
  setMode(RF96_MODE_SLEEP);

  Serial.print("Setting channel ");
  Serial.println(Channel);
  
  Frequency = 434000000.0 + (double)Channel * 25000.0;

  Serial.print("Frequency is ");
  Serial.println(Frequency);
  
  FrequencyValue = (unsigned long)Frequency;
  Serial.println(FrequencyValue);
  
  FrequencyValue *= 7110656;
  Serial.println(FrequencyValue);
  
  FrequencyValue /= 434;

  Serial.print("FrequencyValue is ");
  Serial.println(FrequencyValue);
  
  writeRegister(0x06, 0x6C);
  writeRegister(0x07, 0x9C);
  writeRegister(0x08, 0x8E);
  /*
  writeRegister(0x06, (FrequencyValue >> 16) & 0xFF);
  writeRegister(0x07, (FrequencyValue >> 8) & 0xFF);
  writeRegister(0x08, FrequencyValue & 0xFF);
  */
  
  Serial.println("LoRa Mode Set");
  
  Serial.print("Mode = "); Serial.println(readRegister(REG_OPMODE));
  
  return;
}

/////////////////////////////////////
//    Method:   Setup to receive continuously
//////////////////////////////////////
void startReceiving()
{
  if (LoraMode)
  {
    writeRegister(REG_MODEM_CONFIG, IMPLICIT_MODE | ERROR_CODING_4_5 | BANDWIDTH_20K8);
    writeRegister(REG_MODEM_CONFIG2, SPREADING_6);
    writeRegister(0x31, (readRegister(0x31) & 0xF8) | 0x05);
    writeRegister(0x37, 0x0C);
    // writeRegister(0x26, 0x0C);    // 0000 1 1 00
    writeRegister(REG_PAYLOAD_LENGTH, 255);
    writeRegister(REG_RX_NB_BYTES, 255);
    Serial.println("Set fast mode");
  }
  else
  {
    writeRegister(REG_MODEM_CONFIG, EXPLICIT_MODE | ERROR_CODING_4_8 | BANDWIDTH_20K8);
    writeRegister(REG_MODEM_CONFIG2, SPREADING_11 | CRC_ON);
    writeRegister(0x26, 0x0C);    // 0000 1 1 00
    writeRegister(REG_PAYLOAD_LENGTH, 80);
    writeRegister(REG_RX_NB_BYTES, 80);
    Serial.println("Set slow mode");
  }
  
  writeRegister(REG_HOP_PERIOD,0xFF);
  writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_BASE_AD));   
  
  // Setup Receive Continous Mode
  setMode(RF96_MODE_RX_CONTINUOUS); 
}

void setupRFM98(void)
{
  SPI.begin();
  
  // LoRa mode 
  setLoRaMode();
    
  Serial.println("Setup Complete");
  
  startReceiving();
}

void ShortPress(void)
{
  if (Editing)
  {
    switch(ScreenNumber)
    {
      case 3:  ShortPressFrequencyScreen();   break;
      case 4:  ShortPressModeScreen();        break;
    }
  }
  else
  {
    if (++ScreenNumber > 4)
    {
      ScreenNumber = 0;
    }
      
    DisplayScreen();
  }
}

void LongPress(void)
{
  if (ScreenNumber > 2)
  {
    Editing = !Editing;
  
    if (Editing)
    {
      DisplayEditScreen();
    }
    else
    {
      EEPROM.write(3, LoraMode);
      EEPROM.write(2, Channel);
      EEPROM.write(1, 'A');
      EEPROM.write(0, 'D');
      
      DisplayScreen();
    }
  }
}
  
void ShortPressFrequencyScreen(void)
{
  if (++Channel > 26)
  {
    Channel = 0;
  }
  
  setLoRaMode();
  startReceiving();
  
  DisplayChannel();
}

void ShortPressModeScreen(void)
{
  if (++LoraMode > 1)
  {
    LoraMode = 0;
  }

  setLoRaMode();
  startReceiving();
  
  DisplayLoraMode();
}


void DisplayScreen(void)
{ 
  switch(ScreenNumber)
  {
    case 0:  DisplayGPSScreen();         break;
    case 1:  DisplayTelemetryScreen();   break;
    case 2:  DisplayDirectionScreen();   break;
    case 3:  DisplayFrequencyScreen();   break;
    case 4:  DisplayModeScreen();        break;
  }
}

void DisplayEditScreen(void)
{
  lcd.clear();
  
  switch(ScreenNumber)
  {
    case 3:  DisplayFrequencyEditScreen();   break;
    case 4:  DisplayModeEditScreen();        break;
  }
}

void DisplayGPSScreen(void)
{
  char Temp[10];

  lcd.clear();
  
  lcd.print(GPS_Time+3);

  dtostrf(GPS_Latitude, 7, 5, Temp);
  lcd.setCursor(8, 0);
  lcd.print(Temp);

  lcd.setCursor(0, 1);
  lcd.print(GPS_Altitude);
  lcd.print("m");

  dtostrf(GPS_Longitude, 7, 5, Temp);
  lcd.setCursor(8, 1);
  lcd.print(Temp);
}
  
void DisplayTelemetryScreen(void)
{
  lcd.clear();
  
  if (LastPacketAt)
  {
    char AltitudeString[8];
    
    sprintf(AltitudeString, "%05ld", HAB_Altitude);
      
    lcd.print(AltitudeString);
    lcd.setCursor(8, 0);
    lcd.print(LatitudeString);
    lcd.setCursor(0, 1);
    lcd.print(RSSIString);
    lcd.setCursor(8, 1);
    lcd.print(LongitudeString);
  }
  else 
  {  
    lcd.print("Waiting for data");
  }
}

  
void DisplayDirectionScreen(void)
{
  double DistanceToHAB, DirectionToHAB;
  int Clock;
  long Distance;
  
  lcd.clear();
  if ((GPS_Satellites >= 3) && (LastPacketAt > 0))
  {
    // Have both positions so we can calculate distance and direction
    DistanceToHAB = CalculateDistance(HAB_Latitude, HAB_Longitude, GPS_Latitude, GPS_Longitude);
    Serial.println("");
    Serial.println(HAB_Latitude);
    Serial.println(HAB_Longitude);
    Serial.println(GPS_Latitude);
    Serial.println(GPS_Longitude);
    Serial.println(DistanceToHAB);
    Serial.println("");

    DirectionToHAB = CalculateDirection(HAB_Latitude, HAB_Longitude, GPS_Latitude, GPS_Longitude) - GPS_Direction;
    
    DistanceToHAB = floor(DistanceToHAB);
    Distance = DistanceToHAB;
    
    DirectionToHAB = floor(DirectionToHAB / 30 + 12.5);
    Clock = DirectionToHAB;
    Clock %= 12;

    
    lcd.print("Dir:  ");
    lcd.print(Clock);
    lcd.print(" o'clock");
    
    lcd.setCursor(0,1);
    lcd.print("Dist: ");
    lcd.print(Distance);
    lcd.print("m");
    
  }
  else
  {
    lcd.print("No direction ...");
    if (GPS_Satellites <= 3)
    {
      lcd.setCursor(0,1);
      lcd.print ("No GPS");
      Serial.println("No Gps\n");
    }
    if (LastPacketAt == 0)
    {
      lcd.setCursor(7,1);
      lcd.print ("No Packet");
    }
  }
}

void DisplayFrequencyScreen(void)
{
  lcd.clear();
  lcd.print("Hold to edit Frq");
  
  DisplayChannel();
}

void DisplayModeScreen(void)
{
  lcd.clear();
  lcd.print("Hold to chg Mode");
  
  DisplayLoraMode();
}

void DisplayFrequencyEditScreen(void)
{
  lcd.print("Press ^   Hold X");
  
  DisplayChannel();
}

void DisplayChannel(void)
{
  char Value[4];

  lcd.setCursor(0, 1);
  lcd.print("434.");
  sprintf(Value, "%03d", Channel * 25);
  lcd.print(Value);
  lcd.print("MHz");
}

void DisplayLoraMode(void)
{
  lcd.setCursor(0, 1);
  lcd.print(LoraMode ? "Fast Mode" : "Slow Mode");
}

void DisplayModeEditScreen(void)
{
  lcd.print("Press ^   Hold X");
  
  DisplayLoraMode();
}

double CalculateDistance(double flat1, double flon1, double flat2, double flon2)
{
  double dist_calc=0;
  double dist_calc2=0;
  double diflat=0;
  double diflon=0;

  //I've to spplit all the calculation in several steps. If i try to do it in a single line the arduino will explode.
  diflat=radians(flat2-flat1);
  flat1=radians(flat1);
  flat2=radians(flat2);
  diflon=radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

  dist_calc*=6371000.0; //Converting to meters
  //Serial.println(dist_calc);
  return dist_calc;  
}

double CalculateDirection(double HABLatitude, double HABLongitude, double CarLatitude, double CarLongitude)
{
    double x, y;

    HABLatitude = HABLatitude * Pi / 180;
    HABLongitude = HABLongitude * Pi / 180;
    CarLatitude = CarLatitude * Pi / 180;
    CarLongitude = CarLongitude * Pi / 180;

    y = sin(HABLongitude - CarLongitude) * cos(HABLatitude);
    x = cos(CarLatitude) * sin(HABLatitude) - sin(CarLatitude) * cos(HABLatitude) * cos(HABLongitude - CarLongitude);

    return atan2(y, x) * 180 / Pi;
}

