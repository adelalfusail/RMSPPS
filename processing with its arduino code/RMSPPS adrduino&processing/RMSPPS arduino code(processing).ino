#include <MAX30003.h>

//////////////////////////////////////////////////////////////////////////////////////////
//
//   AFE44xx Arduino Firmware
//
//   Copyright (c) 2016 ProtoCentral
//
//   SpO2 computation based on original code from Maxim Integrated
//
//   This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//hhggggggggggggggggggggggggggggggggggggggggggggg
//   For information on how to use the HealthyPi, visit https://github.com/Protocentral/afe44xx_Oximeter
/////////////////////////////////////////////////////////////////////////////////////////
#include <SoftwareSerial.h>
SoftwareSerial wifiSerial(10,11);
#include <string.h>
#include <SPI.h>
#include <math.h>

//afe44xx Register definition
#define CONTROL0    0x00
#define LED2STC     0x01
#define LED2ENDC    0x02
#define LED2LEDSTC    0x03
#define LED2LEDENDC   0x04
#define ALED2STC    0x05
#define ALED2ENDC   0x06
#define LED1STC     0x07
#define LED1ENDC    0x08
#define LED1LEDSTC    0x09
#define LED1LEDENDC   0x0a
#define ALED1STC    0x0b
#define ALED1ENDC   0x0c
#define LED2CONVST    0x0d
#define LED2CONVEND   0x0e
#define ALED2CONVST   0x0f
#define ALED2CONVEND  0x10
#define LED1CONVST    0x11
#define LED1CONVEND   0x12
#define ALED1CONVST   0x13
#define ALED1CONVEND  0x14
#define ADCRSTCNT0    0x15
#define ADCRSTENDCT0  0x16
#define ADCRSTCNT1    0x17
#define ADCRSTENDCT1  0x18
#define ADCRSTCNT2    0x19
#define ADCRSTENDCT2  0x1a
#define ADCRSTCNT3    0x1b
#define ADCRSTENDCT3  0x1c
#define PRPCOUNT    0x1d
#define CONTROL1    0x1e
#define SPARE1      0x1f
#define TIAGAIN     0x20
#define TIA_AMB_GAIN  0x21
#define LEDCNTRL    0x22
#define CONTROL2    0x23
#define SPARE2      0x24
#define SPARE3      0x25
#define SPARE4      0x26
#define SPARE4      0x26
#define RESERVED1   0x27
#define RESERVED2   0x28
#define ALARM     0x29
#define LED2VAL     0x2a
#define ALED2VAL    0x2b
#define LED1VAL     0x2c
#define ALED1VAL    0x2d
#define LED2ABSVAL    0x2e
#define LED1ABSVAL    0x2f
#define DIAG      0x30
#define count 60

#define CES_CMDIF_PKT_START_1   0x0A
#define CES_CMDIF_PKT_START_2   0xFA
#define CES_CMDIF_TYPE_DATA   0x02
#define CES_CMDIF_PKT_STOP    0x0B
#define AFE4490_START_PIN 5
//#define MISO 12
//#define SCK 13
//#define MOSI 11
//int IRheartsignal[count];
//int Redheartsignal[count];
int IRdc[count];
int Reddc[count];
double difIRheartsig_dc;
double difREDheartsig_dc;
double powdifIR;
double powdifRed;
double IRac;
double Redac;
double SpOpercentage;
double Ratio;
#define RESET 44
#define SPISTE  7// chip select
#define SPIDRDY  2 // data ready pin
volatile int drdy_trigger = LOW;

void afe44xxInit (void);
void afe44xxWrite (uint8_t address, uint32_t data);
uint32_t afe44xxRead (uint8_t address);
signed long average_BPM( signed long );

volatile char DataPacketHeadera[6];
volatile char DataPacket[10];
volatile char DataPacketFooter[2];
int datalen = 19;
unsigned long time;

volatile static int SPI_RX_Buff_Count = 0;
volatile char *SPI_RX_Buff_Ptr;
volatile int afe44xx_data_ready = false;
volatile unsigned int pckt = 0, buff = 0, t = 0;
unsigned long ueegtemp = 0, ueegtemp2 = 0;
unsigned long IRtemp, REDtemp;
signed long seegtemp = 0, seegtemp2 = 0;
volatile int i;
uint8_t global_heart_rate = 0;


uint16_t aun_ir_buffer[100]; //infrared LED sensor data
uint16_t aun_red_buffer[100];  //red LED sensor data

#define FS            25    //sampling frequency
#define BUFFER_SIZE  (FS*4)
#define MA4_SIZE  4 // DONOT CHANGE
#define min(x,y) ((x) < (y) ? (x) : (y))

const uint8_t uch_spo2_table[184] = { 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
                                      99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                                      100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
                                      97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
                                      90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
                                      80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
                                      66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
                                      49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
                                      28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
                                      3, 2, 1
                                    } ;

static  int32_t an_x[ BUFFER_SIZE];
static  int32_t an_y[ BUFFER_SIZE];

volatile int8_t n_buffer_count; //data length

int32_t n_spo2;  //SPO2 value
int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
int32_t n_heart_rate; //heart rate value
int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid

long status_byte = 0;
uint8_t LeadStatus = 0;
boolean leadoff_deteted = true;
uint8_t spo2_probe_open = false;
int dec = 0;
///////////////////////////for ECG ///////////////////
#include<SPI.h>
#include <TimerOne.h>
#include "MAX30003.h"

#define MAX30003_CS_PIN   53
#define CLK_PIN          45

volatile char SPI_RX_Buff[5] ;
//volatile char *SPI_RX_Buff_Ptr;
//int i=0;
unsigned long uintECGraw = 0;
signed long intECGraw = 0;
volatile uint8_t  DataPacketHeader[26];
uint8_t data_len = 19;
signed long ecgdata;
unsigned long data;

char SPI_temp_32b[4];
char SPI_temp_Burst[100];
signed long sresultTemp,sresultTempResp ;
long ECG;
unsigned int hravg;
////////////////////temp snsore include///////////////////
#include <Wire.h>
#include "ClosedCube_MAX30205.h"
unsigned char c=0;
int x=0;
ClosedCube_MAX30205 max30205;
/////////resive respartory varibals/////////

int serialRecive;
int resp_rate=0;
signed long resp_wave;
const int ress=22;
int ww = 0;
// 32KHz clock using timer1
//byte adel= B0000000000000100000000000;
void timerIsr()
{
  digitalWrite( CLK_PIN, digitalRead(CLK_PIN ) ^ 1 ); // toggle Digital6 attached to FCLK  of MAX30003
}



void setup()
{Serial3.begin(115200);
  Serial.begin(115200);
  //     SPI.begin();
   wifiSerial.begin(115200);
  // wifiSerial.println("AT+RST");
 //delay(50);
 
 wifiSerial.println("AT+CWMODE=2");
 delay(50);
 
  wifiSerial.println("AT+CIFSR");
  delay(50);
  
   wifiSerial.println("AT+CIPMUX=1");
   delay(50);
   
    wifiSerial.println("AT+CIPSERVER=1,80"); 
delay(100);
 
  

 pinMode(MAX30003_CS_PIN, OUTPUT);
  digitalWrite(MAX30003_CS_PIN, HIGH); //disable device
  pinMode(CLK_PIN, OUTPUT);
pinMode (SPISTE, OUTPUT); //Slave Select
pinMode (RESET,OUTPUT);//Slave Select
   digitalWrite(RESET, LOW);
   delay(500);
   digitalWrite(RESET, HIGH);
   delay(500); //*/
 //digitalWrite(SPISTE, HIGH);
  pinMode (SPIDRDY, INPUT); // data ready

 SPI.begin();
 SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  MAX30003_begin();   // initialize MAX30003

//SPI.usingInterrupt(0) ;
  attachInterrupt(0, afe44xx_drdy_event,  RISING ); // Digital2 is attached to Data ready pin of AFE is interrupt0 in ARduino

 


  afe44xxInit ();
   //pinMode (SPISTE, OUTPUT);
 //pinMode(MAX30003_CS_PIN, OUTPUT);
  //Serial.println("intilazition is done");
  max30205.begin(0x48);
  //SPI.end();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop()
{
 //digitalWrite   (CONTROL2, 0x000400);
 

 
  ////////////////////ECG^_^///////////
  /*MAX30003_Reg_Read(STATUS);
  Serial.print("data=");
   Serial.println((unsigned char)(SPI_temp_32b[0]));
   Serial.print("data1=");
   Serial.println((unsigned char)(SPI_temp_32b[1]));
   Serial.print("data3=");
   Serial.println((unsigned char)(SPI_temp_32b[2]));*/
  MAX30003_Reg_Read(ECG_FIFO);

  unsigned long data0 = (unsigned long) (SPI_temp_32b[0]);
  data0 = data0 << 24;
  unsigned long data1 = (unsigned long) (SPI_temp_32b[1]);
  data1 = data1 << 16;
  unsigned long data2 = (unsigned long) (SPI_temp_32b[2]);
  data2 = data2 >> 6;
  data2 = data2 & 0x03;

  data = (unsigned long) (data0 | data1 | data2);
  ecgdata = (signed long) (data);
ECG=ecgdata+134217728;
ECG=ECG/16;
  MAX30003_Reg_Read(RTOR);
  unsigned long RTOR_msb = (unsigned long) (SPI_temp_32b[0]);
 // RTOR_msb = RTOR_msb <<8;
 //Serial.println(RTOR_msb);
  unsigned char RTOR_lsb = (unsigned char) (SPI_temp_32b[1]);
 //Serial.println(RTOR_lsb);
  unsigned long rtor = (RTOR_msb << 8 | RTOR_lsb);
  rtor = ((rtor >> 2) & 0x3fff) ;
//Serial.println(rtor);
  float hr =  60 / ((float)rtor * 0.0095);
  unsigned int HR = (unsigned int)hr;  // type cast to int

  unsigned int RR = (unsigned int)rtor * 8 ; //8ms
  //Serial.println(ECG);
  if(x<=600){
hravg=hravg+HR;
//Serial.println(HR);
}else{
  hravg=hravg/600;
//Serial.println(hravg);
x=0;
hravg=0;

}
  //Serial.println(RR);
//Timer1.detachInterrupt();
  if (drdy_trigger == HIGH)
  {
    //detachInterrupt(0);
    afe44xxWrite(CONTROL0, 0x000001);
    IRtemp = afe44xxRead(LED1VAL);
    afe44xxWrite(CONTROL0, 0x000001);
    REDtemp = afe44xxRead(LED2VAL);
    afe44xx_data_ready = true;
  }

  if (afe44xx_data_ready == true)
  {

    IRtemp = (unsigned long) (IRtemp << 10);
    seegtemp = (signed long) (IRtemp);
    seegtemp = (signed long) (seegtemp >> 10);

    REDtemp = (unsigned long) (REDtemp << 10);
    seegtemp2 = (signed long) (REDtemp);
    seegtemp2 = (signed long) (seegtemp2 >> 10);

    if (dec == 10)
    {
      aun_ir_buffer[n_buffer_count] = (uint16_t) (seegtemp >> 4);
      aun_red_buffer[n_buffer_count] = (uint16_t) (seegtemp2 >> 4);
      n_buffer_count++;
      dec = 0;

    }
    dec++;

    if (n_buffer_count > 99)
    {
      // Serial.println("claculating sp02 hr");
      estimate_spo2(aun_ir_buffer, 100, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
      global_heart_rate = n_heart_rate;
   
     //Serial.print(n_spo2);
     //Serial.print("////////");
    //Serial.println(n_heart_rate);
      n_buffer_count = 0;
    
    }
  //   digitalWrite(AFE4490_START_PIN, LOW);
  
    



 
  
  

    if (n_spo2 == -999)
      DataPacket[8] = 0;
    else
      DataPacket[8] = n_spo2;
    DataPacket[9] = global_heart_rate;
 
       // unsigned long ecgfull=(ecgdata+ecgdata>>8+ecgdata>>16+ecgdata>>24)/4;
         
   

 
    afe44xx_data_ready = false;
    drdy_trigger = LOW;
   //SPI.usingInterrupt(0) ;
//detachInterrupt(0);
   
  }
  if (Serial3.available ()>0){
  // processInput ();
   serialRecive=Serial3.read ();
    //Serial.println(serialRecive);
   if(serialRecive>=50){
  //Serial.println("yes");
resp_wave=serialRecive;
   }else {//Serial.println("no");
resp_rate=serialRecive;

   }
  }
  if(n_spo2<=50||n_spo2==-999){
    n_spo2=0;
  }
c=max30205.readTemperature();
//Serial.println(c);
if(c==255){
c=0;
}
DataPacketHeader[0] = 0x0A;
  DataPacketHeader[1] = 0xFA;
  DataPacketHeader[2] = (uint8_t) (data_len);//0x0C;
  DataPacketHeader[3] = (uint8_t) (data_len>>8);//0;
  DataPacketHeader[4] = 0x02;

  DataPacketHeader[5] = ecgdata;
  DataPacketHeader[6] = ecgdata >> 8;
  DataPacketHeader[7] = ecgdata >> 16;
  DataPacketHeader[8] = ecgdata >> 24;

 DataPacketHeader[9] =  resp_wave ;
  DataPacketHeader[10] = resp_wave >> 8;
  DataPacketHeader[11] =resp_wave >> 16 ;
    DataPacketHeader[12] = resp_wave >> 24;
  
  
 /* DataPacketHeader[13] = HR ;
  DataPacketHeader[14] = HR >> 8;
  DataPacketHeader[15] = HR >> 16;
  DataPacketHeader[16] = HR >> 24;*/

   DataPacketHeader[24] = 0x00;
  DataPacketHeader[25] = 0x0b;

  
  DataPacketHeader[13] = seegtemp; 
  DataPacketHeader[14]  = seegtemp >> 8;
  DataPacketHeader[15] = seegtemp >> 16;
  DataPacketHeader[16] = seegtemp >> 24;
  
  DataPacketHeader[17]  = seegtemp2;  
  DataPacketHeader[18] = seegtemp2 >> 8;
  DataPacketHeader[19] = seegtemp2 >> 16;
  DataPacketHeader[20] = seegtemp2 >> 24;//*/

    DataPacketHeader[21] =c;
      DataPacketHeader[22] =n_spo2;
        DataPacketHeader[23] =resp_rate;
 //Serial.println(HR);
 
    for(i=0; i<26; i++) // transmit the data
        {
          Serial.write(DataPacketHeader[i]);
          //delay(2);

         }//*/
          
         //HR=HR+100;
         //RR=RR+100;
         //n_spo2=n_spo2+100;
         //resp_wave=resp_wave+100;
         //resp_rate=resp_rate+100;

/*wifiSerial.println("AT+CIPSEND=0,35");
//Serial.println(n_spo2);
wifiSerial.print(ECG );
//Serial.print(ECG);
wifiSerial.print(":");
wifiSerial.print(seegtemp );

 



 

wifiSerial.print(":");
 //wifiSerial.print(HR);
//wifiSerial.print(":");
//wifiSerial.print(RR);
//wifiSerial.print(":");
if(n_spo2==0){
wifiSerial.print("---");
}
else{
wifiSerial.print(n_spo2);
}
wifiSerial.print(":");
wifiSerial.print(resp_wave);
wifiSerial.print(":");
if(resp_rate==0){
  wifiSerial.print("---");
}else{
wifiSerial.print(resp_rate);
}
wifiSerial.print(":");
if(c==0){
   wifiSerial.println("---");
}else{
wifiSerial.println(c);
}
wifiSerial.println(":");
wifiSerial.println(":");
wifiSerial.println(":");
wifiSerial.println(":");
wifiSerial.println(":");
wifiSerial.println(":");
wifiSerial.println(":");
wifiSerial.println(":");
wifiSerial.println(":");//*/
  attachInterrupt(0, afe44xx_drdy_event, RISING );
// Timer1.attachInterrupt( timerIsr );
 x++;

}

//////////////////ECG FUKEN FUNCTIONS////////////////////////////
void MAX30003_Reg_Write (unsigned char WRITE_ADDRESS, unsigned long data)
{

  // now combine the register address and the command into one byte:
  byte dataToSend = (WRITE_ADDRESS << 1) | WREG;
//   SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  // take the chip select low to select the device:
   //pinMode (SPISTE, INPUT);
  digitalWrite(MAX30003_CS_PIN, LOW);

 // delay(10);
  SPI.transfer(dataToSend);   //Send register location
  SPI.transfer(data >> 16);   //number of register to wr
  SPI.transfer(data >> 8);    //number of register to wr
  SPI.transfer(data);      //Send value to record into register
  // delay(1);

  // take the chip select high to de-select:
  digitalWrite(MAX30003_CS_PIN, HIGH);
 // SPI.endTransaction();
}

void max30003_sw_reset(void)
{
  MAX30003_Reg_Write(SW_RST, 0x000000);
  //delay(100);
}

void max30003_synch(void)
{
  MAX30003_Reg_Write(SYNCH, 0x000000);
}

void MAX30003_Reg_Read(uint8_t Reg_address)
{
  uint8_t SPI_TX_Buff;
 // SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  //  Timer1.attachInterrupt( timerIsr );
 // SPI.usingInterrupt(timerIsr) ;
  digitalWrite(MAX30003_CS_PIN, LOW);
   //pinMode (SPISTE, INPUT);

  SPI_TX_Buff = (Reg_address << 1 ) | RREG;
  SPI.transfer(SPI_TX_Buff); //Send register location

  for ( i = 0; i < 3; i++)
  {
    SPI_temp_32b[i] = SPI.transfer(0xff);
  }

  digitalWrite(MAX30003_CS_PIN, HIGH);
 // SPI.endTransaction();

}

void MAX30003_Read_Data(int num_samples)
{
  uint8_t SPI_TX_Buff;
  //SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  //Timer1.attachInterrupt( timerIsr );
 // SPI.usingInterrupt(timerIsr ) ;
  digitalWrite(MAX30003_CS_PIN, LOW);

  SPI_TX_Buff = (ECG_FIFO_BURST << 1 ) | RREG;
  SPI.transfer(SPI_TX_Buff); //Send register location

  for ( i = 0; i < num_samples * 3; ++i)
  {
    SPI_temp_Burst[i] = SPI.transfer(0x00);
  }

  digitalWrite(MAX30003_CS_PIN, HIGH);
  //SPI.endTransaction();

}

void MAX30003_begin()
{ 
  //Start CLK timer
  Timer1.initialize(20);              // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
 //SPI.usingInterrupt(timerIsr) ;
  // attachInterrupt(digitalPinToInterrupt(20),timerIsr ,RISING);
  //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  Timer1.attachInterrupt( timerIsr );
 //  Timer1.attachInterrupt( timerIsr );

 
  max30003_sw_reset();
  // delay(100);
  MAX30003_Reg_Write(CNFG_GEN, 0x080000);//081007
  // delay(100);
  MAX30003_Reg_Write(CNFG_CAL, 0x720000);  // 720000,,,,,0x700000
  // delay(100);
  MAX30003_Reg_Write(CNFG_EMUX, 0x0E0000);//0B0000
  // delay(100);
  MAX30003_Reg_Write(CNFG_ECG, 0x805000);  // d23 - d22 : 10 for 250sps , 00:500 sps
  // delay(100);


  MAX30003_Reg_Write(CNFG_RTOR1, 0x3fc600);
  max30003_synch();
  //  digitalWrite   (CONTROL2,0x000000);
   //delay(10);
 // SPI.endTransaction();
 
  // Timer1.detachInterrupt();

}
////////////////SPO2 FUKEN FUNCTIONS////////////////////////////
///////// Gets Fired on DRDY event/////////////////////////////

void afe44xx_drdy_event()
{
  drdy_trigger = HIGH;
}

////////////////AFE44xx initialization//////////////////////////////////////////
void afe44xxInit (void)
{ //SPI.usingInterrupt(0);
 // SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  //  Serial.println("afe44xx Initialization Starts");
// pinMode(MAX30003_CS_PIN, INPUT);
 //pinMode (SPISTE, OUTPUT);
 // digitalWrite(AFE4490_START_PIN, HIGH);
 //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  afe44xxWrite(CONTROL0, 0x000000);

  afe44xxWrite(CONTROL0, 0x000008);

  afe44xxWrite(TIAGAIN, 0x000000); // CF = 5pF, RF = 500kR
  afe44xxWrite(TIA_AMB_GAIN, 0x000001);

  afe44xxWrite(LEDCNTRL, 0x001414);//try 000000 
  afe44xxWrite(CONTROL2, 0x000000); // LED_RANGE=100mA, LED=50mA,try d11:1 for pushpull 0 for bridge
  afe44xxWrite(CONTROL1, 0x010707); // Timers ON, average 3 samples
  // DIGOUT_TRISTATE
  afe44xxWrite(PRPCOUNT, 0X001F3F);

  afe44xxWrite(LED2STC, 0X001770);
  afe44xxWrite(LED2ENDC, 0X001F3E);
  afe44xxWrite(LED2LEDSTC, 0X001770);
  afe44xxWrite(LED2LEDENDC, 0X001F3F);
  afe44xxWrite(ALED2STC, 0X000000);
  afe44xxWrite(ALED2ENDC, 0X0007CE);
  afe44xxWrite(LED2CONVST, 0X000002);
  afe44xxWrite(LED2CONVEND, 0X0007CF);
  afe44xxWrite(ALED2CONVST, 0X0007D2);
  afe44xxWrite(ALED2CONVEND, 0X000F9F);

  afe44xxWrite(LED1STC, 0X0007D0);
  afe44xxWrite(LED1ENDC, 0X000F9E);
  afe44xxWrite(LED1LEDSTC, 0X0007D0);
  afe44xxWrite(LED1LEDENDC, 0X000F9F);
  afe44xxWrite(ALED1STC, 0X000FA0);
  afe44xxWrite(ALED1ENDC, 0X00176E);
  afe44xxWrite(LED1CONVST, 0X000FA2);
  afe44xxWrite(LED1CONVEND, 0X00176F);
  afe44xxWrite(ALED1CONVST, 0X001772);
  afe44xxWrite(ALED1CONVEND, 0X001F3F);

  afe44xxWrite(ADCRSTCNT0, 0X000000);
  afe44xxWrite(ADCRSTENDCT0, 0X000000);
  afe44xxWrite(ADCRSTCNT1, 0X0007D0);
  afe44xxWrite(ADCRSTENDCT1, 0X0007D0);
  afe44xxWrite(ADCRSTCNT2, 0X000FA0);
  afe44xxWrite(ADCRSTENDCT2, 0X000FA0);
  afe44xxWrite(ADCRSTCNT3, 0X001770);
  afe44xxWrite(ADCRSTENDCT3, 0X001770);

  //delay(40);
  // Serial.println("afe44xx Initialization Done");
 //SPI.endTransaction();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void afe44xxWrite (uint8_t address, uint32_t data)
{ //SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite (SPISTE, LOW);// enable device
  // delay(40);
  SPI.transfer (address); // send address to device
  SPI.transfer ((data >> 16) & 0xFF); // write top 8 bits
  SPI.transfer ((data >> 8) & 0xFF); // write middle 8 bits
  SPI.transfer (data & 0xFF); // write bottom 8 bits
 // delay(20);
  digitalWrite (SPISTE, HIGH); // disable device
 //SPI.endTransaction();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long afe44xxRead (uint8_t address)
{
  unsigned long data = 0;
  //SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
 // SPI.usingInterrupt(0);
  digitalWrite (SPISTE, LOW); // enable device
  SPI.transfer (address); // send address to device
  //SPI.transfer (data);
  data |= ((unsigned long)SPI.transfer (0) << 16); // read top 8 bits data
  data |= ((unsigned long)SPI.transfer (0) << 8); // read middle 8 bits  data
  data |= SPI.transfer (0); // read bottom 8 bits data
  digitalWrite (SPISTE, HIGH); // disable device

  //SPI.endTransaction();

  return data; // return with 24 bits of read data
}


void estimate_spo2(uint16_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint16_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid)
{
  uint32_t un_ir_mean, un_only_once ;
  int32_t k, n_i_ratio_count;
  int32_t i, s, m, n_exact_ir_valley_locs_count, n_middle_idx;
  int32_t n_th1, n_npks, n_c_min;
  int32_t an_ir_valley_locs[15] ;
  int32_t n_peak_interval_sum;

  int32_t n_y_ac, n_x_ac;
  int32_t n_spo2_calc;
  int32_t n_y_dc_max, n_x_dc_max;
  int32_t n_y_dc_max_idx, n_x_dc_max_idx;
  int32_t an_ratio[5], n_ratio_average;
  int32_t n_nume, n_denom ;

  // calculates DC mean and subtract DC from ir
  un_ir_mean = 0;
  for (k = 0 ; k < n_ir_buffer_length ; k++ ) un_ir_mean += pun_ir_buffer[k] ;
  un_ir_mean = un_ir_mean / n_ir_buffer_length ;

  // remove DC and invert signal so that we can use peak detector as valley detector
  for (k = 0 ; k < n_ir_buffer_length ; k++ )
    an_x[k] = -1 * (pun_ir_buffer[k] - un_ir_mean) ;

  // 4 pt Moving Average
  for (k = 0; k < BUFFER_SIZE - MA4_SIZE; k++) {
    an_x[k] = ( an_x[k] + an_x[k + 1] + an_x[k + 2] + an_x[k + 3]) / (int)4;
  }
  // calculate threshold
  n_th1 = 0;
  for ( k = 0 ; k < BUFFER_SIZE ; k++) {
    n_th1 +=  an_x[k];
  }
  n_th1 =  n_th1 / ( BUFFER_SIZE);
  if ( n_th1 < 30) n_th1 = 30; // min allowed
  if ( n_th1 > 60) n_th1 = 60; // max allowed

  for ( k = 0 ; k < 15; k++) an_ir_valley_locs[k] = 0;
  // since we flipped signal, we use peak detector as valley detector
  find_peak( an_ir_valley_locs, &n_npks, an_x, BUFFER_SIZE, n_th1, 4, 15 );//peak_height, peak_distance, max_num_peaks
  n_peak_interval_sum = 0;
  if (n_npks >= 2) {
    for (k = 1; k < n_npks; k++) n_peak_interval_sum += (an_ir_valley_locs[k] - an_ir_valley_locs[k - 1] ) ;
    n_peak_interval_sum = n_peak_interval_sum / (n_npks - 1);
    *pn_heart_rate = (int32_t)( (FS * 60) / n_peak_interval_sum );
    *pch_hr_valid  = 1;
  }
  else  {
    *pn_heart_rate = -999; // unable to calculate because # of peaks are too small
    *pch_hr_valid  = 0;
  }

  //  load raw value again for SPO2 calculation : RED(=y) and IR(=X)
  for (k = 0 ; k < n_ir_buffer_length ; k++ )  {
    an_x[k] =  pun_ir_buffer[k] ;
    an_y[k] =  pun_red_buffer[k] ;
  }

  // find precise min near an_ir_valley_locs
  n_exact_ir_valley_locs_count = n_npks;

  //using exact_ir_valley_locs , find ir-red DC andir-red AC for SPO2 calibration an_ratio
  //finding AC/DC maximum of raw

  n_ratio_average = 0;
  n_i_ratio_count = 0;
  for (k = 0; k < 5; k++) an_ratio[k] = 0;
  for (k = 0; k < n_exact_ir_valley_locs_count; k++) {
    if (an_ir_valley_locs[k] > BUFFER_SIZE ) {
      *pn_spo2 =  -999 ; // do not use SPO2 since valley loc is out of range
      *pch_spo2_valid  = 0;
      return;
    }
  }
  // find max between two valley locations
  // and use an_ratio betwen AC compoent of Ir & Red and DC compoent of Ir & Red for SPO2
  for (k = 0; k < n_exact_ir_valley_locs_count - 1; k++) {
    n_y_dc_max = -16777216 ;
    n_x_dc_max = -16777216;
    if (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k] > 3) {
      for (i = an_ir_valley_locs[k]; i < an_ir_valley_locs[k + 1]; i++) {
        if (an_x[i] > n_x_dc_max) {
          n_x_dc_max = an_x[i];
          n_x_dc_max_idx = i;
        }
        if (an_y[i] > n_y_dc_max) {
          n_y_dc_max = an_y[i];
          n_y_dc_max_idx = i;
        }
      }
      n_y_ac = (an_y[an_ir_valley_locs[k + 1]] - an_y[an_ir_valley_locs[k] ] ) * (n_y_dc_max_idx - an_ir_valley_locs[k]); //red
      n_y_ac =  an_y[an_ir_valley_locs[k]] + n_y_ac / (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k])  ;
      n_y_ac =  an_y[n_y_dc_max_idx] - n_y_ac;   // subracting linear DC compoenents from raw
      n_x_ac = (an_x[an_ir_valley_locs[k + 1]] - an_x[an_ir_valley_locs[k] ] ) * (n_x_dc_max_idx - an_ir_valley_locs[k]); // ir
      n_x_ac =  an_x[an_ir_valley_locs[k]] + n_x_ac / (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k]);
      n_x_ac =  an_x[n_y_dc_max_idx] - n_x_ac;     // subracting linear DC compoenents from raw
      n_nume = ( n_y_ac * n_x_dc_max) >> 7 ; //prepare X100 to preserve floating value
      n_denom = ( n_x_ac * n_y_dc_max) >> 7;
      if (n_denom > 0  && n_i_ratio_count < 5 &&  n_nume != 0)
      {
        an_ratio[n_i_ratio_count] = (n_nume * 100) / n_denom ; //formular is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;
        n_i_ratio_count++;
      }
    }
  }
  // choose median value since PPG signal may varies from beat to beat
  sort_ascend(an_ratio, n_i_ratio_count);
  n_middle_idx = n_i_ratio_count / 2;

  if (n_middle_idx > 1)
    n_ratio_average = ( an_ratio[n_middle_idx - 1] + an_ratio[n_middle_idx]) / 2; // use median
  else
    n_ratio_average = an_ratio[n_middle_idx ];

  if ( n_ratio_average > 2 && n_ratio_average < 184) {
    n_spo2_calc = uch_spo2_table[n_ratio_average] ;
    *pn_spo2 = n_spo2_calc ;
    *pch_spo2_valid  = 1;//  float_SPO2 =  -45.060*n_ratio_average* n_ratio_average/10000 + 30.354 *n_ratio_average/100 + 94.845 ;  // for comparison with table
  }
  else {
    *pn_spo2 =  -999 ; // do not use SPO2 since signal an_ratio is out of range
    *pch_spo2_valid  = 0;
  }
}

void find_peak( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num )
/**
  \brief        Find peaks
  \par          Details
                Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE

  \retval       None
*/
{
  find_peak_above( pn_locs, n_npks, pn_x, n_size, n_min_height );
  remove_close_peaks( pn_locs, n_npks, pn_x, n_min_distance );
  *n_npks = min( *n_npks, n_max_num );
}

void find_peak_above( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height )
/**
  \brief        Find peaks above n_min_height
  \par          Details
                Find all peaks above MIN_HEIGHT

  \retval       None
*/
{
  int32_t i = 1, n_width;
  *n_npks = 0;

  while (i < n_size - 1) {
    if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i - 1]) {   // find left edge of potential peaks
      n_width = 1;
      while (i + n_width < n_size && pn_x[i] == pn_x[i + n_width]) // find flat peaks
        n_width++;
      if (pn_x[i] > pn_x[i + n_width] && (*n_npks) < 15 ) {   // find right edge of peaks
        pn_locs[(*n_npks)++] = i;
        // for flat peaks, peak location is left edge
        i += n_width + 1;
      }
      else
        i += n_width;
    }
    else
      i++;
    //  Serial.println("beat");
  }
}

void remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance)
/**
  \brief        Remove peaks
  \par          Details
                Remove peaks separated by less than MIN_DISTANCE

  \retval       None
*/
{

  int32_t i, j, n_old_npks, n_dist;

  /* Order peaks from large to small */
  sort_indices_descend( pn_x, pn_locs, *pn_npks );

  for ( i = -1; i < *pn_npks; i++ ) {
    n_old_npks = *pn_npks;
    *pn_npks = i + 1;
    for ( j = i + 1; j < n_old_npks; j++ ) {
      n_dist =  pn_locs[j] - ( i == -1 ? -1 : pn_locs[i] ); // lag-zero peak of autocorr is at index -1
      if ( n_dist > n_min_distance || n_dist < -n_min_distance )
        pn_locs[(*pn_npks)++] = pn_locs[j];
    }
  }

  // Resort indices int32_to ascending order
  sort_ascend( pn_locs, *pn_npks );
}

void sort_ascend(int32_t  *pn_x, int32_t n_size)
/**
  \brief        Sort array
  \par          Details
                Sort array in ascending order (insertion sort algorithm)

  \retval       None
*/
{
  int32_t i, j, n_temp;
  for (i = 1; i < n_size; i++) {
    n_temp = pn_x[i];
    for (j = i; j > 0 && n_temp < pn_x[j - 1]; j--)
      pn_x[j] = pn_x[j - 1];
    pn_x[j] = n_temp;
  }
}

void sort_indices_descend(  int32_t  *pn_x, int32_t *pn_indx, int32_t n_size)
/**
  \brief        Sort indices
  \par          Details
                Sort indices according to descending order (insertion sort algorithm)

  \retval       None
*/
{
  int32_t i, j, n_temp;
  for (i = 1; i < n_size; i++) {
    n_temp = pn_indx[i];
    for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j - 1]]; j--)
      pn_indx[j] = pn_indx[j - 1];
    pn_indx[j] = n_temp;
  }
}

