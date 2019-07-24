#include <Wire.h>
#include <SPI.h>
//#include <KX022.h>
#include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library
#include  <avr/dtostrf.h>
#include "SI114.h"
#include <BLEPeripheral.h>
#include "BLESerial.h"
BLESerial bleSerial(10, 2, 9);
unsigned long long lastSent = 0;
//setting up BLE
// define pins (varies per shield/board)
/*#define BLE_REQ   10
  #define BLE_RDY   2
  #define BLE_RST   9
*/

#define AMBIENT_LIGHT_SAMPLING
//#define GET_PULSE_READING
//#define SEND_TOTAL_TO_PROCESSING
//#define PRINT_LED_VALS
// #define PRINT_LED_VALS
// create peripheral instance, see pinouts above

//BLEPeripheral                    blePeripheral       = BLEPeripheral(BLE_REQ, BLE_RDY, BLE_RST);

// create service
//BLEService                       testService         = BLEService("fff0");
// create counter characteristic
//BLEUnsignedShortCharacteristic   testCharacteristic  = BLEUnsignedShortCharacteristic("fff1", BLERead | BLEWrite | BLEWriteWithoutResponse | BLENotify /*| BLEIndicate*/);
// create user description descriptor for characteristic
//BLEDescriptor                    testDescriptor      = BLEDescriptor("2901", "counter");


// last counter update time
//unsigned long long               lastSent            = 0;





#define OLED_WIDTH 64
#define OLED_HEIGHT 32
MicroOLED oled(OLED_RST, OLED_DC, OLED_CS); // (pin_rst, pin_dc, pin_cs)

float xyz[3];
uint32_t tPage;
bool B1_isPressed = false;
uint8_t page_num = 0;
const uint8_t page_count = 2;
void draw_page(uint8_t idx = 0);

//heartrate sensor si114

const int SAMPLES_TO_AVERAGE = 2;
int binOut;     // 1 or 0 depending on state of heartbeat

int BPM;

unsigned long red;        // read value from visible red LED  ------ only one that gives result

unsigned long IR1;        // read value from infrared LED1

unsigned long IR2J;       // read value from infrared LED2

unsigned long total;     // all three LED reads added together

int signalSize;          // the heartbeat signal minus the offset


const int portForSI114 = 0; //for SI114 JJ
PortI2C myBus (portForSI114);
PulsePlug pulse (myBus);

//void draw_page(uint8_t idx = 0);

void setup()
{

  
  pinMode(6, OUTPUT);

  Wire.begin();

  bleSerial.setLocalName("UART");
  bleSerial.begin();
  Serial.begin(9600);
  Serial.println(__FILE__);




  delay(1000); // show splash for 3s
  tPage = millis();


  initPulseSensor();
  /*
    blePeripheral.setLocalName("test-jj");
    #if 1
    blePeripheral.setAdvertisedServiceUuid(testService.uuid());
    #else
    const char manufacturerData[4] = {0x12, 0x34, 0x56, 0x78};
    blePeripheral.setManufacturerData(manufacturerData, sizeof(manufacturerData));
    #endif

    // set device name and appearance
    blePeripheral.setDeviceName("Test-jj");
    blePeripheral.setAppearance(0x0080);

    // add service, characteristic, and decriptor to peripheral
    blePeripheral.addAttribute(testService);
    blePeripheral.addAttribute(testCharacteristic);
    blePeripheral.addAttribute(testDescriptor);

    // assign event handlers for connected, disconnected to peripheral
    blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

    // assign event handlers for characteristic
    testCharacteristic.setEventHandler(BLEWritten, characteristicWritten);
    testCharacteristic.setEventHandler(BLESubscribed, characteristicSubscribed);
    testCharacteristic.setEventHandler(BLEUnsubscribed, characteristicUnsubscribed);

    // set initial value for characteristic
    testCharacteristic.setValue(0);

    // begin initialization
    blePeripheral.begin();
  */

}

void loop()
{
  
  digitalWrite(6,HIGH);
  delay(500);
  
 int test_vis =pulse.getReg(PulsePlug::ALS_VIS_DATA0)+256*pulse.getReg(PulsePlug::ALS_VIS_DATA1);
 // Serial.println(pulse.readParam(0x22), BIN);
// Serial.println(test_vis);
 //Serial.print("IR ");
  int test_ir =pulse.getReg(PulsePlug::ALS_IR_DATA0)+256*pulse.getReg(PulsePlug::ALS_IR_DATA1);
// Serial.println(test_ir);
//  readPulseSensorPSO2(); //heart rate sensor
//  bleSerial.print("  ");
  bleSerial.print("V");
  bleSerial.print(test_vis);
 // bleSerial.print(",");
  bleSerial.print("I");
  bleSerial.print(test_ir);
  digitalWrite(6,LOW);
  yield();
  delay(60000); // show splash for 3s

}




void float2chars(float &in, char (&out)[5])
{
  bool sign_bit = (in < 0);
  uint16_t tmp = sign_bit ? (-in * 10) : (in * 10);
  out[0] = (sign_bit) ? '-' : ' ';
  out[1] = char('0' + (tmp / 10));
  out[2] = '.';
  out[3] = char('0' + (tmp % 10));
  out[4] = '\0';
}

void draw_page(uint8_t idx)
{
  switch(idx)
  {
    case 1:
      page_accelerometer(); break;
    default:
      page_startup();
    break;
  }
}

void page_startup()
{
  oled.clear();
  //oled.drawString(0,0,"github.com/");
  //oled.drawString(0,10,"micooke");
//  oled.drawString(0,20,__TIME__);
  oled.display();
}



void page_accelerometer()
{
    char fltBuf[5];
    char charVal[10]; 
    //acc.getAccelXYZ(xyz);
     float test_vis =pulse.getReg(PulsePlug::ALS_VIS_DATA0)+256*pulse.getReg(PulsePlug::ALS_VIS_DATA1);
     float test_ir =pulse.getReg(PulsePlug::ALS_IR_DATA0)+256*pulse.getReg(PulsePlug::ALS_IR_DATA1);
    oled.clear();
    oled.setCursor(5, 4); // points cursor to x=27 y=0
    oled.print("Vis");
    oled.setCursor(25, 4); // points cursor to x=27 y=0
    dtostrf(test_vis,5,0,charVal);
    oled.print(charVal);
    Serial.println(charVal);
    
    oled.setCursor(5, 20); // points cursor to x=27 y=0
    oled.print("IR");
    oled.setCursor(25, 20); // points cursor to x=27 y=0
    float2chars(test_ir,fltBuf);
   dtostrf(test_ir,5,0,charVal);
    oled.print(charVal);
    Serial.println(charVal);
    oled.display();  
}

void initPulseSensor() {



  pulse.setReg(PulsePlug::HW_KEY, 0x17);

  // pulse.setReg(PulsePlug::COMMAND, PulsePlug::RESET_Cmd);



  Serial.print("PART: ");

  Serial.print(pulse.getReg(PulsePlug::PART_ID));

  Serial.print(" REV: ");

  Serial.print(pulse.getReg(PulsePlug::REV_ID));

  Serial.print(" SEQ: ");

  Serial.println(pulse.getReg(PulsePlug::SEQ_ID));



  pulse.setReg(PulsePlug::INT_CFG, 0x03);       // turn on interrupts

  // pulse.setReg(PulsePlug::IRQ_ENABLE, 0x10);    // turn on interrupt on PS3
  pulse.setReg(PulsePlug::IRQ_ENABLE, 0x0F);    // turn on interrupt on PS12 JJ
  // pulse.setReg(PulsePlug::IRQ_MODE2, 0x01);     // interrupt on ps3 measurement
  pulse.setReg(PulsePlug::IRQ_MODE1, 0x0F);     // interrupt on ps2 AND PS1 measurement
    pulse.setReg(PulsePlug::MEAS_RATE, 0x84);     // see datasheet -- every 10ms
 // pulse.setReg(PulsePlug::MEAS_RATE, 0x08);     // every 100ms
  pulse.setReg(PulsePlug::ALS_RATE, 0x08);      // see datasheet ---- one measurement

  pulse.setReg(PulsePlug::PS_RATE, 0x08);       // see datasheet --every time the device wakes up



  // Current setting for LEDs pulsed while taking readings

  // PS_LED21  Setting for LEDs 1 & 2. LED 2 is high nibble

  // each LED has 16 possible (0-F in hex) possible settings

  // read the

  //  pulse.setReg(PulsePlug::PS_LED21, 0x38);      // LED current for 2 (IR1 - high nibble) & LEDs 1 (red - low nibble)
  //PSO2
//   pulse.setReg(PulsePlug::PS_LED21, 0x00);      // this powers off the green leds of the ID107HR


//  pulse.setReg(PulsePlug::PS_LED21, 0x11);
    pulse.setReg(PulsePlug::PS_LED3, 0x02);
  pulse.setReg(PulsePlug::PS_LED21, 0x00);      // this powers off the green leds of the ID107HR

  Serial.print( "PS_LED21 = ");

  Serial.println(pulse.getReg(PulsePlug::PS_LED21), BIN);

  Serial.print("CHLIST = ");

  Serial.println(pulse.readParam(0x01), BIN);



  pulse.writeParam(PulsePlug::PARAM_CH_LIST, 0x77);         // all measurements on



  // increasing PARAM_PS_ADC_GAIN will increase the LED on time and ADC window

  // you will see increase in brightness of visible LED's, ADC output, & noise

  // datasheet warns not to go beyond 4 because chip or LEDs may be damaged

  pulse.writeParam(PulsePlug::PARAM_PS_ADC_GAIN, 0x00);





  // You can select which LEDs are energized for each reading.

  // The settings below turn on only the LED that "normally" would be read

  // ie LED1 is pulsed and read first, then LED2 is pulsed and read etc.

  pulse.writeParam(PulsePlug::PARAM_PSLED12_SELECT, 0x21);  // 21 = LED 2 & LED 1 (red) resp.

  //JJ there is no led 3  pulse.writeParam(PulsePlug::PARAM_PSLED3_SELECT, 0x04);   // 4 = LED 3 only



  // Sensors for reading the three LEDs

  // 0x03: Large IR Photodiode

  // 0x02: Visible Photodiode - cannot be read with LEDs on - just for ambient measurement

  // 0x00: Small IR Photodiode

  pulse.writeParam(PulsePlug::PARAM_PS1_ADCMUX, 0x03);      // PS1 photodiode select

  pulse.writeParam(PulsePlug::PARAM_PS2_ADCMUX, 0x03);      // PS2 photodiode select

  pulse.writeParam(PulsePlug::PARAM_PS3_ADCMUX, 0x03);      // PS3 photodiode select



  pulse.writeParam(PulsePlug::PARAM_PS_ADC_COUNTER, B01110000);    // B01110000 is default

  pulse.setReg(PulsePlug::COMMAND, PulsePlug::PSALS_AUTO_Cmd);     // starts an autonomous read loop

  Serial.println(pulse.getReg(PulsePlug::CHIP_STAT), HEX);

  Serial.print("end init");

}

void readPulseSensorPSO2() {



  static int foundNewFinger, red_signalSize, red_smoothValley;

  static long red_valley, red_Peak, red_smoothRedPeak, red_smoothRedValley,

         red_HFoutput, red_smoothPeak; // for PSO2 calc

  static  int IR_valley = 0, IR_peak = 0, IR_smoothPeak, IR_smoothValley, binOut, lastBinOut;

  static unsigned long lastTotal, lastMillis, IRtotal, valleyTime = millis(), lastValleyTime = millis(), peakTime = millis(), lastPeakTime = millis(), lastBeat, beat;

  static float IR_baseline, red_baseline, IR_HFoutput, IR_HFoutput2, shiftedOutput, LFoutput, hysterisis;

int oldtotal,beginteller,startteller,eindteller,teller,pulsen,grens,gemiddelde,maximum,minimum,eerste;

  unsigned long total = 0, start;

  int i = 0;

  int IR_signalSize;

  red = 0;

  IR1 = 0;

  IR2 = 0;

  total = 0;

  start = millis();







#ifdef POWERLINE_SAMPLING



  while (millis() - start < 16) {  // 60 hz - or use 33 for two cycles

    // 50 hz in Europe use 20, or 40

    Serial.print("sample");

#else

  while (i < SAMPLES_TO_AVERAGE) {

#endif





#ifdef AMBIENT_LIGHT_SAMPLING

    pulse.fetchData();

   // pulse.initPulsePlug();


#else

    pulse.fetchLedData();

#endif



    red += pulse.ps1;

    IR1 += pulse.ps2;

    IR2 += pulse.ps3;

    i++;

  }



  red = red / i;  // get averages

  IR1 = IR1 / i;

  IR2 = IR2 / i;

  //  total =  IR1 + IR2 + red;  // red excluded
  total =   red / 100000;  // red is relevant, there are no IR LEDS

  IRtotal = IR1 + IR2;







#ifdef AMBIENT_LIGHT_SAMPLING
  Serial.print("AMBIENT_VIS");  

//  Serial.print(pulse.resp, HEX);     // resp

  Serial.print("\t");

 Serial.println(pulse.als_vis);       //  ambient visible

 // Serial.println(pulse.ps1); 
//  Serial.print("\t");
 // Serial.print("AMBIENT_IR");  
  
  //Serial.print("\t");
// Serial.println(pulse.als_ir);        //  ambient IR

 // Serial.print("\t");




#endif





#ifdef PRINT_LED_VALS



  Serial.print(red);  //this is the only only with results JJ

  Serial.print("\t");

  Serial.print(IR1);

  Serial.print("\t");

  Serial.print(IR2);

  Serial.print("\t");

  Serial.println((long)total);



#endif



#ifdef SEND_TOTAL_TO_PROCESSING
  if (total > 0)
    Serial.println(total);

#endif



#ifdef GET_PULSE_READING



  // except this one for Processing heartbeat monitor

  // comment out all the bottom print lines


  teller++;
  if (teller > 200) teller = 0;
  //during the first 200 measurement we calculate max and min and average
  if (teller > 0)
  {
    if (total > oldtotal)
    {
      maximum = total;
    }
    else
    {
      minimum = total;
    }
    gemiddelde=gemiddelde+total;
  }
  else
  {
    gemiddelde=gemiddelde/200; //calculate average
    grens = (gemiddelde + maximum)/2;
  }

pulsen++;
if (pulsen > 20) pulsen=0;
if ((total > oldtotal) && (total > grens) && (eerste=0))
{
  startteller=pulsen;
  eerste=1;
}
if ((total > oldtotal) && (total > grens) && (eerste=1))
{
  eerste=0;
  eindteller=pulsen;
}

oldtotal=total;







if (eindteller > startteller)
  BPM = 600 / (eindteller-startteller);
else
  BPM = 0;
  /*           Serial.print(binOut);

             Serial.print("\t BPM ");

             Serial.print(BPM);

             Serial.print("\t IR ");

             Serial.print(IR_signalSize);

             Serial.print("\t PSO2 ");

             Serial.println(((float)red_baseline / (float)(IR_baseline/2)), 3);
  */






#endif

}




/*
  void readPulseSensor() {



  static int foundNewFinger;

  static  int valley = 0, peak = 0, smoothPeak, smoothValley, binOut, lastBinOut;

  static unsigned long lastTotal, lastMillis,  valleyTime = millis(), lastValleyTime = millis(), peakTime = millis(), lastPeakTime = millis(), lastBeat, beat;

  static float baseline, HFoutput, HFoutput2, shiftedOutput, LFoutput, hysterisis;



  unsigned long total = 0, start;

  int i = 0;

  int signalSize;

  red = 0;

  IR1 = 0;

  IR2J = 0;

  total = 0;

  start = millis();





  #ifdef POWERLINE_SAMPLING



  while (millis() - start < 16) {  // 60 hz - or use 33 for two cycles

    // 50 hz in Europe use 20, or 40

    Serial.print("sample");

  #else

  while (i < SAMPLES_TO_AVERAGE) {

  #endif





  #ifdef AMBIENT_LIGHT_SAMPLING

    pulse.fetchData();



  #else

    pulse.fetchLedData();

  #endif



    red += pulse.ps1;

    IR1 += pulse.ps2;

    IR2J += pulse.ps3;

    i++;

  }



  red = red / i;  // get averages

  IR1 = IR1 / i;

  IR2J = IR2J / i;

  total = red + IR1 + IR2J;







  #ifdef AMBIENT_LIGHT_SAMPLING



  Serial.print(pulse.resp, HEX);     // resp

  Serial.print("\t");

  Serial.print(pulse.als_vis);       //  ambient visible

  Serial.print("\t");

  Serial.print(pulse.als_ir);        //  ambient IR

  Serial.print("\t");



  #endif





  #ifdef PRINT_LED_VALS



  Serial.print(red);

  Serial.print("\t");

  Serial.print(IR1);

  Serial.print("\t");

  Serial.print(IR2J);

  Serial.print("\t");

  Serial.println((long)total);



  #endif



  #ifdef SEND_TOTAL_TO_PROCESSING
  if (total > 0)
  Serial.println(total);

  #endif



  #ifdef GET_PULSE_READING



  // except this one for Processing heartbeat monitor

  // comment out all the bottom print lines



  if (lastTotal < 20000L && total > 20000L) foundNewFinger = 1;  // found new finger!



  lastTotal = total;



  // if found a new finger prime filters first 20 times through the loop

  if (++foundNewFinger > 25) foundNewFinger = 25;   // prevent rollover



  if ( foundNewFinger < 20) {

    baseline = total - 200;   // take a guess at the baseline to prime smooth filter

    Serial.println("found new finger");

  }



  else if (total > 20000L) {   // main running function





    // baseline is the moving average of the signal - the middle of the waveform

    // the idea here is to keep track of a high frequency signal, HFoutput and a

    // low frequency signal, LFoutput

    // The HF signal is shifted downward slightly (heartbeats are negative peaks)

    // The high freq signal has some hysterisis added. When the HF signal is less than the

    // shifted LF signal, we have found a heartbeat.

    baseline = smooth(total, 0.99, baseline);   //

    HFoutput = smooth((total - baseline), 0.2, HFoutput);    // recycling output - filter to slow down response

    HFoutput2 = HFoutput + hysterisis;

    LFoutput = smooth((total - baseline), 0.95, LFoutput);

    // heartbeat signal is inverted - we are looking for negative peaks

    shiftedOutput = LFoutput - (signalSize * .05);



    // We need to be able to keep track of peaks and valleys to scale the output for

    // user convenience. Hysterisis is also scaled.

    if (HFoutput  > peak) peak = HFoutput;

    if (peak > 1500) peak = 1500;



    if (millis() - lastPeakTime > 1800) { // reset peak detector slower than lowest human HB

      smoothPeak =  smooth((float)peak, 0.6, (float)smoothPeak);  // smooth peaks

      peak = 0;

      lastPeakTime = millis();

    }



    if (HFoutput  < valley)   valley = HFoutput;

    if (valley < -1500) valley = -1500;



    if (millis() - lastValleyTime > 1800) { // reset valleys detector slower than lowest human HB

      smoothValley =  smooth((float)valley, 0.6, (float)smoothValley);  // smooth valleys

      valley = 0;

      lastValleyTime = millis();

    }



    signalSize = smoothPeak - smoothValley;  // this the size of the smoothed HF heartbeat signal



    // Serial.print(" T  ");

    // Serial.print(signalSize);



    if (HFoutput2 < shiftedOutput) {

      lastBinOut = binOut;

      binOut = 1;

      //   Serial.println("\t1");

      hysterisis = - constrain((signalSize / 15), 35, 120) ;   // you might want to divide by smaller number

      // if you start getting "double bumps"

    }

    else {

      //   Serial.println("\t0");

      lastBinOut = binOut;

      binOut = 0;

      hysterisis = constrain((signalSize / 15), 35, 120);    // ditto above



    }



    if (lastBinOut == 1 && binOut == 0) {

      Serial.println(binOut);

    }



    if (lastBinOut == 0 && binOut == 1) {

      lastBeat = beat;

      beat = millis();

      BPM = 60000 / (beat - lastBeat);

      Serial.print(binOut);

      Serial.print("\t BPM ");

      Serial.print(BPM);

      //   oled.drawString(0, 20, BPM);
      //   oled.display();

      Serial.print("\t signal size ");

      Serial.println(signalSize);

    }



  }

  #endif



  }
*/
/*

  void blePeripheralConnectHandler(BLECentral& central) {
  // central connected event handler
  Serial.print(F("Connected event, central: "));
  Serial.println(central.address());
  }

  void blePeripheralDisconnectHandler(BLECentral& central) {
  // central disconnected event handler
  Serial.print(F("Disconnected event, central: "));
  Serial.println(central.address());
  }
  void characteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  // characteristic value written event handler
  Serial.print(F("Characteristic event, writen: "));
  Serial.println(testCharacteristic.value(), DEC);
  }

  void characteristicSubscribed(BLECentral& central, BLECharacteristic& characteristic) {
  // characteristic subscribed event handler
  Serial.println(F("Characteristic event, subscribed"));
  }

  void characteristicUnsubscribed(BLECentral& central, BLECharacteristic& characteristic) {
  // characteristic unsubscribed event handler
  Serial.println(F("Characteristic event, unsubscribed"));
  }
*/
int smooth(float data, float filterVal, float smoothedVal) {



  if (filterVal > 1) {      // check to make sure param's are within range

    filterVal = .99;

  }

  else if (filterVal <= 0) {

    filterVal = 0;

  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return (int)smoothedVal;

}



