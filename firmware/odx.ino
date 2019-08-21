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

  bleSerial.setLocalName("UART2");
  bleSerial.begin();
  Serial.begin(9600);
  Serial.println(__FILE__);

  pinMode(PIN_BUTTON1, INPUT_PULLUP);


  oled.setScreenSize(OLED_WIDTH, OLED_HEIGHT);
  oled.begin();
  draw_page(page_num++);
  delay(3000); // show splash for 3s
  tPage = millis();
  //JJ acc.init();

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
  
 // digitalWrite(6,HIGH);
  //delay(500);
  
// Serial.print("IR Data1  ");
 //Serial.println(pulse.getReg(PulsePlug::ALS_IR_DATA1));
 //Serial.print("IR Data0  ");
 //Serial.println(pulse.getReg(PulsePlug::ALS_IR_DATA0));
  //Serial.print("VIS Data1  ");
// Serial.println(pulse.getReg(PulsePlug::ALS_VIS_DATA1));
if (!B1_isPressed & !digitalRead(PIN_BUTTON1)) // timer used for button debounce
  {
    page_num = (page_num + 1 < page_count)?page_num+1:0;
  }
  B1_isPressed = !digitalRead(PIN_BUTTON1);
  
  if (millis() - tPage > 20) // 20ms = 50Hz
  {
    tPage = millis();
    draw_page(page_num);
  }
 //Serial.print("Vis ");

 int test_vis =pulse.getReg(PulsePlug::ALS_VIS_DATA0)+256*pulse.getReg(PulsePlug::ALS_VIS_DATA1);
 int test_ir =pulse.getReg(PulsePlug::ALS_IR_DATA0)+256*pulse.getReg(PulsePlug::ALS_IR_DATA1);
 //bleSerial.print("V");
 //bleSerial.print(test_vis);
 //bleSerial.print(",");
 //bleSerial.print("I");
 // bleSerial.print(test_ir);
 float od_vis=-0.396*log(test_vis)+3.1196;
 float od_ir=-0.344*log(test_ir)+3.3413;
 float od=0.5*(od_vis+od_ir);
 bleSerial.print("D");
 bleSerial.print(od);
 // digitalWrite(6,LOW);
  yield();
  delay(1000); // show splash for 3s
  oled.clear(PAGE); // Clear the display's internal memory
  oled.clear(ALL);  // Clear the library's display buffer
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
    oled.setCursor(5, 16);
    oled.print("OD");
    float od_vis=-0.396*log(test_vis)+3.1196;
    float od_ir=-0.344*log(test_ir)+3.3413;
    float od=0.5*(od_vis+od_ir);
    oled.setCursor(20,16);
    //dtostrf(od,5,0,charVal);
    float2chars(od,fltBuf);
    oled.print(fltBuf);
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



