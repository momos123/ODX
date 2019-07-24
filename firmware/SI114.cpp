// SI114.h
// Code for the Modern Device Pulse Sensor
// Based on the SI1143 chip
// Also includes subset of JeeLabs Ports library - thanks JCW!
// paul@moderndevice.com 6-27-2012
// 2009-02-13 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php


#include "SI114.h"
//#include <avr/sleep.h>
//#include <util/atomic.h>

// flag bits sent to the receiver
#define MODE_CHANGE 0x80    // a pin mode was changed
#define DIG_CHANGE  0x40    // a digital output was changed
#define PWM_CHANGE  0x30    // an analog (pwm) value was changed on port 2..3
#define ANA_MASK    0x0F    // an analog read was requested on port 1..4


byte PulsePlug::readParam (byte addr) {
    // read from parameter ram
    send();    
    write(PulsePlug::COMMAND);
    write(0x80 | addr); // PARAM_QUERY
    stop();
    delay(10);
    return getReg(PulsePlug::PARAM_RD);
}

byte PulsePlug::getReg (byte reg) {
    // get a register
    send();    
    write(reg);
    receive();
    byte result = read(1);
    stop();
    delay(10);
    return result;
}

void PulsePlug::setReg (byte reg, byte val) {
    // set a register
    send();    
    write(reg);
    write(val);
    stop();
    delay(10);
}

void PulsePlug::initPulsePlug(){
    PulsePlug::setReg(PulsePlug::HW_KEY, 0x17);
    // pulsePlug.setReg(PulsePlug::COMMAND, PulsePlug::RESET_Cmd);
    
    Serial.print("PART: ");
    Serial.print(PulsePlug::getReg(PulsePlug::PART_ID));
    Serial.print(" REV: ");
    Serial.print(PulsePlug::getReg(PulsePlug::REV_ID));
    Serial.print(" SEQ: ");
    Serial.println(PulsePlug::getReg(PulsePlug::SEQ_ID));
    
    PulsePlug::setReg(PulsePlug::INT_CFG, 0x03);       // turn on interrupts
    PulsePlug::setReg(PulsePlug::IRQ_ENABLE, 0x10);    // turn on interrupt on PS3
    PulsePlug::setReg(PulsePlug::IRQ_MODE2, 0x01);     // interrupt on ps3 measurement
    PulsePlug::setReg(PulsePlug::MEAS_RATE, 0x84);     // see datasheet
    PulsePlug::setReg(PulsePlug::ALS_RATE, 0x08);      // see datasheet
    PulsePlug::setReg(PulsePlug::PS_RATE, 0x08);       // see datasheet
    PulsePlug::setReg(PulsePlug::PS_LED21, 0x66 );      // LED current for LEDs 1 (red) & 2 (IR1)
    PulsePlug::setReg(PulsePlug::PS_LED3, 0x06);        // LED current for LED 3 (IR2)
    
    Serial.print( "PS_LED21 = ");
    Serial.println(PulsePlug::getReg(PulsePlug::PS_LED21), BIN);
    Serial.print("CHLIST = ");
    Serial.println(PulsePlug::readParam(0x01), BIN);
    
    PulsePlug::writeParam(PulsePlug::PARAM_CH_LIST, 0x77);         // all measurements on
    
    // increasing PARAM_PS_ADC_GAIN will increase the LED on time and ADC window
    // you will see increase in brightness of visible LED's, ADC output, & noise
    // datasheet warns not to go beyond 4 because chip or LEDs may be damaged
    PulsePlug::writeParam(PulsePlug::PARAM_PS_ADC_GAIN, 0x00);
    
    PulsePlug::writeParam(PulsePlug::PARAM_PSLED12_SELECT, 0x21);  // select LEDs on for readings see datasheet
    PulsePlug::writeParam(PulsePlug::PARAM_PSLED3_SELECT, 0x04);   //  3 only
    PulsePlug::writeParam(PulsePlug::PARAM_PS1_ADCMUX, 0x03);      // PS1 photodiode select
    PulsePlug::writeParam(PulsePlug::PARAM_PS2_ADCMUX, 0x03);      // PS2 photodiode select
    PulsePlug::writeParam(PulsePlug::PARAM_PS3_ADCMUX, 0x03);      // PS3 photodiode select
    
    PulsePlug::writeParam(PulsePlug::PARAM_PS_ADC_COUNTER, B01110000);    // B01110000 is default
    PulsePlug::setReg(PulsePlug::COMMAND, PulsePlug::PSALS_AUTO_Cmd);     // starts an autonomous read loop
    
}

void PulsePlug::setLEDcurrents(byte LED1, byte LED2, byte LED3){
/* VLEDn = 1 V, PS_LEDn = 0001	5.6
VLEDn = 1 V, PS_LEDn = 0010	11.2
VLEDn = 1 V, PS_LEDn = 0011	22.4
VLEDn = 1 V, PS_LEDn = 0100	45
VLEDn = 1 V, PS_LEDn = 0101	67
VLEDn = 1 V, PS_LEDn = 0110	90
VLEDn = 1 V, PS_LEDn = 0111	112
VLEDn = 1 V, PS_LEDn = 1000	135
VLEDn = 1 V, PS_LEDn = 1001	157
VLEDn = 1 V, PS_LEDn = 1010	180
VLEDn = 1 V, PS_LEDn = 1011	202
VLEDn = 1 V, PS_LEDn = 1100	224
VLEDn = 1 V, PS_LEDn = 1101	269
VLEDn = 1 V, PS_LEDn = 1110	314
VLEDn = 1 V, PS_LEDn = 1111	359   */

LED1 = constrain(LED1, 0, 15);
LED2 = constrain(LED2, 0, 15);
LED3 = constrain(LED3, 0, 15);

PulsePlug::setReg(PulsePlug::PS_LED21, (LED2 << 4) | LED1 );
PulsePlug::setReg(PulsePlug::PS_LED3, LED3);       

}

void PulsePlug::setLEDdrive(byte LED1pulse, byte LED2pulse, byte LED3pulse){
 // this sets which LEDs are active on which pulses 
 // any or none of the LEDs may be active on each PulsePlug
 //000: NO LED DRIVE
 //xx1: LED1 Drive Enabled
 //x1x: LED2 Drive Enabled (Si1142 and Si1143 only. Clear for Si1141)
 //1xx: LED3 Drive Enabled (Si1143 only. Clear for Si1141 and Si1142)
 // example setLEDdrive(1, 2, 5); sets LED1 on pulse 1, LED2 on pulse 2, LED3, LED1 on pulse 3
 
PulsePlug::writeParam(PulsePlug::PARAM_PSLED12_SELECT, (LED1pulse << 4) | LED2pulse );  // select LEDs on for readings see datasheet
PulsePlug::writeParam(PulsePlug::PARAM_PSLED3_SELECT, LED3pulse);   

}

void PulsePlug::fetchData () {
    // read out all result registers as lsb-msb pairs of bytes
    send();    
    write(PulsePlug::RESPONSE);
    receive();
    byte* p = (byte*) &resp;
    for (byte i = 0; i < 16; ++i)
        p[i] = read(0);
    read(1); // just to end cleanly
    stop();
}


void PulsePlug::fetchLedData() {

    // read only the LED registers as lsb-msb pairs of bytes
    send();    
    write(PulsePlug::PS1_DATA0);
    receive();
    byte* q = (byte*) &ps1;
    for (byte i = 0; i < 6; ++i)
        q[i] = read(0);
    read(1); // just to end cleanly
    stop();
}



void PulsePlug::fetchALSData()
{
  // static uint16_t als_data[2];
   //static uint16_t tmp;
   // read out all result registers as lsb-msb pairs of bytes
   send();
   write(PulsePlug::ALS_VIS_DATA0);
   receive();
  // requestData(4);
 byte* qc = (byte*) & als_vis;
   for (byte i = 0; i <= 16; ++i)
   {
      qc[i] = read(0);

   }

  // return als_data;
}



void PulsePlug::writeParam (byte addr, byte val) {
    // write to parameter ram
    send();    
    write(PulsePlug::PARAM_WR);
    write(val);
    // auto-increments into PulsePlug::COMMAND
    write(0xA0 | addr); // PARAM_SET
    stop();
    delay(10);
}


uint16_t Port::shiftRead(uint8_t bitOrder, uint8_t count) const {
    uint16_t value = 0, mask = bit(LSBFIRST ? 0 : count - 1);
    for (uint8_t i = 0; i < count; ++i) {
        digiWrite2(1);
        delayMicroseconds(5);
        if (digiRead())
            value |= mask;
        if (bitOrder == LSBFIRST)
            mask <<= 1;
        else
            mask >>= 1;
        digiWrite2(0);
        delayMicroseconds(5);
    }
    return value;
}

void Port::shiftWrite(uint8_t bitOrder, uint16_t value, uint8_t count) const {
    uint16_t mask = bit(LSBFIRST ? 0 : count - 1);
    for (uint8_t i = 0; i < count; ++i) {
        digiWrite((value & mask) != 0);
        if (bitOrder == LSBFIRST)
            mask <<= 1;
        else
            mask >>= 1;
        digiWrite2(1);
        digiWrite2(0);
    }
}


PortI2C::PortI2C (uint8_t num, uint8_t rate)
    : Port (num), uswait (rate)
{
    sdaOut(1);
    mode2(OUTPUT);
    sclHi();
}

uint8_t PortI2C::start(uint8_t addr) const {
    sclLo();
    sclHi();
    sdaOut(0);
    return write(addr);
}

void PortI2C::stop() const {
    sdaOut(0);
    sclHi();
    sdaOut(1);
}

uint8_t PortI2C::write(uint8_t data) const {
    sclLo();
    for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
        sdaOut(data & mask);
        sclHi();
        sclLo();
    }
    sdaOut(1);
    sclHi();
    uint8_t ack = ! sdaIn();
    sclLo();
    return ack;
}

uint8_t PortI2C::read(uint8_t last) const {
    uint8_t data = 0;
    for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
        sclHi();
        if (sdaIn())
            data |= mask;
        sclLo();
    }
    sdaOut(last);
    sclHi();
    sclLo();
    if (last)
        stop();
    sdaOut(1);
    return data;
}

bool DeviceI2C::isPresent () const {
    byte ok = send();
    stop();
    return ok;
}

byte MilliTimer::poll(word ms) {
    byte ready = 0;
    if (armed) {
        word remain = next - millis();
        // since remain is unsigned, it will overflow to large values when
        // the timeout is reached, so this test works as long as poll() is
        // called no later than 5535 millisecs after the timer has expired
        if (remain <= 60000)
            return 0;
        // return a value between 1 and 255, being msecs+1 past expiration
        // note: the actual return value is only reliable if poll() is
        // called no later than 255 millisecs after the timer has expired
        ready = -remain;
    }
    set(ms);
    return ready;
}

word MilliTimer::remaining() const {
    word remain = armed ? next - millis() : 0;
    return remain <= 60000 ? remain : 0;
}

void MilliTimer::set(word ms) {
    armed = ms != 0;
    if (armed)
        next = millis() + ms - 1;
}

Scheduler::Scheduler (byte size) : maxTasks (size), remaining (~0) {
    byte bytes = size * sizeof *tasks;
    tasks = (word*) malloc(bytes);
    memset(tasks, 0xFF, bytes);
}

Scheduler::Scheduler (word* buf, byte size) : tasks (buf), maxTasks (size), remaining(~0) {
    byte bytes = size * sizeof *tasks;
    memset(tasks, 0xFF, bytes);
}

char Scheduler::poll() {
    // all times in the tasks array are relative to the "remaining" value
    // i.e. only remaining counts down while waiting for the next timeout
    if (remaining == 0) {
        word lowest = ~0;
        for (byte i = 0; i < maxTasks; ++i) {
            if (tasks[i] == 0) {
                tasks[i] = ~0;
                return i;
            }
            if (tasks[i] < lowest)
                lowest = tasks[i];
        }
        if (lowest != ~0) {
            for (byte i = 0; i < maxTasks; ++i) {
                if(tasks[i] != ~0) {
                    tasks[i] -= lowest;
                }
            }
        }
        remaining = lowest;
    } else if (remaining == ~0) //remaining == ~0 means nothing running
        return -2;
    else if (ms100.poll(100))
        --remaining;
    return -1;
}


void Scheduler::timer(byte task, word tenths) {
    // if new timer will go off sooner than the rest, then adjust all entries
    if (tenths < remaining) {
        word diff = remaining - tenths;
        for (byte i = 0; i < maxTasks; ++i)
            if (tasks[i] != ~0)
                tasks[i] += diff;
        remaining = tenths;
    }
    tasks[task] = tenths - remaining;
}

void Scheduler::cancel(byte task) {
    tasks[task] = ~0;
}


