#include "PMW3360_SROM.h"

extern const unsigned short firmware_length;
extern const uint16_t firmware_data[];

namespace adns {
    // Registers
#define REG_Product_ID                           0x00
#define REG_Revision_ID                          0x01
#define REG_Motion                               0x02
#define REG_Delta_X_L                            0x03
#define REG_Delta_X_H                            0x04
#define REG_Delta_Y_L                            0x05
#define REG_Delta_Y_H                            0x06
#define REG_SQUAL                                0x07
#define REG_Pixel_Sum                            0x08
#define REG_Maximum_Pixel                        0x09
#define REG_Minimum_Pixel                        0x0a
#define REG_Shutter_Lower                        0x0b
#define REG_Shutter_Upper                        0x0c
#define REG_Frame_Period_Lower                   0x0d
#define REG_Frame_Period_Upper                   0x0e
#define REG_Configuration_I                      0x0f
#define REG_Configuration_II                     0x10
#define REG_Frame_Capture                        0x12
#define REG_SROM_Enable                          0x13
#define REG_Run_Downshift                        0x14
#define REG_Rest1_Rate                           0x15
#define REG_Rest1_Downshift                      0x16
#define REG_Rest2_Rate                           0x17
#define REG_Rest2_Downshift                      0x18
#define REG_Rest3_Rate                           0x19
#define REG_Frame_Period_Max_Bound_Lower         0x1a
#define REG_Frame_Period_Max_Bound_Upper         0x1b
#define REG_Frame_Period_Min_Bound_Lower         0x1c
#define REG_Frame_Period_Min_Bound_Upper         0x1d
#define REG_Shutter_Max_Bound_Lower              0x1e
#define REG_Shutter_Max_Bound_Upper              0x1f
#define REG_LASER_CTRL0                          0x20
#define REG_Observation                          0x24
#define REG_Data_Out_Lower                       0x25
#define REG_Data_Out_Upper                       0x26
#define REG_SROM_ID                              0x2a
#define REG_Lift_Detection_Thr                   0x2e
#define REG_Configuration_V                      0x2f
#define REG_Configuration_IV                     0x39
#define REG_Power_Up_Reset                       0x3a
#define REG_Shutdown                             0x3b
#define REG_Inverse_Product_ID                   0x3f
#define REG_Motion_Burst                         0x50
#define REG_SROM_Load_Burst                      0x62
#define REG_Pixel_Burst                          0x64

#define ENABLE_MOTION_BURST                      1

#define RESET3360                                9
// | ADNS-9800 | Arduino Uno Pins
// | SS ncs    | 10
// | MOSI      | 11
// | SCLK      | 13
// | MISO      | 12
// | MOT       | 2 (attachInterrupt() at int.0 = pin 2; int.1 = pin 3)
// | VI        |+5V (First You must Activate 5V Mode)
// | AG        | Gnd
// | DG        | Gnd
// | 3360rst   | 9
    class controller {
public:
        enum MotionBurst {
            Motion = 0, Observation, 
            Delta_X_L, Delta_X_H, Delta_Y_L, Delta_Y_H, SQUAL, 
            Pixel_Sum, Maximum_Pixel, Minimum_Pixel, 
            Shutter_Upper, Shutter_Lower, 
            Frame_Period_Upper, Frame_Period_Lower, EndData
        };

        controller() {
        }
        ~controller() {
        }

        void setup();
        void loop();

        virtual void get_xy(int16_t x, int16_t y) = 0;
        virtual void get_xy_dist(int16_t x_sum, int16_t y_sum) = 0;
        virtual void get_squal(uint16_t s) = 0;
        virtual void get_fault() = 0;
        virtual void clear() = 0;
        
        void reset_xy_dist();
        void printMotionData();

private:
        void upload_firmware();
        void perform_startup();
        void perform_startup2();
        void display_registers();

        static void com_begin();
        static void com_end();
        static byte read_reg(byte reg_addr);
        static void write_reg(byte reg_addr, byte data);
        static void read_motion_burst_data();
        static void copy_data();
        static void update_motion_data();
        static void update_motion_burst_data();
        static int8_t convert_twos_compliment(byte b);
        static int16_t convert_twos_compliment(byte l, byte h);
        static int16_t convert_twos_compliment(uint16_t u);
        static uint16_t join_byte(byte l, byte h);
    };

    const int _ncs = 10; // The SS pin
    byte _boot_complete = 0;

    volatile byte _data[controller::EndData];
    volatile uint16_t _ux = 0;
    volatile uint16_t _uy = 0;
    volatile uint16_t _ux_dist = 0;
    volatile uint16_t _uy_dist = 0;

    volatile byte _mot = 0;
    volatile byte _fault = 0;
    volatile byte _squal = 0;
    volatile byte _moved = 0;

    void controller::reset_xy_dist() {
        _ux = _uy = _ux_dist = _uy_dist = 0;
    }

    void controller::com_begin() {
        digitalWrite(_ncs, LOW);
    }

    void controller::com_end() {
        digitalWrite(_ncs, HIGH);
    }

    byte controller::read_reg(byte reg_addr) {
        com_begin();

        // send adress of the register, with MSBit = 0 to indicate it's a read
        SPI.transfer(reg_addr & 0x7f);
        delayMicroseconds(100); // tSRAD
        // read data
        byte data = SPI.transfer(0);

        delayMicroseconds(1); // tSCLK-_ncs for read operation is 120ns
        com_end();
        delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-_ncs

        return data;
    }

    void controller::write_reg(byte reg_addr, byte data) {
        com_begin();

        // send adress of the register, with MSBit = 1 to indicate it's a write
        SPI.transfer(reg_addr | 0x80);
        // sent data
        SPI.transfer(data);

        delayMicroseconds(20); // tSCLK-_ncs for write operation
        com_end();
        delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-_ncs. Could be shortened, but is looks like a safe lower bound 
    }

    void controller::read_motion_burst_data() {
        com_begin();

        // send adress of the register, with MSBit = 1 to indicate it's a write
        SPI.transfer(REG_Motion_Burst & 0x7f);
        //delayMicroseconds(100); // tSRAD
        // read data
        for (int i = 0; i < Pixel_Sum; ++i) {
            _data[i] = SPI.transfer(0);
        }

        com_end();
        delayMicroseconds(1); //  tBEXIT
    }

    uint16_t controller::join_byte(byte l, byte h){
        uint16_t b = l;
        b |= (h << 8);
        return b;
    }
    
    int8_t controller::convert_twos_compliment(byte b){
        int8_t val = b;
        //Convert from 2's complement
        if(b & 0x80) {
            val = -1 * ((b ^ 0xff) + 1);
        }
        return val;
    }

    int16_t controller::convert_twos_compliment(uint16_t b){
        int16_t val = b;
        //Convert from 2's complement
        if (b & 0x8000) {
            val = -1 * ((b ^ 0xffff) + 1);
        }
        return val;
    }

    int16_t controller::convert_twos_compliment(byte l, byte h){
        uint16_t b = join_byte(l, h);
        return convert_twos_compliment(b);
    }

    void controller::copy_data()
    {
        _squal = _data[SQUAL];
        _ux = join_byte(_data[Delta_X_L], _data[Delta_X_H]);
        _uy = join_byte(_data[Delta_Y_L], _data[Delta_Y_H]);
        _ux_dist += _ux;
        _uy_dist += _uy;
    }

    void controller::update_motion_data() {
        if(_boot_complete != 9) return;
        com_begin();
        _data[Motion] = read_reg(REG_Motion);
        _data[SQUAL] = read_reg(REG_SQUAL);
        _mot = _data[Motion] & 0x80;
        _fault = _data[Motion] & 0x40;
        if (!_fault && _mot) {
            _data[Delta_X_L] = read_reg(REG_Delta_X_L);
            _data[Delta_X_H] = read_reg(REG_Delta_X_H);
            _data[Delta_Y_L] = read_reg(REG_Delta_Y_L);
            _data[Delta_Y_H] = read_reg(REG_Delta_Y_H);
            copy_data();
            _moved = 1;
        }
        com_end();
    }

    void controller::update_motion_burst_data() {
        if(_boot_complete != 9) return;
        com_begin();
        read_motion_burst_data();
        _mot = _data[Motion] & 0x80;
        _fault = _data[Motion] & 0x40;
        if (!_fault && _mot) {
            copy_data();
            _moved = 1;
        }
        com_end();
    }

    void controller::upload_firmware() {
        // send the firmware to the chip, cf p.18 of the datasheet
        Serial.println("Uploading firmware...");

        write_reg(REG_Configuration_II, 0x00);  //based on https://pastebin.com/TQmL6wA3

        // set the configuration_IV register in 3k firmware mode
        //write_reg(REG_Configuration_IV, 0x00); // bit 1 = 1 for 3k mode, other bits are reserved //was 03 mac

        write_reg(0x22, 0x00);

        // write 0x1d in SROM_enable reg for initializing
        write_reg(REG_SROM_Enable, 0x1d); 

        // wait for more than one frame period
        delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low

        // write 0x18 to SROM_enable to start SROM download
        write_reg(REG_SROM_Enable, 0x18); 

        // write the SROM file (=firmware data) 
        com_begin();
        SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
        delayMicroseconds(15);

        // send all bytes of the firmware
        unsigned char c;
        for(unsigned int i = 0; i < firmware_length; i++){
            c = (unsigned char)pgm_read_byte(firmware_data + i);
            SPI.transfer(c);
            delayMicroseconds(15);
        }

        com_end();
    }

    void controller::perform_startup() {
        com_end(); // ensure that the serial port is reset
        com_begin(); // ensure that the serial port is reset
        com_end(); // ensure that the serial port is reset
        write_reg(REG_Power_Up_Reset, 0x5a); // force reset
        delay(50); // wait for it to reboot
        // read registers 0x02 to 0x06 (and discard the data)
        read_reg(REG_Motion);
        read_reg(REG_Delta_X_L);
        read_reg(REG_Delta_X_H);
        read_reg(REG_Delta_Y_L);
        read_reg(REG_Delta_Y_H);
        // upload the firmware
        upload_firmware();
        delay(10);
        //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
        // reading the actual value of the register is important because the real
        // default value is different from what is said in the datasheet, and if you
        // change the reserved bytes (like by writing 0x00...) it would not work.
        //byte laser_ctrl0 = read_reg(REG_LASER_CTRL0);
        //write_reg(REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );

        delay(1);

        // set the configuration_I register
        // 0x01 = 50, minimum
        // 0x44 = 3400, default
        // 0x8e = 7100
        // 0xA4 = 8200, maximum
        //write_reg(REG_Configuration_I, 0xA4);

        Serial.println("Optical Chip Initialized");
    }

void controller::perform_startup2()  //modified by mac 3/24/2017
 {
  uint8_t tmp;
  uint16_t i;
  //const uint8_t *psrom = srom;

  digitalWrite(_ncs,HIGH);;
  delay(3);

  //shutdown first
  write_reg(0x3b, 0xb6);
  delay(300);

  //drop and raise ncs to reset spi port
  digitalWrite(_ncs,LOW);;
  delayMicroseconds(40);
  digitalWrite(_ncs,HIGH);;
  delayMicroseconds(40);
  
  //power up reset
  write_reg(0x3a, 0x5a);
  delay(50);
  
  //flip on and off the clock tuning. not sure purpose...
  write_reg(0x3d, 0x95);
  delay(1);
  write_reg(0x3d, 0x15);
  
  //read from 0x02 to 0x06
  read_reg(0x02);
  read_reg(0x03);
  read_reg(0x04);
  read_reg(0x05);
  read_reg(0x06);

  //dunno??
  write_reg(0x10, 0x00); //turn sleep mode off
  write_reg(0x22, 0x00);
  
  //srom download
  //write_reg(0x13, 0x1d);
  //delay(10);
  //write_reg(0x13, 0x18);
  
  //digitalWrite(_ncs,LOW);;
  upload_firmware();
  
  /* upload procedure for avago
  SPI_SEND(0x62 | 0x80);
  for (i = 0; i < SROM_LENGTH; i++) {
    delayMicroseconds(16);
    SPI_SEND(pgm_read_byte(psrom++));
  }
  */ 
  //end avago upload procedure
  
  delayMicroseconds(18);
  digitalWrite(_ncs,HIGH);;
  delayMicroseconds(200);

  //check srom id (all i know is that it's 0x07 for g502) 0x03 for 3360
  read_reg(0x2a);
  //if (tmp != 0x07) ...
  
  //configuration?? dunno??
  write_reg(0x10, 0x00); //0x20 (g502 default) enables sleep mode after ~10s of inactivity
  write_reg(0x14, 0xff); //run downshift
  write_reg(0x17, 0xff);
  write_reg(0x18, 0x00);
  write_reg(0x19, 0x00);
  write_reg(0x1b, 0x00);
  write_reg(0x1c, 0x00);
  
  write_reg(0x24, 0x00);
  delay(1);
  write_reg(0x13, 0x15);
  delay(10);

  //read_reg(0x2a);

  //surface tuning settings (default: 0x0a, 0x10)
  //probably not necessary to read them first.
  read_reg(0x2c);
  read_reg(0x2b);
  delayMicroseconds(18);
  write_reg(0x2c, 0x0a);
  write_reg(0x2b, 0x10);

  //"manual" clock tuning
  delayMicroseconds(500); //arbitrary padding
  write_reg(0x3d, 0x9c);
  delay(1);
  write_reg(0x3d, 0x1c);
  delayMicroseconds(500); //arbitrary padding
  write_reg(0x4f, 0x8a);
  delay(1);
  write_reg(0x4f, 0x0a);
  delayMicroseconds(500); //arbitrary padding

  //set dpi and angle snapping
  //motion burst
  write_reg(0x50, 0x00);
  delayMicroseconds(9);
  digitalWrite(_ncs,LOW);;
  SPI.transfer(0x50);
  delayMicroseconds(42);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  delayMicroseconds(4);
  digitalWrite(_ncs,HIGH);;
  delayMicroseconds(500); //arbitrary padding
  //dpi
  write_reg(0x0f, 0x07);
  delayMicroseconds(500); //arbitrary padding
  
  //motion burst
  write_reg(0x50, 0x00);
  delayMicroseconds(9);
  digitalWrite(_ncs,LOW);;
  SPI.transfer(0x50);
  delayMicroseconds(42);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  delayMicroseconds(4);
  digitalWrite(_ncs,HIGH);;
  delayMicroseconds(500); //arbitrary padding
  //angle snapping
  write_reg(0x42, 0x00);
  delayMicroseconds(500); //arbitrary padding
} //end startup from other 3360 program

    void controller::display_registers() {
        int oreg[7] = { 
            REG_Product_ID, REG_Inverse_Product_ID, REG_SROM_ID, REG_Motion, REG_LASER_CTRL0                                                                                                                                                                                                                                                                                                                                                                                             };
        const char* oregname[] = {
            "Product_ID","Inverse_Product_ID","SROM_Version","Motion", "LASER_CTRL0"                                                                                                                                                                                                                                                                                                                                                                                            };
        byte regres;

        com_begin();

        int rctr = 0;
        for(rctr = 0; rctr < 5; rctr++){
            SPI.transfer(oreg[rctr]);
            delay(1);
            Serial.println("---");
            Serial.print(oregname[rctr]);
            Serial.print(" (0x");
            Serial.print(oreg[rctr],HEX);
            Serial.println(")");
            regres = SPI.transfer(0);
            Serial.print(regres,BIN);  
            Serial.print(" (0x");
            Serial.print(regres,HEX);  
            Serial.println(")");
            delay(1);
        }
        //dump all registers.
        rctr = 0;
        for(rctr = 0; rctr < 128; rctr++){
            SPI.transfer(oreg[rctr]);
            delay(2);
            Serial.println("---");
            Serial.print(rctr);
            Serial.print(" (0x");
            Serial.print(rctr,HEX);
            Serial.println(")");
            regres = SPI.transfer(0);
            Serial.print(regres,BIN);  
            Serial.print(" (0x");
            Serial.print(regres,HEX);  
            Serial.println(")");
            delay(2);
        }
        com_end();
    }

    void controller::printMotionData() {
        Serial.print(_data[Delta_X_L], BIN);
        Serial.print("(");
        Serial.print(convert_twos_compliment(_data[Delta_X_L]));
        Serial.print(") ");
        Serial.print(_data[Delta_X_H], BIN);
        Serial.print("(");
        Serial.print(convert_twos_compliment(_data[Delta_X_H]));
        Serial.print(") ");
        Serial.print(_data[Delta_Y_L], BIN);
        Serial.print("(");
        Serial.print(convert_twos_compliment(_data[Delta_Y_L]));
        Serial.print(") ");
        Serial.print(_data[Delta_Y_H], BIN);
        Serial.print("(");
        Serial.print(convert_twos_compliment(_data[Delta_Y_H]));
        Serial.print(") ");
        Serial.print(_data[SQUAL], BIN);
        Serial.println("");
    }

    void controller::setup() {
        pinMode (_ncs, OUTPUT);
        pinMode (RESET3360, OUTPUT);
        digitalWrite(RESET3360, HIGH);

#if ENABLE_MOTION_BURST
        attachInterrupt(2, update_motion_burst_data, FALLING); //pin 3
#else
        attachInterrupt(2, update_motion_data, FALLING);  //pin 3 for c
#endif

        SPI.begin();
        SPI.setDataMode(SPI_MODE3);
        SPI.setBitOrder(MSBFIRST);
        SPI.setClockDivider(16);//8
        
        uint8_t id = read_reg(0x00);
        if (id == 0x42)
        Serial.println(F("PMW3360 found"));
        else {
          Serial.print(F("Could not find ADNS-3080: "));
          Serial.println(id, HEX);
        }

        perform_startup2();  //use startup 2 for 3360
        display_registers();

        write_reg(0x24, 0x00);
        delay(50);
        uint8_t tmpdata = read_reg(0x24);
        Serial.print("7th bit srom check 0x");
        Serial.println(tmpdata, HEX);

        write_reg(0x13, 0x15);
        tmpdata = read_reg(0x26);
        Serial.print("BE EF Check 0x25 0x");        
        Serial.println(tmpdata, HEX);
        tmpdata = read_reg(0x25);
        Serial.print("            0x24 0x");
        Serial.println(tmpdata, HEX);
        tmpdata = read_reg(0x2A);
        Serial.print("SROMVER 0x03     0x");
        Serial.println(tmpdata, HEX);
        
        delay(100);
        _boot_complete=9;
        Serial.println("Boot Complete = 9");
    }

    void controller::loop() {
        if (! _moved) return;
        if(_mot) {
            clear();
            get_squal(_squal);
            get_xy(convert_twos_compliment(_ux), convert_twos_compliment(_uy));
            get_xy_dist(convert_twos_compliment(_ux_dist), convert_twos_compliment(_uy_dist));
            delay(3);
            printMotionData();
        }
        if (_fault) {
            get_fault();
        }
        _moved = 0;
    }
};

