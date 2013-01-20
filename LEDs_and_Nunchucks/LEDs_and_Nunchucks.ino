//#############################################################################
// Controlling an RGB LED strip with a Wii nunchuck using an Arduino
//
// By Martin F. Falatic, http://www.falatic.com/
//
// Using a Nyko Kama wireless Wii nunchuck to control a WS2811-based LED strip
//
// Based on the Arduino Mega 2560 R3 with the stk500boot_v2_mega2560 bootloader.
//     Why I installed the new bootloader:
//         http://code.google.com/p/arduino/issues/detail?id=392
//
// Requires SerialCommand, from https://github.com/kroimon/Arduino-SerialCommand
//
// Based on FastSPI_LED_Effects by Thomas Eldredge, http://www.funkboxing.com/
// Based on NunchuckPrintNew by Tod E. Kurt, http://todbot.com/blog/
// Based on Wii Nunchuck reading code from Windmeadow Labs, http://www.windmeadow.com/node/42
//
//#############################################################################
// How I wired this up:
//
//   Wii nunchuck connections:
//     Connected the breadboard and the Wii Nunchuck via the WiiChuck (lots of places sell them now):
//     http://todbot.com/blog/2008/02/18/wiichuck-wii-nunchuck-adapter-available/
//     3.3v and GND wired normally, Wii_dat -> Mega_pin-20_SDA and Wii_clk -> Mega_pin-21_SCL
//
//   WS2811-based LED strip connections:
//     Provided power to the strip itself from an independent 5V power supply
//     Used a common ground line with the Arduino, WS2811-strip_data-in -> Mega_pin-3
//
//#############################################################################
// Version history:
//   2013-01-20 <MFF> Initial release
//
//#############################################################################

#include <avr/wdt.h>   // watchdog timer functions
#include <FastSPI_LED.h>
#include <Wire.h>
#include <SerialCommand.h>

//#############################################################################
// General globals

#define MAX_STR_BUF 255

boolean verbose_output = false;

SerialCommand sCmd;     // SETUP SerialCommand OBJECT

const char g_help_str[] =
    "-- Commands available:\n" \
    "--   r     Reset device (watchdog method)\n" \
    "--   v     Toggle verbose output\n" \
    "--   m n   Set LED mode to n, where n >= 0\n" \
    "--   h     Display available commands\n";

//#############################################################################
// LED strip / SPI globals

// Hardware SPI uses SCK and MOSI (Master Out Slave In)
// See http://arduino.cc/en/Reference/SPI for more info
// For the WS2811, TM1809 application we're using software SPI
#define CHIPSET   CFastSPI_LED::SPI_WS2811
#define DATA_PIN    3  // For TM1809, WS2811, etc. - doesn't seem to matter if this is a PWM pin
#define DATARATE    7  // IF LEDS FLICKER, PLAY WITH THIS (0-7)
#define NUM_LEDS  320  // How many LEDs are in the strip

unsigned int g_ledmode_curr = 0; // Initial output mode
unsigned int g_ledmode_prev = g_ledmode_curr;

struct CRGB { 
    unsigned char g; 
    unsigned char r; 
    unsigned char b; 
};
struct CRGB *leds;

int ledsX[NUM_LEDS][3]; // ARRAY FOR COPYING WHATS IN THE LED STRIP CURRENTLY (FOR CELL-AUTOMATA, ETC)

//#############################################################################
// Wii nunchuck globals

// I2C uses SCL and SDA
#define WIICHUCKID 0x52
#define USEFAKEPOWER false  // If true, use analog pins 2&3 as fake gnd & pwr
#define ENC_VALUE  0x00    // No encryption
//#define ENC_VALUE  0x17  // If we're using encrypted mode

struct nunchuck_data_s { 
        uint16_t joy_x;
        uint16_t joy_y;
        uint16_t accel_x;
        uint16_t accel_y;
        uint16_t accel_z;
        uint16_t button_z;
        uint16_t button_c;
};
nunchuck_data_s *g_nunchuck_curr = new nunchuck_data_s();
static uint8_t   g_nunchuck_buf[6];
static uint32_t  g_nunchuck_seq = 0;

//#############################################################################
//#############################################################################
// Main routines (setup and loop)
//#############################################################################
void setup()  
{
    Serial.begin(115200);      // SETUP HARDWARE SERIAL (USB)
    delay(100);

    Serial.print("\n");
    Serial.print("===============================================================================\n");
    Serial.print("-- Controlling an RGB LED strip with a Wii nunchuck\n");
    Serial.print("-- By Martin F. Falatic, http://www.Falatic.com/\n");
    Serial.print("===============================================================================\n");
    Serial.print("\n");

    // Set up Wii nunchuck
    Serial.print("-------------------------------------------------------------------------------\n");
    Serial.print("-- Setting up the Wii nunchuck...\n");
    if (USEFAKEPOWER) nunchuck_setpowerpins();
    nunchuck_init();
    Serial.print("\n");

    // Set up LED strip
    Serial.print("-------------------------------------------------------------------------------\n");
    Serial.print("-- Setting up the RGB LED strip...\n");
    led_strip_init();
    Serial.print("\n");

    // Set up serial communications
    Serial.print("-------------------------------------------------------------------------------\n");
    Serial.print("-- Setting up the command handler...\n");

    sCmd.addCommand("r", cmd_reset);
    sCmd.addCommand("h", cmd_help);
    sCmd.addCommand("?", cmd_help);
    sCmd.addCommand("v", cmd_verbose_output);
    sCmd.addCommand("m", cmd_set_mode_strip);
    sCmd.setDefaultHandler(cmd_unrecognized);

    cmd_help();

    // Ready!
    Serial.print("-------------------------------------------------------------------------------\n");
    Serial.print("-- System ready\n\n");
}

//#############################################################################
void loop() {
    if (!nunchuck_get_data())
    {
        Serial.print("!! Error: incomplete data from device (controller not synced?)\n");
        delay(1000);
        return;
    }
    else if (g_nunchuck_buf[0] == 0x00 && g_nunchuck_buf[1] == 0x00 && 
             g_nunchuck_buf[2] == 0x00 && g_nunchuck_buf[3] == 0x00 && 
             g_nunchuck_buf[4] == 0x00 && g_nunchuck_buf[5] == 0x00)
    {
        Serial.print("!! Error: Data pegged low (controller not synced?)\n");
        delay(1000);
        return;
    }
    else if (g_nunchuck_buf[0] == 0xFF && g_nunchuck_buf[1] == 0xFF && 
             g_nunchuck_buf[2] == 0xFF && g_nunchuck_buf[3] == 0xFF && 
             g_nunchuck_buf[4] == 0xFF && g_nunchuck_buf[5] == 0xFF)
    {
        Serial.print("!! Error: Data pegged high: attempting re-initialization\n");
        nunchuck_init(); // send the initilization handshake
        delay(1000);
        return;
    }
    else
    {
        if (verbose_output) {
            nunchuck_print_data();
            delay(1);
        }
    }

    refresh_leds(); // false if not resetting
    sCmd.readSerial(); // Process serial commands
}

//#############################################################################
//#############################################################################
// Serial command handlers
//#############################################################################
void cmd_help() {
    Serial.print("\n");
    Serial.print(g_help_str);
    Serial.print("\n");
}

//#############################################################################
void cmd_unrecognized(const char *command) {
    Serial.print("!! Error: Invalid command:");
    
    Serial.print(" [");
    Serial.print(command);
    Serial.print("]");

    char *arg;
    arg = sCmd.next();

    while (arg != NULL)
    {
        Serial.print(" [");
        Serial.print(arg);
        Serial.print("]");
    }
    Serial.print("\n");
}

//#############################################################################
// Uses a trick found at http://www.nongnu.org/avr-libc/user-manual/FAQ.html#faq_softreset
void cmd_reset() {
    Serial.print("!! Reset requested\n\n");
    mode_one_color_all(0,0,0);  // Blank the LED strip
    delay(100);
    cli();                 // Clear interrupts
    wdt_enable(WDTO_15MS); // Set the watchdog to 15ms
    while(1);              // Enter an infinite loop to trigger the watchdog
}

//#############################################################################
void cmd_verbose_output() {
    verbose_output = !verbose_output;
    if (verbose_output)
    {
        Serial.print("-- Verbose output ON\n");
    }
    else
    {
        Serial.print("-- Verbose output OFF\n");
    }
}

//#############################################################################
void cmd_set_mode_strip() {
    char *arg;
    arg = sCmd.next();

    if (arg == NULL)
    {
        Serial.print("!! Error: No mode specified, assuming mode 0\n");
        g_ledmode_curr = 0;
    }
    else {
        g_ledmode_curr = atoi(arg);
    }

    if (g_ledmode_curr == g_ledmode_prev)
    {
        Serial.print("-- Restarting this LED mode: ");
        Serial.print(g_ledmode_curr);
        Serial.print("\n");
    }
    else
    {
        Serial.print("-- New LED mode selected: ");
        Serial.print(g_ledmode_curr);
        Serial.print("\n");
        g_ledmode_prev = g_ledmode_curr;
    }
    mode_one_color_all(0,0,0);  // Blank the LED strip
}

//#############################################################################
//#############################################################################
// LED strip code
//#############################################################################
void led_strip_init() {

    FastSPI_LED.setLeds(NUM_LEDS);
    FastSPI_LED.setChipset(CHIPSET);
    FastSPI_LED.setPin(DATA_PIN);
    FastSPI_LED.setDataRate(DATARATE);
    FastSPI_LED.init();
    FastSPI_LED.start();
    leds = (struct CRGB*)FastSPI_LED.getRGBData(); 

    Serial.print("-- Blanking the LED strip\n");
    mode_one_color_all(0,0,0);  // Blank the LED strip
    FastSPI_LED.show();
}

//#############################################################################
void refresh_leds() {
    switch (g_ledmode_curr) {
        case 0: // Turn all LEDs off
            mode_one_color_all(0,0,0);
            break;
        case 1: // Set LEDs based on accelerometer
            mode_one_color_all(
                g_nunchuck_curr->accel_x >> 2,
                g_nunchuck_curr->accel_y >> 2,
                g_nunchuck_curr->accel_z >> 2);
            break;
        case 2: // Set LEDs based on accelerometer and joysticks
            uint8_t newcolor[3];
            HSVtoRGB(
                g_nunchuck_curr->accel_x >> 2,
                g_nunchuck_curr->joy_x,
                g_nunchuck_curr->joy_y,
                newcolor);
            mode_one_color_all(newcolor[0], newcolor[1], newcolor[2]);
            break;
        case 6: // Cylon-like
            mode_color_bounce_fade(10);
            break;
        default:
            break;
    }
}

//#############################################################################
void set_color_led(int adex, int cred, int cgrn, int cblu) {  
    leds[adex].r = cred;
    leds[adex].g = cgrn;
    leds[adex].b = cblu;  
}

//#############################################################################
void copy_led_array(){
    for(int i = 0; i < NUM_LEDS; i++ ) {
        ledsX[i][0] = leds[i].r;
        ledsX[i][1] = leds[i].g;
        ledsX[i][2] = leds[i].b;
    }  
}

//#############################################################################
//-FIND ADJACENT INDEX CLOCKWISE
int adjacent_cw(int i) { 
    int r;
    if (i < NUM_LEDS - 1) {
        r = i + 1;
    }
    else {
        r = 0;
    }
    return r;
}

//#############################################################################
//-FIND ADJACENT INDEX COUNTER-CLOCKWISE
int adjacent_ccw(int i) {
    int r;
    if (i > 0) {
        r = i - 1;
    }
    else {
        r = NUM_LEDS - 1;
    }
    return r;
}

//#############################################################################
void mode_one_color_all(int cred, int cgrn, int cblu) { //-SET ALL LEDS TO ONE COLOR
    for(int i = 0 ; i < NUM_LEDS; i++ ) {
        set_color_led(i, cred, cgrn, cblu);
    }  
    FastSPI_LED.show(); 
}

//#############################################################################
int idex = 0;        //-LED INDEX (0 to NUM_LEDS-1
int holdoff = 3;
int bouncedirection = 0;  //-SWITCH FOR COLOR BOUNCE (0-1)

//-BOUNCE COLOR (SIMPLE MULTI-LED FADE)
void mode_color_bounce_fade(int idelay) {
    // A way to display random sprints
    // if (idex == int(NUM_LEDS/2)) {
    //    holdoff = random(3,NUM_LEDS/2-5);
    // }

    if (bouncedirection == 0) {
        idex = idex + 1;
        if (idex >= NUM_LEDS-holdoff) {
            bouncedirection = 1;
            idex = idex - 1;
        }
    }

    if (bouncedirection == 1) {
        idex = idex - 1;
        if (idex < holdoff) {
            bouncedirection = 0;
        }
    }

    int iR1 = adjacent_ccw(idex);
    int iR2 = adjacent_ccw(iR1);
    int iR3 = adjacent_ccw(iR2); 
    int iL1 = adjacent_cw(idex);
    int iL2 = adjacent_cw(iL1);
    int iL3 = adjacent_cw(iL2);  

    for(int i = 0; i < NUM_LEDS; i++ ) {
        if (i == idex) {
            set_color_led(i, 255, 0, 0);
        }
        else if (i == iL1) {
            set_color_led(i, 100, 0, 0);
        }
        else if (i == iL2) {
            set_color_led(i, 50, 0, 0);
        }
        else if (i == iL3) {
            set_color_led(i, 10, 0, 0);
        }        
        else if (i == iR1) {
            set_color_led(i, 100, 0, 0);
        }
        else if (i == iR2) {
            set_color_led(i, 50, 0, 0);
        }
        else if (i == iR3) {
            set_color_led(i, 10, 0, 0);
        }    
        else {
            set_color_led(i, 0, 0, 0);
        }
    }

    FastSPI_LED.show();  
    delay(idelay);
}

//#############################################################################
void HSVtoRGB(int hue, int sat, int val, uint8_t colors[3]) {
    // hue: 0-359, sat: 0-255, val (lightness): 0-255
    uint8_t r, g, b, base;

    if (sat == 0) { // Achromatic color (gray).
        colors[0]=val;
        colors[1]=val;
        colors[2]=val;
    } 
    else  {
        base = ((255 - sat) * val)>>8;
        switch(hue/60) {
        case 0:
            r = val;
            g = (((val-base)*hue)/60)+base;
            b = base;
            break;
        case 1:
            r = (((val-base)*(60-(hue%60)))/60)+base;
            g = val;
            b = base;
            break;
        case 2:
            r = base;
            g = val;
            b = (((val-base)*(hue%60))/60)+base;
            break;
        case 3:
            r = base;
            g = (((val-base)*(60-(hue%60)))/60)+base;
            b = val;
            break;
        case 4:
            r = (((val-base)*(hue%60))/60)+base;
            g = base;
            b = val;
            break;
        case 5:
            r = val;
            g = base;
            b = (((val-base)*(60-(hue%60)))/60)+base;
            break;
        }
        colors[0]=r;
        colors[1]=g;
        colors[2]=b;
    }
}

//#############################################################################
//#############################################################################
// Wii nunchuck code
//#############################################################################
// Uses port C (analog in) pins as power & ground for Nunchuck
static void nunchuck_setpowerpins()
{
#define pwrpin PORTC3
#define gndpin PORTC2
    DDRC |= _BV(pwrpin) | _BV(gndpin);
    PORTC &=~ _BV(gndpin);
    PORTC |=  _BV(pwrpin);
    delay(100);  // wait for things to stabilize        
}

//#############################################################################
// initialize the I2C system, join the I2C bus,
// and tell the nunchuck we're talking to it
void nunchuck_init()
{ 
    Serial.print("-- Initializing the controller\n");

    g_nunchuck_seq = 0; // Reset the sequence counter

    Wire.begin(); // join i2c bus as master

    boolean INITIALIZING = true;

    while (INITIALIZING) {
        if (ENC_VALUE == 0) { // Third-party controllers don't allow encryption
            // It seems critical for F0:55 and FB:00 to be transmitted
            // "separately" when it comes to the Nyko Kama
            Wire.beginTransmission(WIICHUCKID);
            Wire.write((uint8_t)0xF0);
            Wire.write((uint8_t)0x55);
            Wire.endTransmission();

            Wire.beginTransmission(WIICHUCKID);
            Wire.write((uint8_t)0xFB);
            Wire.write((uint8_t)0x00);
            Wire.endTransmission();
        }
        else { // Using encryption (only works with Nintendo-made controllers)
            Wire.beginTransmission(WIICHUCKID);
            Wire.write((uint8_t)0xFB);
            Wire.write((uint8_t)0x00);
            Wire.write((uint8_t)0x40);
            Wire.write((uint8_t)0x17);
            Wire.endTransmission();
        }

        Wire.beginTransmission(WIICHUCKID);
        Wire.write((uint8_t)0xFA);
        Wire.endTransmission();

        const int BYTES_REQD = 6;
        Wire.requestFrom(WIICHUCKID, BYTES_REQD, true); // Request n bytes then release bus

        delay(1); // Doesn't seem to make a difference

        char tmp_str_buf[MAX_STR_BUF];
        int length = 0;
        length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length,
        "-- Controller ident = [");

        int cnt = 0;
        while (Wire.available ())
        {
            g_nunchuck_buf[cnt] = nunchuk_decode_byte(Wire.read());
            if (cnt > 0)
            {
                length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length, ", ");
            }
            length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length, "%02X", g_nunchuck_buf[cnt]);
            cnt++;
        }
        length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length, "]\n");

        if (cnt == BYTES_REQD)
        {
            INITIALIZING = false;
            Serial.print(tmp_str_buf);
        }
        else
        {
            Serial.print("!! Error: Init failed. Retrying...\n");
            delay(1000);
        }
    }

    delay(100); // This seemed to be important but things may work fine without it
}

//#############################################################################
// Send a request for data to the nunchuck
void nunchuck_send_request()
{
    Wire.beginTransmission(WIICHUCKID);
    Wire.write((uint8_t)0x00);
    Wire.endTransmission();
}

//#############################################################################
// Receive data back from the nunchuck
int nunchuck_get_data()
{
    int cnt = 0;
    int BYTES_REQD = 6;

    Wire.requestFrom (WIICHUCKID, BYTES_REQD); // Request n bytes then release bus

    while (Wire.available())
    {
        g_nunchuck_buf[cnt] = nunchuk_decode_byte(Wire.read());
        cnt++;
    }

    nunchuck_send_request(); // Request next data payload

    if (cnt != BYTES_REQD)
    {
        return 0; // Failure
    }

    g_nunchuck_seq++;

    // Also unpack g_nunchuck_buf[5] byte as ZZYYXXcz (accel LSBs and then the buttons)
    g_nunchuck_curr->joy_x    =                                   g_nunchuck_buf[0];
    g_nunchuck_curr->joy_y    =                                   g_nunchuck_buf[1];
    g_nunchuck_curr->button_z =  (g_nunchuck_buf[5] >> 0) & 1;
    g_nunchuck_curr->button_c =  (g_nunchuck_buf[5] >> 1) & 1;
    g_nunchuck_curr->accel_x  = ((g_nunchuck_buf[5] >> 2) & 3) | (g_nunchuck_buf[2] << 2);
    g_nunchuck_curr->accel_y  = ((g_nunchuck_buf[5] >> 4) & 3) | (g_nunchuck_buf[3] << 2);
    g_nunchuck_curr->accel_z  = ((g_nunchuck_buf[5] >> 6) & 3) | (g_nunchuck_buf[4] << 2);

    return 1; // Success
}

//#############################################################################
// Print the input data we have recieved
void nunchuck_print_data()
{ 
    char tmp_str_buf[MAX_STR_BUF];
    int length = 0;

    length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length,
        "SEQ# %10d:  ", g_nunchuck_seq);
    length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length,
        "joy: (h,v)=(%3d, %3d)  ",
        g_nunchuck_curr->joy_x, g_nunchuck_curr->joy_y);
    length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length,
        "accel: (x,y,z)=(%4d, %4d, %4d)  ",
        g_nunchuck_curr->accel_x, g_nunchuck_curr->accel_y, g_nunchuck_curr->accel_z);
    length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length,
        "buttons: (c,z)=(%1d, %1d)  ",
        g_nunchuck_curr->button_c, g_nunchuck_curr->button_z);
    length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length,
        "RAW: [%02X, %02X, %02X, %02X, %02X, %02X]",
        g_nunchuck_buf[0], g_nunchuck_buf[1], g_nunchuck_buf[2],
        g_nunchuck_buf[3], g_nunchuck_buf[4], g_nunchuck_buf[5]);
    length += snprintf(tmp_str_buf+length, MAX_STR_BUF-length, "\n");

    Serial.print(tmp_str_buf);
}

//#############################################################################
// Encode data to format that most wiimote drivers except
// only needed if you use one of the regular wiimote drivers
char nunchuk_decode_byte(char x)
{
    if (ENC_VALUE != 0) {
        x = (x ^ ENC_VALUE) + ENC_VALUE;
    }
    return x;
}

//#############################################################################
//#############################################################################



