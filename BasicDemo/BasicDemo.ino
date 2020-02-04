/*
  ==================================================================
  BasicDemo.ino by Meridian Innovation Limited.
  Last Modified 2020-02-04.

  BasicDemo.ino is a part of the ESP32-MI48-ToF development kit.

  This project demonstrates the basic functionality this development kit:
  - Reading continuous thermal images from the MI48 core board.
  - Reading from the VL53L1X ToF module
  - Displaying the thermal image, distance reading and the distance-compensated
    temperature information on the LCD screen.
  ==================================================================
    
  The board is wired up as follows:
  REQUIRED CONNECTIONS
  ------------------------------------------------------------------
  GUI pushbuttons:
  There are 3 momentary push buttons used to control the GUI
  S1 = left button         - LOW when pressed
  S2 = middle button       - LOW when pressed
  S3 = right button        - LOW when pressed
  NOTE: the middle buton is also used for code download - Press the button to enable serial boot
  ------------------------------------------------------------------
  signal       -   pin      -  ESP32 GPIO
  ------------------------------------------------------------------
  S1           -   S1-1     -  GPIO34              // GPIO34 is input only and has no on-die pullup - hence need for resistor R4
  S2           -   S2-1     -  GPIO0               // Also used to allow boot from serial for code download (hold button to download)
  S3           -   S3-1     -  GPIO35              // GPIO35 is input only and has no on-die pullup - hence need for resistor R5

  ------------------------------------------------------------------
  X1 LCD connector
  The LCD has been configured to be on the HSPI interface of the ESP32
  The LCD connections are declared in the User_Setup.h file in the TFT_eSPI-master library.
  ------------------------------------------------------------------
  LCD Sig      - pin        -   ESP32 GPIO
  ------------------------------------------------------------------
  GND          -   X1-1     -   GND
  VDD          -   X1-2     -   VCC (3v3)
  SCL          -   X1-3     -   IO14      // Connected to SPI CLK
  SDA          -   X1-4     -   IO13      // Connected to SPI MOSI
  RES          -   X1-5     -   IO26      // LCD RESET Connected to spare GPIO
  DC           -   X1-6     -   IO25      // LCD DATA/COMMAND Connected to spare GPIO
  CS           -   X1-7     -   IO15      // Connected to SPI SS
  BL           -   X1-8     -   1027      // Backlight - Connected to DATA - this pin is shared with X4 connecotr. This is a linre on 2P board
                                          // Tracked on 3P board - goes via R11 (wire link) so that connection to BL can be isolated from GPIO27

  ------------------------------------------------------------------
  JP1 MI48TL Board connector:
  ------------------------------------------------------------------
  MI48TL Sig   -   pin      -   ESP32 GPIO
  ------------------------------------------------------------------
  SPI_SS       -   JP1-1    -   IO5
  SPI_CLK      -   JP1-2    -   IO18
  SPI_MOSI     -   JP1-3    -   IO23
  SPI_MISO     -   JP1-4    -   IO19
  DATA_READY   -   JP1-5    -   IO17    // Labelled CUST_INT
  CUST_SCK     -   JP1-6    -   IO22
  CUST_SDA     -   JP1-7    -   IO21
  CUST_GPIO    -   JP1-8    -   GND     // Not pinned out in this version (run out of GPIOS)
  GND          -   JP1-9    -   GND
  +5V          -   JP1-10   -   +5V
  ADDR (PA13)  -   JP1-11   -   IO33    // ADDR - address pin that selects the MI48TL i2c address
  nRESET       -   JP1-12   -   IO16    // labelled XXX on early MI48A0 TIP board circuit diagrams

  ------------------------------------------------------------------
  X3 VL53L1X TOF connector:
  ------------------------------------------------------------------
  X3 sig       -   pin      -  ESP32 GPIO
  ------------------------------------------------------------------
  VCC          -   X3-1     -  VCC (3v3)
  GND          -   X3-2     -  GND
  3v3(out)     -   X3-3     -  N/C

  SCL          -   X3-3     -  GPIO22 (CUST SCK)
  SDA          -   X3-4     -  GPIO21 (CUST SDA)
  GPIO1        -   X3-5     -  GPIO4                // Interupt pin from the VL53L1X
  XSHUT        -   X3-5     -  GPIO2                // Active low signal to shut down VL53L1X - note: GPIO2 is shared with LED.
                                                    // MUST be either left floating or be pulled low for the boot from serial
                                                    // (code download to work). This low pulldown is NOT on the 2P PCB so there is a 10K resistor
                                                    // wired beween GRN and XShUT on the VL53L1X module - fixed on 3P board



  ADDITIONAL CONNECTIONS
  ------------------------------------------------------------------
  J1 SD CARD connector:
  ------------------------------------------------------------------
  J1 sig       -   pin      -  ESP32 GPIO
  ------------------------------------------------------------------
  DATA2        -   J1-1     -  N/C
  CS           -   J1-2     -  IO32 (SD_CS)
  DI           -   J1-3     -  IO23 (SPI_MOSI)
  VDD          -   J1-4     -  VCC (3v3)
  SCLK         -   J1-5     -  IO18 (SPI_CLK)
  VSS          -   J1-6     -  GND
  DO           -   J1-7     -  IO19 (SPI_MISO)
  DATA1        -   J1-8     -  N/C
  CDN          -   J1-9     -  N/C


  ------------------------------------------------------------------
  X2 GY91 IMU connector:
  The GY91 board comsists of an MPU9250 9DOF Inertial Measurement Unit (IMU)
  together with a BMP280 barometer and temperature measurement device
  ------------------------------------------------------------------
  X2 sig       -   pin      -  ESP32 GPIO
  ------------------------------------------------------------------
  VIN          -   X2-1     -  VCC (3v3)
  3v3(out)     -   X2-2     -  N/C
  GND          -   X2-3     -  GND
  SCL          -   X2-4     -  GPIO22 (CUST SCK)
  SDA          -   X2-5     -  GPIO21 (CUST SDA)
  SDO/SAO      -   X2-6     -  N/C
  NCS          -   X2-7     -  N/C
  CSB          -   X2-8     -  N/C
*/
#include <Wire.h>                           // i2c library (standard Arduino library)                     
#include <SPI.h>                            // SPI library (standard Arduin library)
#include <TFT_eSPI.h>                       // SPECIFIC TO ESP32 Graphics and font library for ST7735 driver chip (must be downloaded and installed)
#include <VL53L1X.h>                        // Library for 4 metre TOF distance measuring sensor, distance in mm (must be downloaded and installed)

//#define DEBUG                               // uncomment this if you need serial debug enabled

/* 
  Distance calibration information, only used in objectTemperatureByDistance()  
  
  For instructions to calibrate and calculate the values for the below variables,
  refer to the application note "Distance Correction of Temperature Readout" by Meridian Innovation Limited.
*/
const float A_REF = -1.15f;
const float B     = -0.074f;
const float T_REF = 35.f; 

// =========================================================================
//  Declare the GPIO pins
// =========================================================================
const int DATA_READY_PIN   = 17;          // i/p Pin connected to DATA_READY on MI48TL
const int MI48TL_SS_PIN    = 5;           // o/p Pin Connected to Slave select on the MI48TL
const int nRESET_PIN       = 16;          // o/p pin Connected to nRESET on MI48TL (WAS IO15, changed for LCD)
const int MI48TL_ADDR_PIN  = 33;          // This is the ADDR pin for the MI48TL i2c address
const int CUST_SCK_PIN     = 22;          // need to declare this to enable a pullup on it
const int CUST_SDA_PIN     = 21;          // need to declare this to enable a pullup on it

const uint16_t BACKLIGHT_PIN = 27;        // Pin used for to dissable the LCD backlight (needs to be driven active low to turn off backlight
// NOTE: this is a wire on the 2P board and comes via R11 LINK on 3P board
const uint16_t GPIO1_PIN = 4;             // Pin used for interupt from VL53L1X
const uint16_t XSHUT_PIN = 2;             // This pin controls the shutdown of the VL53L1X TOF sensor - Low = shutdown
// NOTE GPIO2 is also the LED pin it MUST be either left floating or be pulled low for the boot from serial
// (code download to work). This low pulldown is NOT on the 2P PCB so there is a 10K resistor
// wired beween GRN and XShUT on the VL53L1X module the resistor os on the PCB (R10)

const uint16_t S1_LEFT_PIN = 34;          // Left GUI button - GPIO34 is input only so no i/o pullup available - pullup R4 is on PCB
const uint16_t S2_MIDDLE_PIN = 0;         // Middle GUI button - GPIOo is also the serial boot pin so press to download and flash code
const uint16_t S3_RIGHT_PIN = 35;         // Right GUI button - GPIO35 is input only so no i/o pullup available - pullup R5 is on PCB

// =========================================================================
//  Declare variables for use with the GPIOS
// =========================================================================
bool S1Left = true;                       // Boolean to hold value of the left GUI button default it to true until first read
bool S1LeftOld = true;                    // Boolean to hold history value of the left GUI button default it to true until first read
bool S2Middle = false;                    // Boolean to hold value of the middle GUI button default it to false until first read
bool S3Right = false;                     // Boolean to hold value of the right GUI button default it to false until first read

VL53L1X tofSensor;                        // Invoke library, Declare VL53L1X TOF sensor as "tofSensor"

// =========================================================================
//  Declare stuff for TFT and image buffers
//  NOTE: The LCD connections are declared in the User_Setup.h file in the TFT_eSPI-master library.
// =========================================================================
TFT_eSPI tft = TFT_eSPI();                // Invoke library, pins defined in User_Setup.h

const uint16_t PORTRAIT = 2;              // value used To set the LCD into portrait mode 0 = 0 degrees, 2 = 180 degrees
const uint16_t LANDSCAPE = 3;             // Value used to set the LCD into landscape mode 1 = 90 degrees, 3 = 270 degrees
const uint16_t THERMAL_WIDTH = 80;        // Width of image
const uint16_t THERMAL_HEIGHT = 62;       // Height of image
uint16_t roi_Width = 4;                   // Region of interest in X for temperature anaysis
uint16_t roi_Height = 4;                  // Region of interest in Y for temperature anaysis

const uint32_t BUFF_SIZE = (THERMAL_WIDTH * THERMAL_HEIGHT);   // Buffer size = whole thermal image (no room for header info in this size buffer)

uint16_t draw_buffer[BUFF_SIZE];          // buffer to hold the final image to draw (16 bits per pixel)
uint16_t header_buffer[THERMAL_WIDTH];    // Buffer to hold the header informationr (16 bits per entry)

// =========================================================================
// Declare values used for GIU layout
// The Simple GUI consists of the following items
// 2 lines of guiBanner
// guiLine0 - guiTitle:   guiStatus
// guiLine1 - guiTitle:   guiStatus
// Thermal image + region of interest
// guiLine2 - guiTitle:   guiStatus
// guiLine3 - guiTitle:   guiStatus
// guiLine4 - guiTitle:   guiStatus
// Animation line
// All the items are on a grid and the various positions are controlled by the variables below
// =========================================================================
const uint16_t TFT_COLOR_BACKGROUND = TFT_BLACK;    // Value used to clear text (ie. write item to background colour)
const uint16_t TFT_COLOR_STATUS = TFT_WHITE;  // Value used to draw colour text

const int WINDOW_OFFSET_X = 24;        // Centers thermal image in 160 x 128 dsiplay
const int WINDOW_OFFSET_Y = 51;        // Centers thermal image in 160 x 128 display

const int guiBanner_X = 8;              // X position of top 2 lines of banner information
const int guiBanner_Y = 12;             // Y position of top 2 lines of banner information

const int guiTitle_X = 20;              // X position of the title for each gui item on every line
const int guiValue_X = 60;              // X position of the VALUE for eash gui item of every line
const int guiLine0_Y = 30;              // Y position for the line 0 of gui information
const int guiLine1_Y = 40;              // Y position for the line 1 of gui information
const int guiLine2_Y = 120;             // Y position for the line 2 of gui information
const int guiLine3_Y = 130;             // Y position for the line 3 of gui information
const int guiLine4_Y = 140;             // Y position for the line 4 of gui information

// =========================================================================
//  Declare Registers for MI48TL
// =========================================================================
// First the base i2c address for the MI48TL device
uint16_t mi48tlAddr = 0x40;                 // i2c address for MI48TL (0x40 if MI48TL_ADDR_PIN =0 or 0x41 if MI48TL_ADDR_PIN = 1 )

// Now the addresses for each of the registers within the MI48TL device
const uint16_t MI48TL_FRAME_MODE      = 0xB1; // Frame Mode register address     - Read/Write
const uint16_t MI48TL_SW_VERSION      = 0xB2; // swVersion register address        - Read only
const uint16_t MI48TL_BUILD           = 0xB3; // swVersion build Number register address - Read only
const uint16_t MI48TL_FRAME_RATE      = 0xB4; // Frame Rate register addres      - Read/Write
const uint16_t MI48TL_POWER_DOWN      = 0xB5; // Power down register address     - Read/Write
const uint16_t MI48TL_STATUS_ERROR    = 0xB6; // Status/Error register address   - Read only
const uint16_t MI48TL_SENSOR_TYPE     = 0xBA; // SenXor type register address    - Read only
const uint16_t MI48TL_EMISSIVITY      = 0xCA; // emissivity register address     - Read/Write
// Note filter controls are only available in SW_VERSION >= 2.1 and BUILD > 2 (i.e. 2.1.2)
const uint16_t MI48TL_FILTER_CONTROL      = 0xD0; // filter control (bits 0-2 R/W)
const uint16_t MI48TL_FILTER_SETTINGS_LSB = 0xD1; // filter setting LSB (0x32 default, 0x80 rec)
const uint16_t MI48TL_FILTER_SETTINGS_MSB = 0xD2; // filter setting MSB (0x00 default)
const uint16_t MI48TL_ROLLING_AVG_SETTING = 0xD3; // rolling average setting (0x04 default)

uint16_t mi48tlSerialNum = 0xE0; // SenXor Serial Number register address start (Note: this field is 16 consecutuve bytes) - Read only

// Variables to hold the corresponding register contents
unsigned char frameMode;
unsigned char swVersion;
unsigned char build;
unsigned char frameRate;
unsigned char powerDown;
unsigned char statusError;
unsigned char senxorType;
unsigned char emissivity;
unsigned char serialNum;
uint16_t mi48tlSpiData = 0x0;                // 16 bit variable that is used to store the SPI frame data being read from the MI48TL

// =========================================================================
// Declare Array for Colour lookup palettes
// This is a 2 dimensional array, there are currently 3 lookups + one algorithmic lookup
// Only the lookoups are declared below, each one 256 entries long
// each entry consisting of 5 bits Red, 6 bits green, 5 bits blue (MS-LS)
// =========================================================================

// Variables for algorithmic colour palett conversion
uint16_t red;                             // used in algorimic lookup for red component
uint16_t green;                           // used in algorimic lookup for green component
uint16_t blue;                            // used in algorimic lookup for blue component

// General values
int32_t pixelVal;                         // Value to be converted
int16_t lutIndex;                         // Calculated value to use to index into 256 entry LUT

// INFERNO color palette
const int16_t palette[256] = {
  0x0000, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0002, 0x0022, 0x0022, 0x0822, 0x0823, 0x0823, 0x0823, 0x0824, 0x0824, 0x0844,
  0x0844, 0x0845, 0x1045, 0x1045, 0x1046, 0x1046, 0x1066, 0x1066, 0x1867, 0x1867, 0x1867, 0x1868, 0x1868, 0x1868, 0x2068, 0x2069,
  0x2069, 0x2069, 0x206A, 0x286A, 0x286A, 0x286A, 0x286B, 0x286B, 0x306B, 0x304B, 0x304B, 0x304C, 0x384C, 0x384C, 0x384C, 0x384C,
  0x384C, 0x404C, 0x404D, 0x404D, 0x404D, 0x406D, 0x486D, 0x486D, 0x486D, 0x486D, 0x486D, 0x506D, 0x506D, 0x508D, 0x508D, 0x508D,
  0x588D, 0x588D, 0x588D, 0x588D, 0x58AD, 0x60AD, 0x60AD, 0x60AD, 0x60AD, 0x60AD, 0x68AD, 0x68CD, 0x68CD, 0x68CD, 0x68CD, 0x68CD,
  0x70CD, 0x70CD, 0x70ED, 0x70ED, 0x70ED, 0x78ED, 0x78ED, 0x78ED, 0x78ED, 0x790D, 0x810D, 0x810D, 0x810D, 0x810D, 0x810D, 0x890D,
  0x890D, 0x892D, 0x892D, 0x892D, 0x912D, 0x912D, 0x912D, 0x912C, 0x914C, 0x994C, 0x994C, 0x994C, 0x994C, 0x994C, 0x994C, 0xA16C,
  0xA16C, 0xA16C, 0xA16C, 0xA16C, 0xA96B, 0xA98B, 0xA98B, 0xA98B, 0xA98B, 0xB18B, 0xB18B, 0xB1AB, 0xB1AB, 0xB1AB, 0xB1AA, 0xB9AA,
  0xB9CA, 0xB9CA, 0xB9CA, 0xB9CA, 0xC1CA, 0xC1EA, 0xC1EA, 0xC1E9, 0xC1E9, 0xC209, 0xCA09, 0xCA09, 0xCA09, 0xCA29, 0xCA29, 0xCA28,
  0xCA28, 0xD248, 0xD248, 0xD248, 0xD248, 0xD268, 0xD267, 0xDA67, 0xDA87, 0xDA87, 0xDA87, 0xDAA7, 0xDAA7, 0xDAA6, 0xDAC6, 0xE2C6,
  0xE2C6, 0xE2E6, 0xE2E6, 0xE2E6, 0xE305, 0xE305, 0xE305, 0xEB25, 0xEB25, 0xEB45, 0xEB45, 0xEB44, 0xEB64, 0xEB64, 0xEB64, 0xEB84,
  0xEB84, 0xEBA3, 0xF3A3, 0xF3C3, 0xF3C3, 0xF3C3, 0xF3E3, 0xF3E2, 0xF402, 0xF402, 0xF422, 0xF422, 0xF422, 0xF442, 0xF441, 0xF461,
  0xF461, 0xF481, 0xF481, 0xF4A1, 0xF4A1, 0xF4A1, 0xFCC1, 0xFCC1, 0xFCE1, 0xFCE1, 0xFD01, 0xFD01, 0xFD21, 0xFD21, 0xFD42, 0xFD42,
  0xFD62, 0xFD62, 0xFD62, 0xFD83, 0xFD83, 0xFDA3, 0xFDA3, 0xFDC4, 0xFDC4, 0xF5E4, 0xF5E5, 0xF605, 0xF605, 0xF625, 0xF626, 0xF646,
  0xF646, 0xF667, 0xF667, 0xF687, 0xF688, 0xF6A8, 0xF6A8, 0xF6C9, 0xF6C9, 0xF6EA, 0xF6EA, 0xF70A, 0xF70B, 0xEF0B, 0xEF2C, 0xEF2C,
  0xEF4D, 0xEF4D, 0xEF6E, 0xEF6E, 0xEF6F, 0xEF8F, 0xEF90, 0xF7B0, 0xF7B1, 0xF7B1, 0xF7B2, 0xF7D2, 0xF7D3, 0xF7D3, 0xF7F4, 0xFFF4
};

// =========================================================================
//  VL53L1X TOF sensor declarations
// =========================================================================
uint16_t rangeMM = 0;
uint16_t rangeMMOld = 0;

// =========================================================================
//  Anything else
// =========================================================================
bool dataReady;                             // Boolean to hold the value of DATA_READY pin
uint16_t mi48tlMinVal = 2900;               // Clip below this value on SPI frame date
uint16_t mi48tlMaxVal = 3200;               // Clip Above this value on SPI frame data
uint16_t highlightMinVal = mi48tlMinVal + (mi48tlMaxVal - mi48tlMinVal) / 2;
int32_t tempCorrection = 2731;              // Was 2731 - This is the value to use to convert output to degrees C ((degrees c = (spi data value - tempCorrection)/10)
// Out is in  0.1k and 0k = -2731.5C

uint16_t roiAvgSpiData = 0;                 // holds the sum of the SPI values of all pixels in ROI
float roiTotalFinal = 0;                    // holds the SenXor-reported ROI temperature
float roiAdjustedTemp = 0;                  // holds the range adjusted ROI temperature
float roiTotalFinalOld = 0;
float roiAdjustedTempOld = 0;

/*
  float rangeMult = 0.00166;                  // value used to adjust the ROI temperature based on range
  uint16_t rangeCalib = 70;                   // range (in CM) at which the SenXor module was factory calibrated
*/
// =========================================================================
//  SETUP
// =========================================================================
void setup()
{
  Wire.begin();                               // Initialise and configure the i2C
#ifdef DEBUG
  Serial.begin(115200);                       // Only reguired if we need to use serial for debug
#endif

  // =========================================================================
  //  Initialise the GPIO pins
  // =========================================================================
  pinMode (BACKLIGHT_PIN, OUTPUT);            // configure LCD BACKLIGHT Pin as an output pin
  digitalWrite (BACKLIGHT_PIN, LOW);          // Initialise BACKLIGHT pin low, turn off the screen display whilst we are booting.

  pinMode(S1_LEFT_PIN, INPUT);                // GPIO34 is input only so no pullup available
  pinMode(S2_MIDDLE_PIN, INPUT_PULLUP);
  pinMode(S3_RIGHT_PIN, INPUT);               // GPIO35 is input only so no pullup available

  pinMode(DATA_READY_PIN, INPUT_PULLUP);
  pinMode (nRESET_PIN, OUTPUT);               // o/p Pin connected to nRESET (XXX) on the MI48TL
  digitalWrite (nRESET_PIN, LOW);             // First put the MI48TL in reset - THIS NEEDS TO BE TIMED
  pinMode (MI48TL_ADDR_PIN, OUTPUT);          // o/p connected to PA13 (??) on MI48TlL
  digitalWrite (MI48TL_ADDR_PIN, LOW);        // Set the MI48TL i2c addrees LOW = 0x40 HIGH = 0x41
  pinMode(MI48TL_SS_PIN, OUTPUT);             // Configure SS pin for MI48TL - other pins done by SPI.begin()
  digitalWrite (MI48TL_SS_PIN, HIGH);         // reset the SPI slave select pin to its off state (it is active low)

  pinMode(XSHUT_PIN, OUTPUT);                 // Configure XHUT pin as ouput pin (will also work blue onboard LED)
  digitalWrite (XSHUT_PIN, HIGH);             // Initialise XHUT pin high, enable VL53L1X TOF sensor.

  // =========================================================================
  //  Setup VL53L1X TOF sensor
  // =========================================================================
  Wire.setClock(400000);                      // use 400 kHz I2C
  tofSensor.setTimeout(500);
  if (!tofSensor.init())
  {
#ifdef DEBUG
    Serial.println("Failed to detect and initialize VL53L1X!");
#endif
    while (1);
  }
#ifdef DEBUG
  Serial.println("Found the VL53L1X ok");
#endif

  // Use long distance mode and allow up to 200 ms for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. the default in examle code was 50 ms (50000.
  // See the VL53L1X datasheet for more information on range and timing limits.
  tofSensor.setDistanceMode(VL53L1X::Long);
  tofSensor.setMeasurementTimingBudget(200000);  //was 50000

  // Start continuous readings at a rate of one measurement every 250ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget default in example code was 50ms.
  tofSensor.startContinuous(250);                //was 50

  for (int i = 0; i < THERMAL_WIDTH; i++) {
    header_buffer[i] = 0;                   // Initialise the header buffer
  }

  // =========================================================================
  //  Initialise the LCD and write all the static GUI elements
  // =========================================================================
#ifdef DEBUG
  Serial.println ("started");
#endif
  // Now setup the TFT
  tft.init();                                 // Iniialise the TFT - this should only be done once after each boot
  tft.fillScreen(TFT_BLACK);                  // Clear LCD screen to black
  tft.setRotation(PORTRAIT);                  // Set LCD to potrait mode mode just for text
  tft.setTextColor(TFT_WHITE);                // Set the test colour
  tft.setTextSize(0);                         // Set the txt size

  // Draw Banner info (2 lines at top of screen)
  tft.setCursor(guiBanner_X, guiBanner_Y);    // Set the banner cursor position
  tft.println("Meridian Innovation");         // Banner message at top of screen
  tft.println("      SenXor 080");

  // Draw bottom 3 lines of GUI info (after thermal image and bfore bottom annotation line)
  tft.setCursor(guiTitle_X, guiLine2_Y);      // Set cursor to line 2 gui TITLE
  tft.println("A-ROI:");                      // Write line 2 TITLE (adjusted ROI tamperature)
  tft.setCursor(guiTitle_X, guiLine3_Y);      // Set cursor to line 3 gui TITLE
  tft.println("  ROI:");                      // Write line 3 TITLE (ROI temperature)
  tft.setCursor(guiTitle_X, guiLine4_Y);      // Set cursor to line 4 gui TITLE
  tft.println("Range:");                      // Write line 4 TITLE (Range distance)

  // Draw an outline frame around the thermal image (this does not get overwriten by the thermal image so only gets drawn once)
  tft.drawRect ((WINDOW_OFFSET_X - 1) , (WINDOW_OFFSET_Y - 1), (THERMAL_WIDTH + 2), (THERMAL_HEIGHT + 2), TFT_COLOR_STATUS);

  // =========================================================================
  // Finish of MI48TL reset and setup SPI interface for MI48TL
  // NOTE: The MI48TL and LCD are on differnt SPI busses
  // =========================================================================
  // nRESET has already been driven low when the GPIO was declared
  delay(200);                                 // Wait 0.2 seconds - just to make sure MI48TL has seen nRESET pin low
  digitalWrite (nRESET_PIN, HIGH);            // remove reset to the MI48TL - allow it to boot
  delay(1000);                                // Wait 1 seconds - this is for the MI48TL to boot - may be able to reduce
  SPI.begin();                                // Initialise the SPI - this configures MOSI, MISO, SCK pins etc

  // =========================================================================
  //  Read all the individual i2c registers
  //  Uses ReadI2c() routine
  // =========================================================================
  // Currently these reads are not used - they are only here in case they are required for DEBUG - (could add checking)

  frameMode = ReadI2c(MI48TL_FRAME_MODE);     // Set the address for the Frame mode register
  swVersion = ReadI2c(MI48TL_SW_VERSION);     // Set the address for the Firmware swVersion number register
  build = ReadI2c(MI48TL_BUILD);              // Set the address for the Firmware build Number register
  frameRate = ReadI2c(MI48TL_FRAME_RATE);     // Set the address for the Frame Rate register
  powerDown = ReadI2c(MI48TL_POWER_DOWN);     // Set the address for the Power Down register
  statusError = ReadI2c(MI48TL_STATUS_ERROR); // Set the address for the Status/Error register
  senxorType = ReadI2c(MI48TL_SENSOR_TYPE);   // Set the address for the SenXor Type register
  emissivity = ReadI2c(MI48TL_EMISSIVITY);    // Set the address for the emissivity register

#ifdef DEBUG
  // print out the register dump + the MI48TL serial number
  Serial.println("\nDump of MI48TL registers");

  Serial.print("Addr (0x");
  Serial.print(MI48TL_FRAME_MODE, HEX);
  Serial.print(") - frameMode    = ");
  Serial.println(frameMode, HEX);

  Serial.print("Addr (0x");
  Serial.print(MI48TL_SW_VERSION, HEX);
  Serial.print(") - swVersion      = ");
  Serial.println(swVersion, HEX);

  Serial.print("Addr (0x");
  Serial.print(MI48TL_BUILD, HEX);
  Serial.print(") - build        = ");
  Serial.println(build, HEX);

  Serial.print("Addr (0x");
  Serial.print(MI48TL_FRAME_RATE, HEX);
  Serial.print(") - frameRate    = ");
  Serial.println(frameRate, HEX);

  Serial.print("Addr (0x");
  Serial.print(MI48TL_POWER_DOWN, HEX);
  Serial.print(") - powerDown    = ");
  Serial.println(powerDown, HEX);

  Serial.print("Addr (0x");
  Serial.print(MI48TL_STATUS_ERROR, HEX);
  Serial.print(") - statusError  = ");
  Serial.println(statusError, HEX);

  Serial.print("Addr (0x");
  Serial.print(MI48TL_SENSOR_TYPE, HEX);
  Serial.print(") - senxorType   = ");
  Serial.println(senxorType, HEX);

  Serial.print("Addr (0x");
  Serial.print(MI48TL_EMISSIVITY, HEX);
  Serial.print(") - emissivity   = ");
  Serial.println(emissivity, HEX);

  // =========================================================================
  //  Read the serial number, this is over 16 consecutive addresses in the MI48TL.
  //  Coded directly, does not currently make use of ReadI2c() routine
  // =========================================================================
  Serial.print("addr (0xE0-F) SerialNumber = ");
  Wire.beginTransmission(mi48tlAddr);         // Begin transmission to the Sensor
  Wire.write (mi48tlSerialNum);               // Set the address for the Serial Number register
  Wire.endTransmission();                     // End tranmission We should have set the address of the register
  for (int x = 0; x < 16; x++) {              // Now loop through all 16 bytes
    Wire.requestFrom(mi48tlAddr, 1);          // Tell slave we need to read 1byte from the current register
    serialNum = Wire.read();                  // Read that Serial Number byte (register will auto increment)
    Serial.print(serialNum, HEX);             // Print next character in serial number
    Serial.print(", ");
  }
  Serial.println();
  Serial.println();
#endif

  Wire.endTransmission(true);                 // Release i2c bus - so others can use it (can have multiple slaves & masters connected

  // =========================================================================
  // Write any registers required beofre starting exitig setup
  // and starting Data aquisition
  // =========================================================================
  WriteI2c(MI48TL_FILTER_SETTINGS_LSB, 0x80);
  WriteI2c(MI48TL_FILTER_SETTINGS_MSB, 0x00);  
  WriteI2c(MI48TL_FILTER_CONTROL, 0x03);  
  delay(100); // currently required after modifying filter values    
  WriteI2c(MI48TL_FRAME_RATE, 0x1);           // Write the Frame_rate register 0x1 = as fast as possible (30FPS)
  WriteI2c(MI48TL_FRAME_MODE, 0x3);           // Write the Frame_mode register 0x3 = capture continuous with header)
  digitalWrite (BACKLIGHT_PIN, HIGH);         // Screen has been setup so turn on the backlight
}

// =========================================================================
//  Main loop
// =========================================================================
void loop()
{  
  // read LV53LV1X TOF sensor, we probably need to waitt for data ready so it should not waste too much time
  rangeMM = tofSensor.read(false);              //read the TOF sensor non blocking
  if (rangeMM == 0) {                           // will return 0 if new data is not ready yet
    rangeMM = rangeMMOld;                       // No new data so restore the last value and keep goin
  }
  rangeMM = rangeMM / 10;                       // We got new data - it is in Milli meters so convert to centimeters

#ifdef DEBUG
  Serial.print(rangeMM);
  if (tofSensor.timeoutOccurred()) {
    Serial.print(" TIMEOUT");
  }
  Serial.println();
#endif

  dataReady = digitalRead(DATA_READY_PIN);       // Read the state on DATA_READY line
  if ( digitalRead (DATA_READY_PIN) == HIGH) {   // Wait for DATA_READY to assert

#ifdef DEBUG
    Serial.println ("Data ready!!");
#endif

    //DATA_READY has been asserted so data is now available on the MI48TL SPI bus
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));  // set up the SPI transfer for the MI48TL requirements (10MHz)
    digitalWrite (MI48TL_SS_PIN, LOW);                                 // Assert the active low SPI Slave Select pin to MI48TL
    for (int i = 0; i < THERMAL_WIDTH; i++) {                          // Read the header data (1 rows worth)
      header_buffer[i] = SPI.transfer16(0x0);                          // write data to the header buffer
    }
    for (int j = 0; j < THERMAL_HEIGHT; j++) {                         // outer loop = j do this for every line (row) in the image
      for (int i = 0; i < THERMAL_WIDTH; i++) {                        // Inner loop - i - stay in this loop whilst we read a ROW
        mi48tlSpiData = SPI.transfer16(0x0);                           // send out 16 bits of zeros and recieve new 16 bit data word from MI48TL

        // Region of Interest Calculation (ROI)
        // Check to see if this data word is within the ROI bounding boc
        if (  (j >= (THERMAL_HEIGHT / 2) - (roi_Height / 2)) &&
              (j < (THERMAL_HEIGHT / 2) + (roi_Height / 2)) &&
              (i >= (THERMAL_WIDTH / 2) - (roi_Width / 2)) &&
              (i < (THERMAL_WIDTH / 2) + (roi_Width / 2))) {
          roiAvgSpiData += mi48tlSpiData;
        }
        // The data from the sensor is flipped with respect to the screen so we have to draw the rows backwards
        draw_buffer[((THERMAL_WIDTH - 1) - i) + (j * THERMAL_WIDTH)] = mi48tlSpiData;
      }                                                                // End inner loop (finished a row)
    }                                                                  // End of outer loop
    // Average the ROI SPI data values
    roiAvgSpiData /= (roi_Width * roi_Height);
    // We have now read the entire frame of data
    digitalWrite (MI48TL_SS_PIN, HIGH);                                // De-Assert the active low SPI Slave Select pin to MI48TL
    SPI.endTransaction();                                              // Give up the SPI bus - the TFT screen will need it

    // =========================================================================
    // Colour conversion - one pixel at a time
    // The draw_buffer starts as 16 bit sensor data
    // At the end it is 16 bit RGB (5-6-5)
    // =========================================================================
    for (int j = 0; j < THERMAL_HEIGHT; j++) {                              // outer loop
      for (int i = 0; i < THERMAL_WIDTH; i++) {                             // Inner loop - i - stay in this loop whilst we read a ROW
        pixelVal = draw_buffer[(i) + (j * THERMAL_WIDTH)];                  //pixelVal  = the data sample to test.
        if (pixelVal <= mi48tlMinVal) {
          lutIndex = 0;                                                     //data is below absolute minimum so set LUT index to low default;
        }
        else if (pixelVal >= mi48tlMaxVal) {
          lutIndex = 255;                                                   //data is above absolute maximun so set LUT index high default;
        }
        else  {
          lutIndex = map (pixelVal, mi48tlMinVal, mi48tlMaxVal , 0, 0xff);  // Data is in range so map it LUT index between low and high defaults
        }
        draw_buffer[(i) + (j * THERMAL_WIDTH)] = palette[lutIndex]; //now lookup the correct entry in the selected palette
      }                                                                     // End of inner loop
    }                                                                       // End of outer loop

    // =========================================================================
    // Now send frame data to the screen and sort out the rolling buffer pointer
    // Set up the window to draw on the LCD screen this is in form (X_Start, Y_Start, X_End, Y_End)
    // =========================================================================
    tft.setAddrWindow(WINDOW_OFFSET_X, WINDOW_OFFSET_Y, THERMAL_WIDTH, THERMAL_HEIGHT);
    tft.pushColors(draw_buffer, BUFF_SIZE);                                 // Now send the data to the TFT screen

    // calculate the average temperature within the ROi
    roiTotalFinal = ((float)roiAvgSpiData - 2731.f) / 10.f;

    // calculate what the ROI temperature is - adjusted for range
    roiAdjustedTemp = objectTemperatureByDistance(roiTotalFinal, ((float)rangeMM) / 1000, A_REF, B, T_REF);

    // =========================================================================
    // Any per-frame annotation should go in here
    // Write new status data to screen
    // The old value is first erased by writing it to background colour
    // Then the new value is writen in the gui status colour
    // =========================================================================
    // Set the global parameters for this operation
    tft.setRotation(PORTRAIT);                                          // Set LCD to potrait mode for text
    tft.setTextSize(0);                                                 // Set the text size

    //--------------------------------- Update GUI line 2 --------------------------------------------------------------
    // first rewite the old value in the background colour to erase it
    tft.setTextColor(TFT_COLOR_BACKGROUND);                                    // make the text colour = gui background colour
    tft.setCursor(guiValue_X, guiLine2_Y);                              // set cursor to VALUE item of line 0
    tft.print(roiAdjustedTempOld, 1);                                   // output the last value to set it back to background (1 decimal place)
    tft.print((char)247);                                               // Degree symbol
    tft.print("C");
    // Now write the new value in the the foreground colour
    tft.setTextColor(TFT_COLOR_STATUS);                                  // Set the font colour to draw the new status info
    tft.setCursor(guiValue_X, guiLine2_Y);                              // set cursor to VALUE item of line 0
    tft.print(roiAdjustedTemp, 1);                                      // output the new value (1 decimal place)
    tft.print((char)247);                                               // Degree symbol
    tft.print("C");

    //--------------------------------- Update GUI line 3 --------------------------------------------------------------
    // first rewite the old value in the background colour to erase it
    tft.setTextColor(TFT_COLOR_BACKGROUND);                                    // make the text colour = gui background colour
    tft.setCursor(guiValue_X, guiLine3_Y);                              // set cursor to VALUE item of line 0
    tft.print(roiTotalFinalOld, 1);                                     // output the last value to set it back to background (1 decimal place)
    tft.print((char)247);                                               // Degree symbol
    tft.print("C");
    // Now write the new value in the the foreground colour
    tft.setTextColor(TFT_COLOR_STATUS);                                  // Set the font colour to draw the new status info
    tft.setCursor(guiValue_X, guiLine3_Y);                              // set cursor to VALUE item of line 0
    tft.print(roiTotalFinal, 1);                                        // output the new value (1 decimal place)
    tft.print((char)247);                                               // Degree symbol
    tft.print("C");

    //--------------------------------- Update GUI line 4 --------------------------------------------------------------
    // first rewite the old value in the background colour to erase it
    tft.setTextColor(TFT_COLOR_BACKGROUND);                                    // make the text colour = gui background colour
    tft.setCursor(guiValue_X, guiLine4_Y);                              // set cursor to VALUE item of line 0
    tft.print(rangeMMOld);                                              // output the last value to set it back to background (2 decimal place2)
    tft.print(" CM");
    // Now write the new value in the the foreground colour
    tft.setTextColor(TFT_COLOR_STATUS);                                  // Set the font colour to draw the new status info
    tft.setCursor(guiValue_X, guiLine4_Y);                              // set cursor to VALUE item of line 0
    tft.print(rangeMM);                                                 // output the new value (2 decimal places)
    tft.print(" CM");

    //--------------------------------- Draw ROI box ------------------------------------------------------------------
    tft.drawRect ((WINDOW_OFFSET_X + (THERMAL_WIDTH / 2) - (roi_Width / 2)), (WINDOW_OFFSET_Y + (THERMAL_HEIGHT / 2) - (roi_Height / 2)), roi_Width, roi_Height, TFT_WHITE);

    //--------------------------------- Draw the Range bar ------------------------------------------------------------
    // The range bar is drawn in the animation line (currently all magic numbers)
    tft.fillRect (40, 150, rangeMMOld / 2, 4, TFT_COLOR_BACKGROUND);            // Erase the old bar first
    if ((rangeMM <= 120) && (rangeMM >= 50)) {
      tft.fillRect (40, 150, rangeMM / 2, 4, TFT_GREEN);                  // Range is within bounds drw it in Green
    }
    else {
      tft.fillRect (40, 150, rangeMM / 2, 4, TFT_RED);                    // Range is not within bounds, draw it in red
    }

    // =========================================================================
    // tidy up all variables at end of frame
    // =========================================================================

    // Reset the variables ready for next frame
    rangeMMOld = rangeMM;                                               // Update the range to the new value    
    roiTotalFinalOld = roiTotalFinal;
    roiAdjustedTempOld = roiAdjustedTemp;
    roiAvgSpiData = 0;
  }                                                                       // loop to here if DATA_READY not asserted
}                                                                         // end of loop()

// =========================================================================
//  Function to write i2c register
// =========================================================================
void WriteI2c(int RegAddr, unsigned char RegData)
{
  Wire.beginTransmission(mi48tlAddr);       // Begin transmission to the Sensor
  Wire.write (RegAddr);                     // Set the address for the requested register
  Wire.write (RegData);                     // Write the data for that register
  Wire.endTransmission(true);               // Release i2c bus - so others can use it (can have multiple slaves & masters connected
  return;
}

// =========================================================================
//  Function to read i2c register
// =========================================================================
unsigned char ReadI2c(int RegAddr)
{
  unsigned char Result;
  Wire.beginTransmission(mi48tlAddr);       // Begin transmission to the Sensor
  Wire.write (RegAddr);                     // Set the address for the requested register
  Wire.endTransmission();                   // End tranmission We should have set the address of the register
  Wire.requestFrom(mi48tlAddr, 1);          // Tell slave we need to read 1 byte from the current register
  Result = Wire.read();                     // read that Serial Number byte (register will auto increment)
  Wire.endTransmission(true);               // Release i2c bus - so others can use it (can have multiple slaves & masters connected
  return Result;
}

/*
  Calulate distance-adjusted temperature by distance.
  ========================================
  For more details, please refer to Application Note:
  "Distance Correction of Temperature Readout" by Meridian Innovation Limited (NDA required)
  Section: "Compensation Done By Host System"
  ========================================
  t_ro = t_obj + a(t_obj) * x    ---- (1)
  a(t) = a_ref + b * (t - t_ref) ---- (2)
  
  where
    t_obj       is the "true" temperature of the object to be calculated
    t_ro        is the temperature of the object as measured (readout) by the sensor
    x           is the distance from the object from the sensor in meters (m)
    a(t)        is the linear function describing the attenuation of temperature with distance
    a_ref       is the value of a(t_ref)
    t_ref       is a reference temperature of the object at a known distance
    b           is a constant that describes the gradient of the linear function a(t)   
  
  Combining (1) and (2) gives the formula provided in the function below.    
*/
float objectTemperatureByDistance(float t_ro, 
                                  float x,        
                                  float a_ref,    
                                  float b,
                                  float t_ref) {
/*  
  return (temperatureReadout - (-1.15 * distanceInMeters) + (35 * -0.074 * distanceInMeters)) / (1 + (-0.074 * distanceInMeters));
*/
  return (t_ro - (a_ref * x) + (b * t_ref * x)) / (1 + b * x);
}
