//***************************************************************************************************
// ESP32_stoflamp -- main.cpp -- Main program file.                                                 *
//***************************************************************************************************
// Fijnstofmeter met lichtweergave.                                                                 *
// Compile for DOIT ESP32 DEVKIT V1                                                                 *
//***************************************************************************************************
// 14-03-2019, ES: Eerste opzet.                                                                    *
// 22-04-2019, ES: Introduced AsyncWebServer.                                                       *
// 05-04-2019, ES: LoRa en WiFi communicatie erbij.                                                 *
// 19-04-2019, ES: PlatformIO version.                                                              *
// 03-05-2019, ES: Reconnect WiFi indien het weggevallen is.                                        *
// 08-05-2019, ES: Voorkom tegelijkertijd zenden van WiFi en LoRa.                                  *
// 14-05-2019, ES: Eén semaphore voor alle I/O.                                                     *
//***************************************************************************************************
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <freertos/task.h>              // For multitasking
#include <freertos/queue.h>             // For queues
#include <Wire.h>                       // Voor display
#include <SparkFunHTU21D.h>             // Voor temp/humidity sensor
#include <lmic.h>                       // MMCI LoRaWAN LMIC library
#include <hal/hal.h>
#include <time.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <WiFiMulti.h>
#include <WiFiClient.h>
#include <AsyncTCP.h>                   // Download https://github.com/me-no-dev/AsyncTCP
#include <ESPAsyncWebServer.h>          // Download https://github.com/me-no-dev/ESPAsyncWebServer
#include <lwip/dns.h>
#include <ESPmDNS.h>                    // Maakt STOFLAMP.local bereikbaar
#include <nvs.h>                        // NVS voor opslag instellingen
#include <esp_partition.h>              // Lezen van de NVS partitie
#include <SPIFFS.h>                     // File system for onboard flash
#include <ArduinoOTA.h>                 // Update over the air
#include <TinyGPS++.h>                  // Voor GPS
#include <driver/ledc.h>                // Voor IR output
#include "nrf_driver.h"                 // Voor NRF24L01 functions
#include "defaultprefs.h"               // Voorbeeld instellingen

//***************************************************************************************************
// Diverse definities.                                                                              *
//***************************************************************************************************
#define LEDC_BASE_FREQ     38000            // IR op 38 kHz voor VS1838B

#define SCREEN_WIDTH   128                  // OLED display width, in pixels
#define SCREEN_HEIGHT   64                  // OLED display height, in pixels

// Access point name als de connectie met WiFi network niet lukt.
// Het is tevens de hostname for WiFi and OTA.
// Merk op dat het password van een AP tenminste 8 tekens moet hebben.
#define NAME "STOFLAMP"
// Define the version number, also used for webserver as Last-Modified header and to
// check version for update.  The format must be exactly as specified by the HTTP standard!
#define VERSION     "Tue, 14 May 2019 15:55:00 GMT"

#define MAXKEYS       200                   // Max aantal NVS keys in tabel
#define NVSBUFSIZE    150                   // Maximale lengte in een NVS parameter

#define DATSIZ          7                   // Aantal 32 bits getallen in payload WiFi

#define FORMAT_SPIFFS_IF_FAILED true        // Creeer SPIFFS als er geen is

// Definitie voor update software (download van externe server)
//                    Filename         NVS key met versie
//                    ---------------  ------------------
#define UPDATEBIN     "/firmware.bin", "lst_sketch"
#define UPDATEP1      "/Chart.min.js", "lst_chart"
#define UPDATEP2      "/gauge.min.js", "lst_gauge"
#define UPDATEP3      "/about.html",   "lst_about"
#define UPDATEP4      "/config.html",  "lst_config"
#define UPDATEP5      "/index.html",   "lst_index"
#define UPDATEP6      "/favicon.ico",  "lst_favicon"

// Definities voor LoRa communicatie
#define LORA_PORT    11
#define LORA_FW      200                  // Firmware version

// Definities voor NRF24L01 verbinding
#define  NETWORK     "STFL"               // Network name
#define  RF_CHAN     12                   // RF channel to be used
#define  REMNODE     2                    // Send to node 2

// Definities voor I2C devices: Oled en temp/vocht sensor.
#define  OLED_ADR   0x3C                 // OLED I2C address
#define  HTU21D_ADR 0x40                 // HTU21D I2C address

// Tijd van de dag
#define UTC_OFFSET 2                     // Tijdsverschil met UTC (aantaal uren)


//***************************************************************************************************
// PIN definities.                                                                                  *
//***************************************************************************************************
// GPIO 16 en 17 (RX2 en TX2) zijn gereserveerd voor aansluiting GPS                                *
//***************************************************************************************************
#define IR_LED_PIN     15

// I2C signals for display
#define PIN_SDA        21                   // I2C voor display, data
#define PIN_SCL        22                   // I2C voor display, clock

#define PIN_DIM        12                   // Drukknop om lichtsterkte te regelen

// Pins voor LORA
// Pins 18, 19 en 23 vormen de SPI bus, repectievelijk SCK, MISO en MOSI
#define LORA_CS         2                   // Chip select RFM95
#define LORA_DIO0       4                   // RX done, TX done van RFM95
#define LORA_DIO1       5                   // RX time-out van RFM95

#define DUST_PM10_PIN  13                   // Dust sensor PM10 pin

// Pins voor NRF24L01, gebruikt ook SPI
#define NRF24_CE_PIN   25
#define NRF24_CSN_PIN  26

//***************************************************************************************************
// Diverse benodigde structs.                                                                       *
//***************************************************************************************************
struct pwm_struct                           // Voor PWM/LED sturing
{
  uint8_t  channel ;                        // PWM kanaal
  uint8_t  pin ;                            // Output pin
} ;

enum comm_qdata_type { QDATA, QUPDATE } ;   // datatyp in qdata_struct for regtask
struct qdata_struct
{
  int datatyp ;                             // Identifier
  int parameter ;                           // Parameter, ???
} ;

struct WifiInfo_t                           // Voor lijst with WiFi info
{
  uint8_t inx ;                             // Index zoals in "wifi_00"
  char * ssid ;                             // SSID voor een WiFi netwerk
  char * passphrase ;                       // Passphrase voor dat netwerk
  bool   accessable ;                       // True indien netwerk aanwezig
} ;

struct nvs_entry                            // In de ESP32 opgeslagen NVS record
{
  uint8_t  Ns ;                             // Namespace ID
  uint8_t  Type ;                           // Type of value
  uint8_t  Span ;                           // Number of entries used for this item
  uint8_t  Rvs ;                            // Reserved, should be 0xFF
  uint32_t CRC ;                            // CRC
  char     Key[16] ;                        // Key in Ascii
  uint64_t Data ;                           // Data in entry
} ;

struct nvs_page                             // Pagina met NVS entries
{ // 1 page is 4096 bytes
  uint32_t  State ;
  uint32_t  Seqnr ;
  uint32_t  Unused[5] ;
  uint32_t  CRC ;
  uint8_t   Bitmap[32] ;
  nvs_entry Entry[126] ;
} ;

struct keyname_t                            // Voor keys in NVS
{
  char      Key[16] ;                       // Max lengte is 15 plus 1 delimeter
} ;

//***************************************************************************************************
// Forward declarations.                                                                            *
//***************************************************************************************************
const char* analyzeCmd ( const char* str ) ;
const char* analyzeCmd ( const char* par, const char* val ) ;
uint8_t FindNsID ( const char* ns ) ;
void fillkeylist() ;
void showstronoled ( const char* str ) ;
void testing() ;
void  mk_lsan() ;
void listNetworks() ;
String readprefs ( bool output ) ;
void otastart() ;
void otaerror ( ota_error_t error ) ;
void otaprogress ( unsigned int progress, unsigned int total ) ;
bool connectwifi ( bool oledreport ) ;
void handle_spec ( AsyncWebServerRequest *request ) ;
void handleFileRead ( AsyncWebServerRequest *request ) ;
void regtask ( void * parameter ) ;
void wifitask ( void * parameter ) ;
void loratask ( void * parameter ) ;
void do_leds() ;
String getContentType ( String filename ) ;
void writeprefs ( AsyncWebServerRequest *request ) ;
bool update_software ( const char* dwlfile, const char* lstupd ) ;
bool nvssearch ( const char* key ) ;
void nvsopen() ;
String nvsgetstr ( const char* key ) ;
esp_err_t nvssetstr ( const char* key, String val ) ;
void fillttnkey ( uint8_t* ttnkey, uint8_t klen, const char* str ) ;
uint32_t a8tohex ( const char* hstr ) ;
uint8_t hex ( char h ) ;


//***************************************************************************************************
// Globale variabelen.                                                                              *
//***************************************************************************************************
// Dust Sensor variabelen
uint32_t          intcount ;                  // Teller aantal interrupts van fijnstof sensor
uint32_t          countSET   = 0 ;            // Tijd in usec A2 is SET (pulse up time)
uint32_t          countRESET = 0 ;            // Tijd in usec A2 is RESET (pulse down time)
float             fijnstof ;                  // Gemeten hoeveelheid fijnstof
uint32_t          starttime = 0 ;
boolean           valP2 = HIGH ;
boolean           triggerP2 = false ;
float             ratioP2 = 0 ;
uint32_t          sampletime_ms = 20000 ;     // 1 meting per 20 seconden

// Voor middeling van stofmeting:
long              NumMelding = 0 ;

// Voor IR-LED
uint8_t           chksum ;                    // Checksum IR bericht
uint8_t           rgbMessage[3] ;             // Values for RGB LEDs

// Voor OLED:
Adafruit_SSD1306 display ( SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1 ) ;

// Voor WiFi/webserver
int               numSsid ;                    // Aantal beschikbare WiFi networks
WiFiMulti         wifiMulti ;                  // Bruikbare WiFi networks
AsyncWebServer    webserver ( 80 ) ;           // Instance of embedded webserver, port 80
WiFiClient        otaclient ;                  // Client voor OTA
bool              NetworkFound = false ;       // True indien WiFi network connected
String            networks ;                   // Found networks in the surrounding
String            ipaddress ;                  // Eigen IP-address
std::vector<WifiInfo_t> wifilist ;             // Lijst with wifi_xx info
uint16_t          reconcount = 0 ;             // Number of WiFi reconnects

// Voor NVS (Non Volatile Storage)
nvs_page                nvsbuf ;               // Space for 1 page of NVS info
const esp_partition_t*  nvs ;                  // Pointer to partition struct
esp_err_t               nvserr ;               // Error code from nvs functions
uint32_t                nvshandle = 0 ;        // Handle for nvs access
uint8_t                 namespace_ID ;         // Namespace ID found
char                    nvskeys[MAXKEYS][16] ; // Ruimte voor NVS keys

// Voor registratie task
int16_t           regint = 60 ;                // Registratie tijd interval in seconden
int16_t           cregint = 60 ;               // Counter voor de timing, 1e keer 60 sec.
uint32_t          regdata[DATSIZ] ;            // Data te registreren LoRa/WiFi

// Voor LoRa communicatie
uint8_t           lorabuf[50] ;                // Space for LoRa data
bool              lora = false ;               // Neem aan: LoRa transfer niet active
TaskHandle_t      xloratask ;                  // Task handle for LoRa communications task
QueueHandle_t     loraqueue ;                  // Queue for loratask
bool              lorabusy = false ;           // LoRa handling is busy
uint8_t           mydatalen = 0 ;              // Will be real data length for LoRa
bool              OTAA = true ;                // Use LoRa OTAA if keys unknown
osjob_t           sendjob ;
String            host ;                       // Target host WiFi registratie
String            post ;                       // POST string
char              loraOK = ' ' ;               // Wordt "L" indien okay

// LoRa stuff ABP.  Example.  Keys will be overwritten by data from NVS and/or OTAA.  Origin is TTN.
uint8_t           NWKSKEY[16] = { 0xBC, 0x50, 0x7C, 0x7D, 0x28, 0xDD, 0x14, 0x96,
                                  0x85, 0xBF, 0x22, 0x31, 0x11, 0x64, 0xF3, 0xFF } ;
uint8_t           APPSKEY[16] = { 0x32, 0x97, 0x90, 0x6D, 0x7D, 0x0F, 0xD6, 0x48,
                                  0xCE, 0x45, 0xE5, 0x8E, 0x88, 0x02, 0x32, 0xFF } ;
uint32_t          DEVADDR = 0x26012082 ;
// For LoRa OTAA.  Origin is TTN.
u1_t              DEVEUI[8]  = { 0x00, 0xF7, 0x30, 0x95, 0xAB, 0x5D, 0x30, 0xF8 } ;
u1_t              APPEUI[8]  = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x01, 0xA5, 0x17 } ;
u1_t              APPKEY[16] = { 0x63, 0xD6, 0xD7, 0xAF, 0xB4, 0x32, 0x8E, 0x60,
                                 0x3B, 0xBF, 0x1B, 0x7F, 0x65, 0xA3, 0x54, 0xAB } ;

const lmic_pinmap lmic_pins =                  // Pins used for RM95 control
      {
        .nss = LORA_CS,                        // Chip select
        .rxtx = LMIC_UNUSED_PIN,               // RX/TX not used
        .rst = LMIC_UNUSED_PIN,                // Reset
        .dio = { LORA_DIO0,                    // DIO0 used
                 LORA_DIO1,                    // DIO1 used
                 LMIC_UNUSED_PIN               // DIO2 not used
               }
      } ;

// Voor WiFi communicatie
bool              wifi = true ;                // Neem aan: WiFi transfer active
TaskHandle_t      xwifitask ;                  // Task handle for WiFi communications task
QueueHandle_t     wifiqueue ;                  // Queue for wifitask
char              wifiOK = ' ' ;               // Wordt "W" indien okay
uint16_t          lamp_id = 0 ;                // Lamp identificatie

// Voor OTA
bool              updatereq = false ;          // Request to update software from remote host

// Voor http handling
bool              resetreq = false ;           // Request to reset the ESP32

// Voor HTDU21D temperatuur/vochtigheid sensor
HTU21D            htu ;                        // Instance voor de HTU21D class
float             temperatuur ;                // Temperatuur in graden Celcius
float             vochtigheid ;                // Vochtigheid in procent (relatief)

// Voor GPS
TinyGPSPlus       gps ;                        // GPS class instance
double            gps_lat ;                    // Latitude
double            gps_lon ;                    // Longitude
String            gps_lat_s ;                  // Latitude als string
String            gps_lon_s ;                  // Longitude as string

// Voor software update
String            upd_server ;                 // Host van software download
String            upd_map ;                    // Map op de host waar files staan

// Debugging
uint8_t           STDEBUG = 1 ;                // Debug on/off

// Voor DIM button
uint16_t          clickcount = 0 ;             // Increment per DIM button click
uint16_t          clickcount2 = 0 ;            // Idem for test
uint16_t          clickcount3 = 0 ;            // Idem for test
bool              singleclick = false ;        // True bij enkele click gedetecteerd
bool              doubleclick = false ;        // True bij dubbele click gedetecteerd
bool              longclick = false ;          // True bij lange click gedetecteerd
float             dimlevel = 50.0 ;            // Range of PWM, 0 to 100

// Timing
hw_timer_t*       sttimer = NULL ;             // Voor timer
bool              secondelater = false ;       // True indien er 1 seconde is verstreken
uint32_t          seconds = 0 ;                // Seconds counter, free running


// Diversen
TaskHandle_t      xmaintask ;                  // Taskhandle for main task
TaskHandle_t      xregtask ;                   // Taskhandle for registration task
QueueHandle_t     regqueue ;                   // Queue for regtask
SemaphoreHandle_t dbgsem = NULL ;              // For exclusive usage of dbgprint
SemaphoreHandle_t IOsem = NULL ;               // For exclusive usage of I/O (SPI, I2C, WiFi)
uint16_t          xregcount = 0 ;              // Loop counter for registration task
uint16_t          xcommcount = 0 ;             // Loop counter for communication task
uint16_t          xloracount = 0 ;             // Loop counter for LoRa communication task
uint16_t          xwificount = 0 ;             // Loop counter for WiFi communication task
bool              g_data_avail = false ;       // Alle metingen beschikbaar


//******************************************************************************************
//                                  D B G P R I N T                                        *
//******************************************************************************************
// Send a line of info to serial output.  Works like vsprintf(), but checks STDEBUG flag.  *
// Print only if STDEBUG flag is true.  Always returns the the formatted string.           *
//******************************************************************************************
char* dbgprint ( const char* format, ... )
{
  static char sbuf[130] ;                                // For debug lines
  va_list varArgs ;                                      // For variable number of params

  if ( xSemaphoreTake ( dbgsem, 20 ) == pdTRUE  )        // Claim resource
  {
    va_start ( varArgs, format ) ;                       // Prepare parameters
    vsnprintf ( sbuf, sizeof(sbuf), format, varArgs ) ;  // Format the message
    va_end ( varArgs ) ;                                 // End of using parameters
    if ( STDEBUG )                                       // DEBUG on?
    {
      Serial.print ( "D: " ) ;                           // Yes, print prefix
      Serial.println ( sbuf ) ;                          // and the info
    }
    xSemaphoreGive ( dbgsem ) ;                          // Release resource
  }
  else
  {
    strcpy ( sbuf, "dbgprint not available" ) ;          // Not available
  }
  return sbuf ;                                          // Return stored string
}


//******************************************************************************************
//                                      C L A I M I O                                      *
//******************************************************************************************
// Claim the I/O resouce.  Uses FreeRTOS semaphores.                                       *
// In order to prevent interference between the various I/O, a semaphore is used.          *
// The resource will be claimed before using SPI, I2C, IR, WiFi send.                      *
//******************************************************************************************
void claimIO ( const char* p )
{
  const        TickType_t ctry = 10 ;                       // Time to wait for semaphore
  uint32_t     count = 0 ;                                  // Wait time in ticks

  while ( xSemaphoreTake ( IOsem, ctry ) != pdTRUE  )       // Claim I/O resource
  {
    if ( count++ > 10 )
    {
      dbgprint ( "I/O semaphore not taken within %d ticks by CPU %d, id %s",
                 count * ctry,
                 xPortGetCoreID(),
                 p ) ;
    }
  }
}


//************************************************************************************
//                                   R E L E A S E I O                               *
//************************************************************************************
// Free the the I/O resource.  Uses FreeRTOS semaphores.                             *
//************************************************************************************
void releaseIO()
{
  xSemaphoreGive ( IOsem ) ;                            // Release I/O resource
}


//************************************************************************************
//                                   B I T S T R E A M                               *
//************************************************************************************
// Class that writes signed or unsigned integers with any bit size (not requiring    *
// multiples of 8) to a buffer. This class in intended to be used for building       *
// networking packets.                                                               *
//************************************************************************************
class BitStream
{
  public:
    BitStream ( uint8_t *buf, size_t len ) ;
    bool append ( uint32_t v, size_t bits ) ;
    void reset() ;

    size_t byte_size() ;
    size_t bit_size() ;
    size_t free_bits() ;
    uint8_t *data() ;

  private:
    uint8_t *buf ;                  // External buffer to write to
    size_t buflen ;                 // Length of buffer in bytes
    size_t pos = 0 ;                // Bit position of next write
} ;

inline BitStream::BitStream(uint8_t *buf, size_t len)
  : buf(buf), buflen(len) { }

inline size_t BitStream::byte_size()
{
  return ( this->pos + 7 ) / 8 ;
}

inline size_t BitStream::bit_size()
{
  return this->pos ;
}

inline size_t BitStream::free_bits()
{
  return buflen * 8 - bit_size() ;
}

inline uint8_t *BitStream::data()
{
  return this->buf ;
}

inline void BitStream::reset()
{
  this->pos = 0 ;
}

//************************************************************************************
//                      B I T S T R E A M : : A P P E N D                            *
//************************************************************************************
// Append a new variable to the stream, with the given size in bits.                 *
// This works for both signed and unsigned values, since signed values               *
// will get sign-extended to 32 bits, so (provided the signed value fits             *
// in a bitsize-bits signed integer) all uppper bits down to and                     *
// including bit number (bitsize - 1) contain the sign bit.                          *
//                                                                                   *
// Returns true when the data was appended, false when there was                     *
// insufficient room in the buffer.                                                  *
//************************************************************************************
inline bool BitStream::append ( uint32_t v, size_t bitsize )
{
  // Sanity check
  if ( bitsize > sizeof(v) * 8 )
  {
    return false ;
  }
  // The value is written to buf starting from the lowest bits working
  // upward, so start from the last byte
  uint8_t *ptr = this->buf + ( this->pos + bitsize - 1 ) / 8 ;
  // Do not overfow the buffer
  if ( ptr >= this->buf + this->buflen )
  {
    this->pos -= bitsize ;
    return false ;
  }
  this->pos += bitsize ;
  // Find out how many bits there are in the last partial byte to be
  // written to buf. This can be 0 if the lsb from v lines up with bytes
  // in the buffer.
  uint8_t partial = this->pos % 8 ;

  // If partial is zero, v is already byte aligned
  if (partial)
  {
    // To bytewise align v with buf, v needs to be shifted left this much
    uint8_t shift = ( 8 - partial ) ;
    // If we can shift without losing bits, do so
    if ( shift + bitsize < sizeof(v) * 8 )
    {
      v <<= shift ;
      bitsize += shift ;
    }
    else
    {
      // If we cannot shift so far left, handle the lowest byte separately
      // and then shift right.
      *ptr = v << shift ;
      ptr-- ;
      // Remove the bits from v
      bitsize -= partial ;
      v >>= partial ;
    }
  }
  while ( bitsize >= 8 )
  {
    // Write a full byte to the buffer
    *ptr = v & 0xff ;
    ptr-- ;
    // Remove the byte from v
    v >>= 8 ;
    bitsize -= 8 ;
  }
  if ( bitsize )
  {
    // Write the uppermost partial byte. Mask off all but the bottom
    // bitsize bits and add them to the existing bits in buf.
    uint8_t mask = ~( 0xff << bitsize ) ;
    *ptr |= v & mask ;
  }
  return true ;
}



//************************************************************************************
//                           H A N D L E D U S T I N T E R R U P T                   *
//************************************************************************************
// Wordt gestart door een level change van de stofmeter output.                      *
//************************************************************************************
void IRAM_ATTR handleDustInterrupt()
{
  static uint32_t t0 ;                            // Vorige timerstand
  uint32_t        t1 ;                            // Huidige timerstand
  uint32_t        delta ;                         // Tijdverschil in microsec

  t1 = micros() ;                                 // Get current time
  delta = t1 - t0 ;                               // Bepaal verlopen tijd
  t0 = t1 ;                                       // Voor de volgende keer
  if ( delta < 1500000L )                         // Redelijke tijd?
  {
    intcount++ ;                                  // Ja, tel als een interrupt
    if ( digitalRead ( DUST_PM10_PIN ) == HIGH )  // Laag naar hoog overgang?
    {
      countRESET = delta ;                        // Ja, set laag tijd
    }
    else
    {
      countSET = delta ;                          // Nee, set hoog tijd
    }
  }
}


//**************************************************************************************************
//                                     S C A N S E R I A L                                         *
//**************************************************************************************************
// Listen to commands on the Serial inputline.                                                     *
//**************************************************************************************************
void scanserial()
{
  static char   cmd[100] ;                       // Command from Serial
  static int    cmdinx = 0 ;                     // Index in cmd
  char          c ;                              // Input character
  const char*   reply ;                          // Reply string from analyzeCmd

  static uint16_t clickcount_old = 0 ;

  while ( Serial.available() )                   // Any input seen?
  {
    c =  (char)Serial.read() ;                   // Yes, read the next input character
    //Serial.write ( c ) ;                       // Echo
    if ( ( c == '\n' ) || ( c == '\r' ) )
    {
      if ( cmdinx )
      {
        cmd[cmdinx] = '\0';                      // Delimeter
        reply = analyzeCmd ( cmd ) ;             // Analyze command and handle it
        dbgprint ( reply ) ;                     // Result for debugging
        cmdinx = 0 ;                             // Prepare for new command
      }
    }
    if ( c >= ' ' )                              // Only accept useful characters
    {
      cmd[cmdinx++] = c ;                        // Add to the command
    }
    if ( cmdinx >= ( sizeof(cmd) - 2 )  )        // Check for excessive length
    {
      cmdinx = 0 ;                               // Too long, reset
    }
  }
  if ( clickcount2 != clickcount_old )
  {
    clickcount_old = clickcount2 ;
    dbgprint ( "Click count 1/2/3 nu %d/%d/%d",
               clickcount,
               clickcount2,
               clickcount3 ) ;
  }
}


//**************************************************************************************************
//                                S C A N G P S S E R I A L                                        *
//**************************************************************************************************
// Listen to input on the GPS serial line.                                                         *
//**************************************************************************************************
void scanGPSserial()
{
  bool           validGPS = false ;              // True if GPS fixed
  char           c ;                             // Character read from serial input
  static char    gpsline[80] ;                   // Buffer for debug output
  static uint8_t inx = 0 ;                       // Index in buffer
  static int     dbgcount = 0 ;                  // Limit debug output
  static int     nsat = 0 ;                      // Aantal satelieten
  int            x ;                             // Idem, nieuw aantal
  
  while ( Serial2.available() )                  // Input on serial line?
  {
    c = Serial2.read() ;                         // Yes, get next injput character
    validGPS = gps.encode ( c ) ;                // Let TinyGPS do the parsing
    if ( c < ' ' )                               // Control character?
    {
      gpsline[inx] = '\0' ;                      // Delimit input line
      if ( ( dbgcount < 40 ) &&                  // Check debug limit
           ( strlen ( gpsline ) > 3 ) )          // Serious input?
      {
        dbgcount++ ;                             // Count number of debug lines
        dbgprint ( "GPS input is <%s>",          // Yes, show debug info
                   gpsline ) ;
      }
      inx = 0 ;                                  // For next input line
    }
    else
    {
      gpsline[inx++] = c ;                       // Store normal characters
    }
  }
  if ( validGPS )
  {
    x = gps.satellites.value() ;                 // Aantal satelieten
    if ( x != nsat )
    {
      nsat = x ;                                 // Is veranderd
      dbgprint ( "GPS satellieten: %d", nsat ) ; // Aantal satelliten in gebruik
    }
    if ( gps.location.isUpdated() )
    {
      gps_lat = gps.location.lat() ;             // Latitude ophalen
      gps_lon = gps.location.lng() ;             // Longitude ophalen
      gps_lat_s = String ( gps.location.lat(),
                           6 ) ;                 // Latitude als string
      gps_lon_s = String ( gps.location.lng(),
                           6 ) ;                 // Longitude als string
    }
  }
}


//**************************************************************************************************
//                                          T I M E R 1 0 0                                        *
//**************************************************************************************************
// Called every 100 msec on interrupt level, so must be in IRAM and no lengthy operations          *
// allowed.                                                                                        *
//**************************************************************************************************
void IRAM_ATTR timer100()
{
  static int16_t      c_100_msec = 0 ;            // Counter for 1 second, update per 100 msec
  static int16_t      eqcount = 0 ;               // Counter for equal number of clicks
  static int16_t      oldclickcount = 0 ;         // To detect difference
  static qdata_struct regfunc = { QDATA, 0 } ;    // Function to queue
  static uint8_t      oldstat = 1 ;               // Oude stand PIN_DIM
  bool                newstat ;                   // Nieuwe stand PIN_DIM
  static uint32_t     oldtime ;                   // Tijd laatste wijziging PIN_DIM
  uint32_t            newtime ;                   // Tijd huidige wijziging PIN_DIM
  
  if ( ++c_100_msec == 10 )                       // 1 seconde voorbij?
  {
    c_100_msec = 0 ;                              // Ja, teller reset
    seconds++ ;                                   // Vrijlopende teller
    secondelater = true ;                         // Ja, activeer display functie
    if ( --cregint == 0 )                         // Teller voor registratie
    {
      cregint = regint ;                          // Tijd voor registratie, reset teller
      xQueueSendFromISR ( regqueue, &regfunc,     // Registratie request naar queue
                          pdFALSE ) ;
    }
  }
  // Handle dim knop.
  newstat = digitalRead ( PIN_DIM ) ;             // Lees de input
  if ( newstat != oldstat )                       // Input gewijzigd?
  {
    newtime = millis() ;                          // Ja, pak huidig tijdstip
    if ( newstat )                                // Button losgelaten?
    {
      if ( ( newtime - oldtime ) > 3000 )         // Ja, tijdsduur meer dan 3 seconden?
      {
        longclick = true ;                        // Ja, longclick gezien
      }
      else
      {
        clickcount++ ;                            // Nee, tel aantal clicks
      }
    }
    oldtime = newtime ;                           // Werk tijdstip bij
    oldstat = newstat ;                           // Werk status bij
  }
  // Detectie van single/double/triple click op de DIM button
  if ( clickcount )                               // Any click?
  {
    if ( oldclickcount == clickcount )            // Yes, stable situation?
    {
      if ( ++eqcount == 4 )                       // Long time (0.4 sec) stable?
      {
        eqcount = 0 ;
        if ( clickcount == 2 )                    // Double click?
        {
          doubleclick = true ;                    // Yes, set result
        }
        else
        {
          singleclick = true ;                    // Just one click seen
        }
        clickcount = 0 ;                          // Reset number of clicks
      }
    }
    else
    {
      oldclickcount = clickcount ;                // To detect change
      eqcount = 0 ;                               // Not stable, reset count
    }
  }
}


//**************************************************************************************************
//                                  W I F I _ E V E N T                                            *
//**************************************************************************************************
// Print WiFi events voor test.                                                                    *
//**************************************************************************************************
void wifi_event ( WiFiEvent_t event )
{
  const char* p = "WiFi event" ;          // Default debug string

  switch(event)
  {
    case SYSTEM_EVENT_WIFI_READY:
      p = "WiFI ready" ;
      break ;
    case SYSTEM_EVENT_SCAN_DONE:
      p= "SYSTEM_EVENT_SCAN_DONE" ;
      break ;
    case SYSTEM_EVENT_STA_CONNECTED:
      p = "SYSTEM_EVENT_STA_CONNECTED" ;
      break ;
    case SYSTEM_EVENT_STA_GOT_IP:
      p = "SYSTEM_EVENT_STA_GOT_IP" ;
      break ;
    case SYSTEM_EVENT_STA_START:
      p = "SYSTEM_EVENT_STA_START" ;
      break ;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
      p = "AP_STA IP ASSIGNED" ;
      break ;
    case SYSTEM_EVENT_AP_START:
      p = "AP started." ;
        break ;
    case SYSTEM_EVENT_AP_STOP:
      p = "AP Stopped" ;
      break ;
    case SYSTEM_EVENT_AP_STACONNECTED:
      p = "WiFi Client Connected" ;
      break ;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      p = "WiFi Client Disconnected" ;
      break ;
    default:
      dbgprint ( "Unhandled WiFi Event: %d", event ) ;
      break ;
  }
  dbgprint ( p ) ;
}


#if false
//************************************************************************************
//                                        I 2 C S C A N                              *
//************************************************************************************
// Scan de I2c bus om de OLED te detecteren.                                         *
//************************************************************************************
uint8_t i2cscan()
{
  uint8_t error ;
  uint8_t address ;
  uint8_t res = 0 ;
  int  nDevices = 0 ;

  dbgprint ( "I2C scan...." ) ;
  for ( address = 1 ; address < 127 ; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission ( address ) ;
    error = Wire.endTransmission() ;
    if (error == 0)
    {
      dbgprint ( "I2C device found at address 0x%02X", address ) ;
      res = address ;                           // Result address
      nDevices++ ;
    }
    else if (error == 4)
    {
      dbgprint ( "Unknown error at address 0x%02X", address ) ;
    }
  }
  if ( nDevices == 0 )
  {
    dbgprint ( "No I2C devices found" ) ;
  }
  else
  {
    dbgprint ( "done" ) ;
  }
  return res ;                                    // Return address of display
}
#endif

//******************************************************************************
//                                S E N D B I T                                *
//******************************************************************************
// Send a bit of a duration (micros) according to the parameter, followed by   *
// a 500 micros low.                                                          *
//******************************************************************************
void sendbit ( uint16_t duration )
{
  ledcWrite ( LEDC_CHANNEL_0, 1 ) ;
  delayMicroseconds ( duration ) ;
  ledcWrite ( LEDC_CHANNEL_0, 0 ) ;;
  delayMicroseconds ( 500 ) ;
}


//******************************************************************************
//                                       S Y N C                               *
//******************************************************************************
// Send a sync. 10 times 1000 micros high, 500 micros low.                     *
//******************************************************************************
void sync(void)
{
  uint8_t i ;
  
  for ( i = 0 ; i < 10 ; i++ )
  {
    sendbit ( 1000 ) ;
  }
  chksum = 0 ;                                // Reset checksum
}


//******************************************************************************
//                             S E N D B Y T E S                               *
//******************************************************************************
// Send a number of 8 bits byte data with the RF transmitter. MSB bit first.   *
//******************************************************************************
void sendbytes ( uint8_t* buf, uint8_t n )
{
  uint8_t val ;                           // Byte to be send 
  uint8_t bp ;                            // Bit pattern for 8 data bits

  while ( n-- )                           // Then send all bytes
  {
    val = *buf++ ;                        // Byte to send
    for ( bp = 0x80 ; bp ; bp >>= 1 )     // Send 8 bits
    {
      if ( val & bp )                     // Send a "1"-bit?
      {
        sendbit ( 600 ) ;                 // Yes, duration is 600 micros
      }
      else
      {
        sendbit ( 300 ) ;                 // "0"-bit, duration is 600 micros
      }
    }
    chksum ^= val ;                       // Update checksum
    delayMicroseconds ( 500 ) ;           // Wait some time between bytes
  }
}


//************************************************************************************
//                            T E S T D R A W C I R C L E                            *
//************************************************************************************
// Teken een cirkelpatroon op de OLED voor testdoeleinden.                           *
//************************************************************************************
void testdrawcircle()
{
  uint16_t radius = display.width() / 2 ;   // Straal van de cirkel en x0
  uint32_t y0 = display.height() / 2 ;

  display.clearDisplay();
  for ( int16_t i = 0 ; i < radius ; i += 2 )
  {
    display.drawCircle ( radius, y0, i, WHITE ) ; // Teken de cirkel
    display.display() ;
    display.drawCircle ( radius, y0, i, BLACK ) ; // Gum weer uit
    display.display() ;
  }
}


//************************************************************************************
//                                        S E T U P                                  *
//************************************************************************************
// Wordt éénmalig uitgevoerd bij opstarten.                                          *
//************************************************************************************
void setup()
{
  const   char* title = "ESP32 stoflamp V0.1" ;       // Titel op Serial en display
  uint8_t       inr ;                                 // Interrupt nummer van GPIO
  const char*   partname = "nvs" ;                    // Partition with NVS info
  esp_partition_iterator_t  pi ;                      // Iterator for find

  Serial.begin ( 115200 ) ;                           // Voor debug output en commands
  Serial.println() ;
  Serial.println() ;
  Serial2.begin ( 9600 ) ;                            // Voor GPS input
  dbgsem = xSemaphoreCreateMutex() ;                 // Semaphore for exclusive use of dbgprint
  IOsem  = xSemaphoreCreateMutex() ;                 // Semaphore for SPI bus
  regqueue  = xQueueCreate ( 5,                       // Create queue for communication with regtask
                             sizeof(qdata_struct) ) ;
  wifiqueue = xQueueCreate ( 5,                       // Create queue for communication with wifitask
                             sizeof(qdata_struct) ) ;
  loraqueue = xQueueCreate ( 5,                       // Create queue for communication with loratask
                             sizeof(qdata_struct) ) ;
  dbgprint ( "%s wordt gestart", NAME ) ;             // Toon intro
  xmaintask = xTaskGetCurrentTaskHandle() ;           // Taskhandle hoofdprogramma
  if ( !SPIFFS.begin ( FORMAT_SPIFFS_IF_FAILED ) )    // Mount en test SPIFFS
  {
    dbgprint ( "SPIFFS Mount Fout!" ) ;               // Helaas...
  }
  else
  {
    dbgprint ( "SPIFFS is okay, space %d, used %d",   // Toon beschikbare ruimte
               SPIFFS.totalBytes(),
               SPIFFS.usedBytes() ) ;
  }
  pi = esp_partition_find ( ESP_PARTITION_TYPE_DATA,   // Get partition iterator for
                            ESP_PARTITION_SUBTYPE_ANY, // the NVS partition
                            partname ) ;
  if ( pi )
  {
    nvs = esp_partition_get ( pi ) ;                  // Get partition struct
    esp_partition_iterator_release ( pi ) ;           // Release the iterator
    dbgprint ( "Partition %s found, %d bytes",
               partname,
               nvs->size ) ;
  }
  else
  {
    dbgprint ( "Partition %s not found!",
               partname ) ;                           // Very unlikely...
    while ( true ) ;                                  // Impossible to continue
  }
  namespace_ID = FindNsID ( NAME ) ;                  // Find0 ID of our namespace in NVS
  if ( namespace_ID == 0 )
  {
    dbgprint ( "NVS: namespace "                      // Geen namespace gevonden
               "NAME" " niet gevonden" ) ;
  }
  fillkeylist() ;                                     // Fill nvskeys with all keys
  Wire.begin ( PIN_SDA, PIN_SCL ) ;
  pinMode ( DUST_PM10_PIN, INPUT ) ;
  inr = digitalPinToInterrupt ( DUST_PM10_PIN ) ;     // Interrupt nummer van stofsensor pin
  attachInterrupt ( inr,                              // Verbind interruptroutine aan pin
                    handleDustInterrupt, CHANGE ) ;
  ledcSetup ( LEDC_CHANNEL_0, LEDC_BASE_FREQ,         // Setup timer for IR LED
              LEDC_TIMER_1_BIT ) ;
  ledcAttachPin ( IR_LED_PIN, LEDC_CHANNEL_0 ) ;      // And attach timer to the IR LED pin
  pinMode ( PIN_DIM, INPUT_PULLUP ) ;
  inr = digitalPinToInterrupt ( PIN_DIM ) ;           // Interrupt nummer van DIM pin
  //oled_adr = i2cscan() ;                            // Scan i2cbus
  if ( !display.begin ( SSD1306_SWITCHCAPVCC,         // Start OLED met backlight
                        OLED_ADR ) )
  {
    dbgprint ( "OLED initialisatie fout!" ) ;
  }
  Wire.beginTransmission ( HTU21D_ADR ) ;             // Test I2C bus voor HTU21D sensor
  if ( Wire.endTransmission() != 0 )
  {
    dbgprint ( "HTU21D initialisatie fout!" ) ;       // HTU21D werkt niet
  }
  htu.begin() ;                                       // Initialiseer HTU21D sensor
  sttimer = timerBegin ( 0, 80, true ) ;              // Gebruik 1e timer met prescaler 80
  timerAttachInterrupt ( sttimer, &timer100, true ) ; // Call timer100() bij timer alarm
  timerAlarmWrite ( sttimer, 100000, true ) ;         // Alarm elke 100 msec
  timerAlarmEnable ( sttimer ) ;                      // Enable de timer
  testdrawcircle() ;                                  // Toon cirkelpatroon op OLED
  showstronoled ( title ) ;                           // Toon titel op OLED
  RF24L01_init ( 18, 19, 23,                          // Init SPI and RF24L01
                 NRF24_CSN_PIN, NRF24_CE_PIN ) ;
  RF24L01_setup ( NETWORK, RF_CHAN ) ;                // Init communication channel
  //RF24L01_auto_ack ( FALSE ) ;                      // No auto ack
  testing() ;                                         // Toon lichtpatroon op LEDs
  if ( intcount < 2 )                                 // Al interrupts van stofsensor?
  {
    showstronoled ( "Stofsensor fout!" ) ;            // Nee, geef waarschuwing
  }
  if ( Serial2.available() < 10 )                     // Al input van GPS?
  {
    showstronoled ( "GPS fout!" ) ;                   // Nee, geef waarschuwing
  }
  mk_lsan() ;                                         // Maak een lijst van alle bruikbare netwerken
  WiFi.onEvent ( wifi_event ) ;
  WiFi.persistent ( false ) ;                         // SSID and password niet opslaan
  listNetworks() ;                                    // Search for WiFi networks
  readprefs ( false ) ;                               // Read preferences
  tcpip_adapter_set_hostname ( TCPIP_ADAPTER_IF_STA,  // Set hostname
                               NAME ) ;
  NetworkFound = connectwifi ( true ) ;               // Connect to WiFi network
  vTaskDelay ( 3000 / portTICK_PERIOD_MS ) ;          // Geef tijd om display te lezen
  webserver.on ( "/dummy.html", handle_spec ) ;       // Behandle speciale functies
  webserver.onNotFound ( handleFileRead ) ;           // Anders gewoon een pagina/script laden
  dbgprint ( "Start webserver" ) ;
  webserver.begin() ;                                 // Start http server
  if ( NetworkFound )                                 // OTA only if Wifi network found
  {
    dbgprint ( "WiFi netwerk gevonden. Start OTA" ) ;
    ArduinoOTA.setHostname ( NAME ) ;                 // Set the hostname
    ArduinoOTA.onStart ( otastart ) ;
    ArduinoOTA.onError ( otaerror ) ;
    ArduinoOTA.onProgress ( otaprogress ) ;
    ArduinoOTA.begin() ;                              // Allow update over the air
    if ( MDNS.begin ( NAME ) )                        // Start MDNS transponder
    {
      dbgprint ( "MDNS responder started" ) ;
    }
    else
    {
      dbgprint ( "Error setting up MDNS responder!" ) ;
    }
  }
  xTaskCreate (
    regtask,                                             // Task to handle interface communication
    "Regtask",                                           // Name of task.
    2048,                                                // Stack size of task
    NULL,                                                // parameter of the task
    1,                                                   // priority of the task
    &xregtask ) ;                                        // Task handle to keep track of created task
  vTaskDelay ( 1000 / portTICK_PERIOD_MS ) ;             // 1000 ms delay
  xTaskCreate (
    loratask,                                            // Task to handle LoRa communication
    "LoRatask",                                          // Name of task.
    2048,                                                // Stack size of task
    NULL,                                                // parameter of the task
    1,                                                   // priority of the task
    &xloratask ) ;                                       // Task handle to keep track of created task
  vTaskDelay ( 1000 / portTICK_PERIOD_MS ) ;             // 1000 ms delay
  xTaskCreate (
    wifitask,                                            // Task to handle WiFi communication
    "WiFitask",                                          // Name of task.
    2048,                                                // Stack size of task
    NULL,                                                // paramelter of the task
    1,                                                   // priority of the task
    &xwifitask ) ;                                       // Task handle to keep track of created task
  starttime = millis() ;
}


//************************************************************************************
//                             O U T _ T O _ L A M P                                 *
//************************************************************************************
// Stuur bericht met RGB info naar lamp.                                             *
//************************************************************************************
void out_to_lamp()
{
  uint8_t     RF_status ;                         // Status zenden naar RF24L01
  const char* p = "ack" ;                         // Neem aan dat het goed gaat

  // Begin met de IR zender
  sync() ;                                        // Stuur sync patroon
  sendbytes ( rgbMessage, sizeof(rgbMessage) ) ;  // Verstuur
  sendbytes ( &chksum, 1 ) ;                      // Add checksum
  sync() ;                                        // Stuur bericht einde
  // Nu de NRF24L01
  claimIO ( "NRF24L01 send" ) ;                   // Claim HF send resource
  RF24L01_set_mode_TX ( REMNODE ) ;               // Prepare to send to node "REMNODE"
  RF24L01_write_payload ( rgbMessage,             // Send data
                          sizeof(rgbMessage) ) ;
  int timout = 0 ;                                // Set timer for time-out
  while ( ( RF24L01_get_status() & 0x30 ) == 0 )  // Wait for NRF24L01 ready
  {
    delay ( 10 ) ;
    if ( timout++ >= 20 )                         // Don't wait too long
    {
      p = "fail" ;                                // Fout, set status status
      break ;
    }          
  }
  RF_status = RF24L01_clear_interrupts() ;        // Read and clear status
  releaseIO() ;                                   // Release resource
  if ( RF_status & 0x10 )                         // Test op max retry
  {
    p = "no ack" ;                                // Fout, print status
  }
  dbgprint ( "Output naar de lamp, "
             "R is %d, G is %d, B is %d, RF24 %s",
             rgbMessage[0], rgbMessage[1],
             rgbMessage[2], p ) ;

}


//************************************************************************************
//                                      T E S T I N G                                *
//************************************************************************************
// Stuur testpatroon naar LEDs.                                                      *
//************************************************************************************
void testing()
{
  int16_t i, j ;                                      // Voor de loops

  showstronoled ( "Testen Lamp" ) ;
  for ( j = 255 ; j > 0 ; j -= 25  )                  // Aflopende helderheid
  {
    for ( i = 2 ; i >= 0 ; i-- )                      // 3 kleuren
    {
      memset ( rgbMessage, 0, sizeof(rgbMessage) ) ;  // Alle licht uit
      rgbMessage[i] = j ;                             // Eentje sturen
      out_to_lamp() ;                                 // Stuur naar lamp
      vTaskDelay ( 500 / portTICK_PERIOD_MS ) ;       // 500 ms delay
    }
  }
  showstronoled ( "Einde Test" ) ;
}


//************************************************************************************
//                            S H O W S T R O N O L E D                              *
//************************************************************************************
// Toon string op de OLED.                                                           *
//************************************************************************************
void showstronoled ( const char* str )
{
  static bool first = true ;

  if ( first )
  {
    first = false ;
    display.clearDisplay() ;
    display.setTextSize ( 1 ) ;
    display.setTextColor ( WHITE ) ;
    display.setCursor ( 0, 0 ) ;
  }
  dbgprint ( str ) ;
  display.println ( str ) ;
  display.display() ;
}


//************************************************************************************
//                                  S H O W O L E D                                  *
//************************************************************************************
// Toon fijnstoggehalte op de OLED.                                                  *
// Onderaan wordt de temperatuur, vochtigheid en tijd weergegeven.                   *
//************************************************************************************
void showoled()
{
  char    tijdstr[10] ;                                 // String met tijd
  uint8_t uur ;                                         // Uur gecorrigeerd voor UTC

  uur = ( gps.time.hour() + UTC_OFFSET ) % 24 ;         // Beppal het uur van de dag
  sprintf ( tijdstr, "%02d:%02d", uur,
            gps.time.minute() ) ;
  claimIO ( "OLED display") ;                           // Claim I/O resource
  display.clearDisplay() ;                              // Begin met een schone lei
  display.setCursor ( 15, 15 ) ;                        // Beetje middenin
  display.setTextSize ( 3 ) ;                           // Goed leesbaar
  display.println ( fijnstof ) ;                        // Toon fijnstof
  display.setTextSize ( 1 ) ;                           // Rest met kleine letters
  display.setCursor ( 0, 56 ) ;                         // Onderaan
  display.print ( String ( temperatuur, 1 ).c_str() ) ; // Toon temperatuur
  display.print ( "C " ) ;
  display.print ( String ( vochtigheid, 0 ).c_str() ) ; // Toon vochtigheid
  display.print ( "%" ) ;
  display.setCursor ( 128 - 30, 56 ) ;                  // Onderaan rechts
  display.println ( tijdstr ) ;
  display.setCursor ( 128 - 12, 0 ) ;                   // Bovenaan rechts
  display.print ( loraOK ) ;                            // Spatie of "L"
  display.print ( wifiOK ) ;                            // Spatie of "W"
  display.display() ;                                   // Eigenlijke output
  releaseIO() ;                                         // Release resource
}


//************************************************************************************
//                               H A N D L E _ D I M                                 *
//************************************************************************************
// Kijk of er aktie op de DIM-knop nodig is.                                         *
//************************************************************************************
void handle_dim()
{
  if ( singleclick )                                  // Click?
  {
    singleclick = false ;                             // Ja, reset flag
    dbgprint ( "Single click") ;
    analyzeCmd ( "updim" ) ;                          // Meer licht
  }
  else if ( doubleclick )                             // Dubbelclick?
  {
    doubleclick = false ;                             // Ja, reset flag
    dbgprint ( "Double click") ;
    analyzeCmd ( "dimlevel", "0" ) ;                  // Licht volledig dimmen
  }
  else if ( longclick )                               // Lang gedrukt?
  {
    longclick = false ;
    dbgprint ( "Long click") ;                        // Ja, toon debug
    resetreq = true ;                                 // Reset de ESP32
  }
}


//************************************************************************************
//                         H A N D L E _ T E M P                                     *
//************************************************************************************
// Kijk of er een nieuwe temperatuur/vochtigheid meting gedaan moet worden.          *
// Resultaat wordt in common data gezet.                                             *
//************************************************************************************
void handle_temp()
{
  static uint16_t  sec20 = 0 ;                         // Teller voor 20 seconden

  if ( sec20++ >= 400 )                                // 20 sec is ong. 400 loops
  {
    sec20 = 0 ;                                        // Reset voor volgende keer
    claimIO ( "HTO read") ;                            // Claim resource
    temperatuur = htu.readTemperature() ;              // Lees de temperatuur
    vochtigheid = htu.readHumidity() ;                 // en de vochtigheid
    releaseIO() ;                                      // Release resource
    dbgprint ( "Temperatuur is  %s, "                  // Toon voor debug
               "vochtigheid is  %s",
               String ( temperatuur, 1 ).c_str(),
               String ( vochtigheid, 1 ).c_str() ) ;
  }
}


//************************************************************************************
//                         H A N D L E _ S T O F M E T I N G                         *
//************************************************************************************
// Kijk of er aktie op de stofmeting nodig is.                                       *
// Wordt elke seconde aangeroepen.                                                   *
//************************************************************************************
void handle_stofmeting()
{
  static uint8_t   seconden = 0 ;                      // Teller voor 3 seconden
  static uint8_t   sec20 = 0 ;                         // Teller voor 20 seconden
  float            s, r, ratio ;                       // Voor floating point rekenen
  String           tmpstr ;                            // Gehalte als string

  if ( ! secondelater )                                // Seconde voorbij?
  {
    return ;                                           // Nee, niets doen
  }
  secondelater = false ;                               // Ja, reset flag
  sec20++ ;                                            // Tel aantal seconden
  if ( seconden++ < 3 )                                // 3 seconden voorbij?
  {
    return ;                                           // Nee, niets doen
  }
  s = countSET ;
  r = countRESET ;
  if ( ( s + r ) > 0.0 )                               // Voorkom delen door 0
  {
    ratio = 1000.0 * s / ( s + r ) ;                   // Bereken ug/m3
  }
  else
  {
    ratio = random ( 200, 1000 ) / 10.0 ;              // Simuleer data
  }
  fijnstof = ( 2 * fijnstof + ratio ) / 3.0 ;          // Beetje filteren
  showoled() ;                                         // Info naar OLED
  if ( sec20 >= 20 )                                   // 20 seconden voorbij?
  {
    sec20 = 0 ;                                        // Reset voor volgende keer
    tmpstr = String ( fijnstof, 2 ) ;                  // Maak een string van het gehalte
    dbgprint ( "PM10 = %s", tmpstr.c_str() ) ;
    starttime = millis() ;
    do_leds() ;                                        // Voor PWM LEDs AND IR LED lamp
  }
}


//**************************************************************************************************
//                                  H A N D L E F I L E R E A D                                    *
//**************************************************************************************************
// Transfer file van SPIFFS naar webserver client.                                                 *
//**************************************************************************************************
void handleFileRead ( AsyncWebServerRequest *request )
{
  String ct ;                                         // Content type
  String path ;                                       // Filename for SPIFFS
  
  path = request->url() ;                             // Is de gevraagde filenaam
  dbgprint ( "handleFileRead %s", path.c_str() ) ;    // Laat zien
  if ( path.endsWith ( "/" ) )                        // Default is het index.html
  {
    path = "/index.html" ;
  }
  if ( SPIFFS.exists ( path ) )                       // Bestaat de file?
  {
    ct = getContentType ( path ) ;                    // Get content type
    request->send ( SPIFFS, path, ct ) ;              // Stuur de file naar de client
  }
  else
  {
    ct = String ( "text/plain" ) ;
    request->send ( 404, "File bestaat niet", ct ) ;  // Stuur 404 status
  }
}


//**************************************************************************************************
//                                     H A N D L E _ S P E C                                       *
//**************************************************************************************************
// Handle speciale input (parameters meegegeven in het request) van de webinterface.               *
// Wordt aangeroepen als de URL begint met /dummy.html.                                            *
//**************************************************************************************************
void handle_spec ( AsyncWebServerRequest *request )
{
  int    numargs ;                                // Aantal meegegeven parameters
  int    i ;                                      // Voor de loop
  String argn ;                                   // Naam van parameter n
  String cmd ;                                    // Commando in de vorm "key=value"
  String reply = "Commando geaccepteerd" ;        // Reply voor de request

  numargs = request->params() ;                   // Haal aantal parameters
  for ( i = 0 ; i < numargs ; i++ )               // Scan de parameters
  {
    argn = request->argName ( i ) ;               // Pak de naam (key)
    cmd = argn + String ( "=" ) +                 // Stel commando samen
          request->arg ( i ) ;
    if ( argn == "version" )                      // Skip de "version" parameter
    {
      continue ;
    }
    dbgprint ( "Handle POST %s", cmd.c_str() ) ;  // Toon POST parameter
    if ( argn == "getprefs" )                     // Is het "Get preferences"?
    {
      reply = readprefs ( true ) ;                // Read and send
    }
    else if ( argn == "getdefs" )                 // Is het "Get default preferences"?
    {
      reply = String ( defprefs_txt + 1 ) ;       // Ja, stuur voorbeeld data
    }
    else if ( argn == "getnetworks" )             // Is het "Get all WiFi networks"?
    {
      reply = networks ;                          // Reply is SSIDs
    }
    else if ( argn == "saveprefs" )               // Instellingen wegschrijven?
    {
      writeprefs ( request ) ;                    // Ja, schrijf naar NVS
      break ;
    }
    else if ( argn == "getstof" )                 // Is is a "Get stof concentration"
    {
      reply = String ( fijnstof, 2 ) ;            // Get current concentration
    }
    else if ( argn == "gettemp" )                 // Is is a "Get temperature"
    {
      reply = String ( temperatuur, 2 ) ;         // Get current temperature
    }
    else if ( argn == "gethum" )                  // Is is a "Get humidity"
    {
      reply = String ( vochtigheid, 2 ) ;         // Get current humidity
    }
    else if ( argn == "update" )
    {
      vTaskSuspend ( xregtask ) ;                 // Stop all other tasks
      vTaskSuspend ( xloratask ) ;
      vTaskSuspend ( xwifitask ) ;
      if ( update_software ( UPDATEP1 ) &&        // Update webpages, scripts and sketch
           update_software ( UPDATEP2 ) &&
           update_software ( UPDATEP3 ) &&
           update_software ( UPDATEP4 ) &&
           update_software ( UPDATEP5 ) &&
           update_software ( UPDATEP6 ) &&
           update_software ( UPDATEBIN ) )
      {
        reply = String ( "Update gereed" ) ;     // Het is gelukt
      }
      else
      {
        reply = String ( "Update mislukt" ) ;    // Het is mislukt        
      }
    }
    else
    {
      reply = analyzeCmd ( cmd.c_str() ) ;       // Analyseer overige commando's
    }
  }
  //dbgprint ( "Reply op POST is %s",            // Toon reply
  //           reply.c_str() ) ;
  request->send ( 200, "text/plain", reply ) ;   // Send the reply
}


//************************************************************************************
//                                      L O O P                                      *
//************************************************************************************
// The main loop that is continually called/executed.                                *
//************************************************************************************
void loop()
{
  handle_dim() ;                                       // Controleer DIM-knop
  handle_stofmeting() ;                                // Behandel stofmeting
  handle_temp() ;                                      // Behandel temp/vocht meteing
  ArduinoOTA.handle() ;                                // Check for OTA
  scanserial() ;                                       // Handle serial input
  scanGPSserial() ;                                    // Check GPS serial input
  if ( resetreq )                                      // Reset gevraagd?
  {
    dbgprint ( "Reset request seen") ;                 // Ja, reboot
    ESP.restart() ;
  }
  vTaskDelay ( 50 / portTICK_PERIOD_MS ) ;             // 50 ms delay
}


//************************************************************************************
//                              D O _ G R E E N                                      *
//************************************************************************************
// Stuur de groene LED aan.                                                          *
//************************************************************************************
void do_green ( float M )
{
  float    V = 0.0 ;                   // PWM waarde in procenten
  int      ipwm ;

  if ( M < 33.33 )
  {
    if ( M < 25.0 )
    {
      V = 100.0 ;
    }
    else
    {
      V = ( 33.33 - M ) * 3.0 ;
    }
  }
  ipwm = V * 2.55 * dimlevel / 100.0 ; // Schalen naar 0..255
  rgbMessage[1] = ipwm ;               // Zet GREEN waarde in message
}


//************************************************************************************
//                              D O _ B L U E                                        *
//************************************************************************************
// Stuur de blauwe LED aan.                                                          *
//************************************************************************************
void do_blue ( float M )
{
  float V = 0.0 ;                       // PWM waarde in procenten
  int   ipwm ;

  if ( ( M > 16.66 ) && ( M < 58.33 ) )
  {
    if ( M < 25.0 )
    {
      V = ( M - 16.66 ) * 3.0 ;
    }
    else if ( M > 50 )
    {
      V = ( 58.33 - M ) * 3.0 ;
    }
    else
    {
      V = 100.0 ;
    }
  }
  ipwm = V *2.55 * dimlevel / 100.0 ; // Schalen naar 0..255
  rgbMessage[2] = ipwm ;              // Zet BLUE waarde in message
}


//************************************************************************************
//                              D O _ R E D                                          *
//************************************************************************************
// Stuur de rode LED aan.                                                            *
//************************************************************************************
void do_red ( float M )
{
  float V = 0.0 ;                 // PWM waarde in procenten
  int   ipwm ;

  if ( M > 41.66 )
  {
    if ( M < 50.0 )
    {
      V = ( M - 41.66 ) * 3.0 ;
    }
    else
    {
      V = 100.0 ;
    }
  }
  ipwm = V *2.55 * dimlevel / 100.0 ; // Schalen naar 0..255
  rgbMessage[0] = ipwm ;              // Zet RED waarde in message
}


//************************************************************************************
//                                   D O _ L E D S                                   *
//************************************************************************************
// Stuur de LEDs aan.                                                                *
// Wordt regelmatig aangeroepen.                                                     *
//************************************************************************************
void do_leds()
{
  do_blue ( fijnstof ) ;                         // Voor PWM LEDs AND IR LED lamp
  do_red ( fijnstof ) ;
  do_green ( fijnstof ) ;
  out_to_lamp() ;                                // Stuur naar de lamp
}


//**************************************************************************************************
//                                     G E T E N C R Y P T I O N T Y P E                           *
//**************************************************************************************************
// Read the encryption type of the network and return as a string.                                 *
//**************************************************************************************************
const char* getEncryptionType ( wifi_auth_mode_t thisType )
{
  switch ( thisType )
  {
    case WIFI_AUTH_OPEN:
      return "OPEN" ;
    case WIFI_AUTH_WEP:
      return "WEP" ;
    case WIFI_AUTH_WPA_PSK:
      return "WPA_PSK" ;
    case WIFI_AUTH_WPA2_PSK:
      return "WPA2_PSK" ;
    case WIFI_AUTH_WPA_WPA2_PSK:
      return "WPA_WPA2_PSK" ;
    case WIFI_AUTH_MAX:
      return "MAX" ;
    default:
      break ;
  }
  return "????" ;
}


//**************************************************************************************************
//                                        L I S T N E T W O R K S                                  *
//**************************************************************************************************
// List the available networks.                                                                    *
// Acceptable networks are those who have an entry in the preferences.                             *
// SSIDs of available networks will be saved for use in webinterface.                              *
//**************************************************************************************************
void listNetworks()
{
  WifiInfo_t       winfo ;            // Entry from wifilist
  wifi_auth_mode_t encryption ;       // TKIP(WPA), WEP, etc.
  const char*      acceptable ;       // Netwerk is acceptable for connection
  int              i, j ;             // Loop control

  dbgprint ( "Scan Networks" ) ;                         // Scan for nearby networks
  numSsid = WiFi.scanNetworks() ;
  dbgprint ( "Scan completed" ) ;
  if ( numSsid <= 0 )
  {
    dbgprint ( "Couldn't get a wifi connection" ) ;
    return ;
  }
  // print the list of networks seen:
  dbgprint ( "Number of available networks: %d",
             numSsid ) ;
  // Print the network number and name for each network found and
  for ( i = 0 ; i < numSsid ; i++ )
  {
    acceptable = "" ;                                    // Assume not acceptable
    for ( j = 0 ; j < wifilist.size() ; j++ )            // Search in wifilist
    {
      winfo = wifilist[j] ;                              // Get one entry
      if ( WiFi.SSID(i).indexOf ( winfo.ssid ) == 0 )    // Is this SSID acceptable?
      {
        acceptable = "Acceptable" ;
        winfo.accessable = true ;                        // Set to accessable in list
        wifilist[j] = winfo ;                            // Write back
        break ;
      }
    }
    encryption = WiFi.encryptionType ( i ) ;
    dbgprint ( "%2d - %-25s Signal: %3d dBm, Encryption %4s, %s",
               i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i),
               getEncryptionType ( encryption ),
               acceptable ) ;
    // Remember this network for later use
    networks += WiFi.SSID(i) + String ( "|" ) ;
  }
  dbgprint ( "End of list" ) ;
  // Now remove entries that has a non-available network
  for ( j = 0 ; j < wifilist.size() ; j++ )            // Search in wifilist
  {
    winfo = wifilist[j] ;                              // Get one entry
    dbgprint ( "Check wifilist %s",
               winfo.ssid ) ;
    if ( winfo.accessable == false )                   // Is this SSID accessable?
    {
      wifilist.erase ( wifilist.begin() + j ) ;        // No, remove
      dbgprint ( "SSID %s removed from list",
                 winfo.ssid ) ;
    }
  }
}


//**************************************************************************************************
//                                       C O N N E C T W I F I                                     *
//**************************************************************************************************
// Connect to WiFi using the SSID's available in wifiMulti.                                        *
// If only one AP if found in preferences (i.e. wifi_00) the connection is made without            *
// using wifiMulti.                                                                                *
// If connection fails, an AP is created and the function returns false.                           *
// If the parameter is true, there is some reporting on the OLED screen.                           *
//**************************************************************************************************
bool connectwifi ( bool oledreport )
{
  bool       localAP = false ;                          // True if only local AP is left
  WifiInfo_t winfo ;                                    // Entry from wifilist

  reconcount++ ;                                        // Count number of (re)connections
  if ( wifilist.size()  )                               // Any AP defined?
  {
    WiFi.mode ( WIFI_STA ) ;                            // ESP32 is een "station"
    WiFi.disconnect() ;                                 // Voorkom router probleem
    vTaskDelay ( 100 / portTICK_PERIOD_MS ) ;           // 100 ms delay
    if ( wifilist.size() == 1 )                         // Just one AP defined in preferences?
    {
      winfo = wifilist[0] ;                             // Get this entry
      WiFi.begin ( winfo.ssid, winfo.passphrase ) ;     // Connect to single SSID found in wifi_xx
      dbgprint ( "Try WiFi %s", winfo.ssid ) ;          // Message to show during WiFi connect
    }
    else                                                // More AP to try
    {
      wifiMulti.run() ;                                 // Connect to best network
    }
    if (  WiFi.waitForConnectResult() != WL_CONNECTED ) // Try to connect
    {
      localAP = true ;                                  // Error, setup own AP
    }
  }
  else
  {
    dbgprint ( "No known WiFi networks!" ) ;
    localAP = true ;                                    // Not even a single AP defined
  }
  if ( localAP )                                        // Must setup local AP?
  {
    dbgprint ( "WiFi Failed!  Trying to setup AP with name %s and password %s.", NAME, NAME ) ;
    WiFi.mode ( WIFI_AP ) ;                             // ESP32 wordt een access point
    vTaskDelay ( 100 / portTICK_PERIOD_MS ) ;           // 100 ms delay
    WiFi.softAP ( NAME, NAME ) ;                        // This ESP will be an AP
    vTaskDelay ( 100 / portTICK_PERIOD_MS ) ;           // 100 ms delay
    ipaddress = WiFi.softAPIP().toString() ;            // Form IP address
  }
  else
  {
    ipaddress = WiFi.localIP().toString() ;             // Form IP address
    if ( oledreport )
    {
      showstronoled ( WiFi.SSID().c_str() ) ;
    }
  }
  if ( oledreport )
  {
    showstronoled ( ipaddress.c_str( ) ) ;              // Address for STA/AP
  }
  return ( localAP == false ) ;                         // Return result of connection
}


//**************************************************************************************************
//                                           M K _ L S A N                                         *
//**************************************************************************************************
// Make al list of acceptable networks in preferences.                                             *
// Will be called only once by setup().                                                            *
// The result will be stored in wifilist.                                                          *
// Not that the last found SSID and password are kept in common data.  If only one SSID is         *
// defined, the connect is made without using wifiMulti.  In this case a connection will           *
// be made even if de SSID is hidden.                                                              *
//**************************************************************************************************
void  mk_lsan()
{
  uint8_t     i ;                                        // Loop control
  char        key[10] ;                                  // For example: "wifi_03"
  String      buf ;                                      // "SSID/password"
  String      lssid, lpw ;                               // Last read SSID and password from nvs
  int         inx ;                                      // Place of "/"
  WifiInfo_t  winfo ;                                    // Element to store in list

  dbgprint ( "Create list with acceptable WiFi networks" ) ;
  for ( i = 0 ; i < 100 ; i++ )                          // Examine wifi_00 .. wifi_99
  {
    sprintf ( key, "wifi_%02d", i ) ;                    // Form key in preferences
    if ( nvssearch ( key  ) )                            // Does it exists?
    {
      buf = nvsgetstr ( key ) ;                          // Get the contents
      inx = buf.indexOf ( "/" ) ;                        // Find separator between ssid and password
      if ( inx > 0 )                                     // Separator found?
      {
        lpw = buf.substring ( inx + 1 ) ;                // Isolate password
        lssid = buf.substring ( 0, inx ) ;               // Holds SSID now
        dbgprint ( "Added %s to list of networks",
                   lssid.c_str() ) ;
        winfo.inx = i ;                                  // Create new element for wifilist ;
        winfo.ssid = strdup ( lssid.c_str() ) ;          // Set ssid of element
        winfo.passphrase = strdup ( lpw.c_str() ) ;
        winfo.accessable = false ;
        wifilist.push_back ( winfo ) ;                   // Add to list
        wifiMulti.addAP ( winfo.ssid,                    // Add to wifi acceptable network list
                          winfo.passphrase ) ;
      }
    }
  }
  dbgprint ( "End adding networks" ) ;
}


//**************************************************************************************************
//                                           O T A S T A R T                                       *
//**************************************************************************************************
// Update via WiFi has been started by Arduino IDE or update request.                              *
//**************************************************************************************************
void otastart()
{
  dbgprint ( "OTA update Started" ) ;
}


//**************************************************************************************************
//                                   O T A P R O G R E S S                                         *
//**************************************************************************************************
// Update via WiFi has progressed.                                                                 *
//**************************************************************************************************
void otaprogress ( unsigned int progress, unsigned int total )
{
  static int step = 0 ;                            // Beperk output
  int        perc ;

  perc = progress / ( total / 100 ) ;              // Berekend het percentage gedaan
  if ( perc >= step )                              // Tijd voor een melding?
  {
    dbgprint ( "OTA progressie is %3d%%",          // Ja, laat zien
               perc ) ;
    step = step + 10 ;
  }
}


//**************************************************************************************************
//                                           O T A E R R O R                                       *
//**************************************************************************************************
// Update via WiFi has been started by Arduino IDE or update request.                              *
//**************************************************************************************************
void otaerror ( ota_error_t error )
{
  const char* p = "" ;
  
  if ( error == OTA_AUTH_ERROR ) p = "Auth Failed" ;
  else if ( error == OTA_BEGIN_ERROR ) p = "Begin Failed" ;
  else if ( error == OTA_CONNECT_ERROR ) p = "Connect Failed" ;
  else if ( error == OTA_RECEIVE_ERROR ) p = "Receive Failed" ;
  else if ( error == OTA_END_ERROR ) p = "End Failed" ;
  dbgprint ( "OTA Error[%u]: %s ", error, p ) ;
}


//**************************************************************************************************
//                                D O _ S O F T W A R E _ U P D A T E                              *
//**************************************************************************************************
// Update software vanuit OTA stream.  Resultaat wordt geflashed.                                  *
//**************************************************************************************************
bool do_software_update ( uint32_t clength )
{
  bool  res = false ;                                         // Update result
  
  if ( Update.begin ( clength ) )                             // Update possible?
  {
    dbgprint ( "Begin OTA update, length is %d",
               clength ) ;
    vTaskDelay ( 1 ) ;
    if ( Update.writeStream ( otaclient ) == clength )        // writeStream is the real download
    {
      dbgprint ( "Written %d bytes successfully", clength ) ;
    }
    else
    {
      dbgprint ( "Write failed!" ) ;
    }
    if ( Update.end() )                                       // Check for successful flash
    {
      dbgprint( "OTA done" ) ;
      if ( Update.isFinished() )
      {
        dbgprint ( "Update successfully completed" ) ;
        res = true ;                                          // Positive result
      }
      else
      {
        dbgprint ( "Update not finished!" ) ;
      }
    }
    else
    {
      dbgprint ( "Error Occurred. Error %d", Update.getError() ) ;
    }
  }
  else
  {
    // Not enough space to begin OTA
    dbgprint ( "Not enough space to begin OTA" ) ;
    otaclient.flush() ;
  }
  return res ;
}


//**************************************************************************************************
//                            S C A N _ C O N T E N T _ L E N G T H                                *
//**************************************************************************************************
// If the line contains content-length information: set clength (content length counter).          *
//**************************************************************************************************
uint32_t scan_content_length ( const char* metalinebf )
{
  static uint32_t  clength = 0 ;                        // Content lengte gevonden in http header

  if ( strstr ( metalinebf, "Content-Length" ) )        // Line contains content length
  {
    clength = atoi ( metalinebf + 15 ) ;                // Yes, set clength
    dbgprint ( "Content-Length is %d", clength ) ;      // Show for debugging purposes
  }
  return clength ;
}


//**************************************************************************************************
//                                        U P D A T E _ S O F T W A R E                            *
//**************************************************************************************************
// Update software by download from remote host.                                                   *
// Update vindt alleen plaats als er iets is gewijzigd (op grond van het tijdstip).                *
//**************************************************************************************************
bool update_software ( const char* dwlfile, const char* lstupd )
{
  uint32_t       timeout = millis() ;                           // To detect time-out
  String         line ;                                         // Input header line
  String         lstmod = "" ;                                  // Last modified timestamp in NVS
  String         newlstmod ;                                    // Last modified from host
  File           file ;                                         // Output file handle SPIFFS
  static uint8_t buf[512] ;                                     // Buufer for SPIFFS rwite
  int            len ;                                          // Number of bytes for transfer
  uint32_t       clength = 0 ;                                  // Content length uit header
  
  vTaskDelay ( 1 ) ;
  lstmod = nvsgetstr ( lstupd ) ;                               // Get current last modified timestamp
  dbgprint ( "Connecting to %s for %s%s",
             upd_server.c_str(), upd_map.c_str(), dwlfile ) ;
  if ( !otaclient.connect ( upd_server.c_str(), 80 ) )          // Connect to host
  {
    dbgprint ( "Connect to %s failed!", upd_server.c_str() ) ;
    return false ;
  }
  otaclient.printf ( "GET %s%s HTTP/1.1\r\n"
                     "Host: %s\r\n"
                     "Cache-Control: no-cache\r\n"
                     "Connection: close\r\n\r\n",
                     upd_map.c_str(), dwlfile,
                     upd_server.c_str() ) ;
  while ( otaclient.available() == 0 )                          // Wait until response appears
  {
    if ( millis() - timeout > 5000 )
    {
      dbgprint ( "Connect to update host Time-out!" ) ;
      otaclient.stop() ;
      return false ;
    }
  }
  // Connected, handle response
  while ( otaclient.available() )
  {
    line = otaclient.readStringUntil ( '\n' ) ;                 // Read a line from response
    line.trim() ;                                               // Remove garbage
    if ( !line.length() )                                       // End of headers?
    {
      break ;                                                   // Yes, get the OTA started
    }
    dbgprint ( line.c_str() ) ;                                 // Debug info
    // Check if the HTTP Response is 200.  Any other response is an error.
    if ( line.startsWith ( "HTTP/1.1" ) )
    {
      if ( line.indexOf ( " 200 " ) < 0 )                       // Scan for status 200
      {
        dbgprint ( "Got a non 200 status code from server!" ) ; // Not 200
        return false ;
      }
    }
    clength = scan_content_length ( line.c_str() ) ;            // Scan for content_length
    if ( line.startsWith ( "Last-Modified: " ) )                // Timestamp of binary file
    {
      newlstmod = line.substring ( 15 ) ;                       // Isolate timestamp
    }
  }
  // End of headers reached
  if ( newlstmod == lstmod )                                    // Need for update?
  {
    dbgprint ( "No new version available" ) ;                   // No, show reason
    while ( otaclient.available() )
    {
      otaclient.read ( buf, sizeof(buf) ) ;                     // Lees 1..n bytes
    }
    return true ;
  }
  if ( clength > 0 )
  {
    if ( strstr ( dwlfile, ".bin" ) )                           // Update of the sketch?
    {
      if ( !do_software_update ( clength ) )                    // Flash updated sketch
      {
        return false ;
      }
    }
    else
    {
      // Download a file for SPIFFS
      file = SPIFFS.open ( dwlfile, FILE_WRITE ) ;              // Open de output file
      while ( clength )
      {
        if ( otaclient.available() )
        {
          len = otaclient.read ( buf, sizeof(buf) ) ;            // Lees 1..n bytes
          file.write ( buf, len ) ;                              // Kopieer naar file
          clength -= len ;                                       // Bereken overgebleven lengte
        }
        else
        {
          vTaskDelay ( 100 / portTICK_PERIOD_MS ) ;              // 100 ms delay
          if ( millis() - timeout > 20000 )
          {
            dbgprint ( "Update host Time-out!" ) ;
            otaclient.stop() ;
            return false ;
          }
        }
      }
      file.close() ;
      dbgprint ( "%s saved in SPIFFS", dwlfile ) ;
    }
  }
  else
  {
    dbgprint ( "There was no content in the response" ) ;
    otaclient.flush() ;
    return false ;
  }
  nvssetstr ( lstupd, newlstmod ) ;                            // Update Last Modified in NVS
  return true ;
}


//**************************************************************************************************
//                                       R E A D P R E F S                                         *
//**************************************************************************************************
// Read the preferences and interpret the commands.                                                *
// If output == true, the key / value pairs are returned to the caller as a String.                *
//**************************************************************************************************
String readprefs ( bool output )
{
  uint16_t    i ;                                           // Loop control
  String      val ;                                         // Contents of preference entry
  String      cmd ;                                         // Command for analyzCmd
  String      outstr = "" ;                                 // Outputstring
  char*       key ;                                         // Point to nvskeys[i]
  uint8_t     winx ;                                        // Index in wifilist
  uint16_t    last2char = 0 ;                               // To detect paragraphs

  fillkeylist() ;                                           // Zorg voor gevulde keylist
  i = 0 ;
  while ( *( key = nvskeys[i] ) )                           // Loop trough all available keys
  {
    //dbgprint ( "readprefs key <%s>", key ) ;
    val = nvsgetstr ( key ) ;                               // Read value of this key
    cmd = String ( key ) +                                  // Yes, form command
          String ( " = " ) +
          val ;
    if ( strstr ( key, "wifi_"  ) )                         // Is it a wifi ssid/password?
    {
      winx = atoi ( key + 5 ) ;                             // Get index in wifilist
      if ( ( winx < wifilist.size() ) &&                    // Existing wifi spec in wifilist?
           ( val.indexOf ( wifilist[winx].ssid ) == 0 ) )
      {
        val = String ( wifilist[winx].ssid ) +              // Yes, hide password
              String ( "/*******" ) ;
      }
      cmd = String ( "" ) ;                                 // Do not analyze this
    }
    if ( output )
    {
      if ( ( i > 0 ) &&
           ( *(uint16_t*)key != last2char ) )               // New paragraph?
      {
        outstr += String ( "//\n" ) ;                       // Yes, add separator
      }
      last2char = *(uint16_t*)key ;                         // Save 2 chars for next compare
      outstr += String ( key ) +                            // Add to outstr
                String ( " = " ) +
                val +
                String ( "\n" ) ;                           // Add newline
    }
    else
    {
      analyzeCmd ( cmd.c_str() ) ;                          // Analyze it
    }
    i++ ;                                                   // Next key
  }
  if ( i == 0 )
  {
    outstr = String ( "Geen instellingen gevonden.\n"
                      "Gebruik de \"Default\" knop.\n" ) ;
  }
  return outstr ;
}


//**************************************************************************************************
//                                   F I N D N S I D                                               *
//**************************************************************************************************
// Find the namespace ID for the namespace passed as parameter.                                    *
//**************************************************************************************************
uint8_t FindNsID ( const char* ns )
{
  esp_err_t                 result = ESP_OK ;                 // Result of reading partition
  uint32_t                  offset = 0 ;                      // Offset in nvs partition
  uint8_t                   i ;                               // Index in Entry 0..125
  uint8_t                   bm ;                              // Bitmap for an entry
  uint8_t                   res = 0xFF ;                      // Function result

  while ( offset < nvs->size )
  {
    result = esp_partition_read ( nvs, offset,                // Read 1 page in nvs partition
                                  &nvsbuf,
                                  sizeof(nvsbuf) ) ;
    if ( result != ESP_OK )
    {
      dbgprint ( "Error reading NVS!" ) ;
      break ;
    }
    i = 0 ;
    while ( i < 126 )
    {

      bm = ( nvsbuf.Bitmap[i / 4] >> ( ( i % 4 ) * 2 ) ) ;    // Get bitmap for this entry,
      bm &= 0x03 ;                                            // 2 bits for one entry
      if ( ( bm == 2 ) &&
           ( nvsbuf.Entry[i].Ns == 0 ) &&
           ( strcmp ( ns, nvsbuf.Entry[i].Key ) == 0 ) )
      {
        res = nvsbuf.Entry[i].Data & 0xFF ;                   // Return the ID
        offset = nvs->size ;                                  // Stop outer loop as well
        break ;
      }
      else
      {
        if ( bm == 2 )
        {
          i += nvsbuf.Entry[i].Span ;                         // Next entry
        }
        else
        {
          i++ ;
        }
      }
    }
    offset += sizeof(nvs_page) ;                              // Prepare to read next page in nvs
  }
  return res ;
}


//**************************************************************************************************
//                            B U B B L E S O R T K E Y S                                          *
//**************************************************************************************************
// Bubblesort the nvskeys.                                                                         *
//**************************************************************************************************
void bubbleSortKeys ( uint16_t n )
{
  uint16_t i, j ;                                             // Indexes in nvskeys
  char     tmpstr[16] ;                                       // Temp. storage for a key

  for ( i = 0 ; i < n - 1 ; i++ )                             // Examine all keys
  {
    for ( j = 0 ; j < n - i - 1 ; j++ )                       // Compare to following keys
    {
      if ( strcmp ( nvskeys[j], nvskeys[j + 1] ) > 0 )        // Next key out of order?
      {
        strcpy ( tmpstr, nvskeys[j] ) ;                       // Save current key a while
        strcpy ( nvskeys[j], nvskeys[j + 1] ) ;               // Replace current with next key
        strcpy ( nvskeys[j + 1], tmpstr ) ;                   // Replace next with saved current
      }
    }
  }
}


//**************************************************************************************************
//                                      F I L L K E Y L I S T                                      *
//**************************************************************************************************
// File the list of all relevant keys in NVS.                                                      *
// The keys will be sorted.                                                                        *
//**************************************************************************************************
void fillkeylist()
{
  esp_err_t    result = ESP_OK ;                                // Result of reading partition
  uint32_t     offset = 0 ;                                     // Offset in nvs partition
  uint16_t     i ;                                              // Index in Entry 0..125.
  uint8_t      bm ;                                             // Bitmap for an entry
  uint16_t     nvsinx = 0 ;                                     // Index in nvskey table

  while ( offset < nvs->size )
  {
    result = esp_partition_read ( nvs, offset,                  // Read 1 page in nvs partition
                                  &nvsbuf,
                                  sizeof(nvsbuf) ) ;
    if ( result != ESP_OK )
    {
      dbgprint ( "Error reading NVS!" ) ;
      break ;
    }
    i = 0 ;
    while ( i < 126 )
    {
      bm = ( nvsbuf.Bitmap[i / 4] >> ( ( i % 4 ) * 2 ) ) ;      // Get bitmap for this entry,
      bm &= 0x03 ;                                              // 2 bits for one entry
      if ( bm == 2 )                                            // Entry is active?
      {
        if ( nvsbuf.Entry[i].Ns == namespace_ID )               // Namespace right?
        {
          //dbgprint ( "Set nvskeys[%d] = <%s%>",
          //           nvsinx, nvsbuf.Entry[i].Key ) ;
          strcpy ( nvskeys[nvsinx], nvsbuf.Entry[i].Key ) ;     // Yes, save in table
          if ( ++nvsinx == MAXKEYS )
          {
            nvsinx-- ;                                          // Prevent excessive index
          }
        }
        i += nvsbuf.Entry[i].Span ;                             // Next entry
      }
      else
      {
        i++ ;
      }
    }
    offset += sizeof(nvs_page) ;                                // Prepare to read next page in nvs
  }
  nvskeys[nvsinx][0] = '\0' ;                                   // Empty key at the end
  dbgprint ( "Read %d keys from NVS", nvsinx ) ;
  bubbleSortKeys ( nvsinx ) ;                                   // Sort the keys
}


//**************************************************************************************************
//                                      N V S S E A R C H                                          *
//**************************************************************************************************
// Check if key exists in nvs.                                                                     *
//**************************************************************************************************
bool nvssearch ( const char* key )
{
  size_t        len = NVSBUFSIZE ;                      // Length of the string

  nvsopen() ;                                           // Be sure to open nvs
  nvserr = nvs_get_str ( nvshandle, key, NULL, &len ) ; // Get length of contents
  return ( nvserr == ESP_OK ) ;                         // Return true if found
}


//**************************************************************************************************
//                                      N V S O P E N                                              *
//**************************************************************************************************
// Open Preferences with my-app namespace. Each application module, library, etc.                  *
// has to use namespace name to prevent key name collisions. We will open storage in               *
// RW-mode (second parameter has to be false).                                                     *
//**************************************************************************************************
void nvsopen()
{
  if ( ! nvshandle )                                         // Opened already?
  {
    nvserr = nvs_open ( NAME, NVS_READWRITE, &nvshandle ) ;  // No, open nvs
    if ( nvserr )
    {
      dbgprint ( "nvs_open failed!" ) ;
    }
  }
}


//**************************************************************************************************
//                                      N V S C L E A R                                            *
//**************************************************************************************************
// Clear all preferences.                                                                          *
//**************************************************************************************************
esp_err_t nvsclear()
{
  nvsopen() ;                                         // Be sure to open nvs
  return nvs_erase_all ( nvshandle ) ;                // Clear all keys
}


//**************************************************************************************************
//                                      N V S G E T S T R                                          *
//**************************************************************************************************
// Read a string from nvs.                                                                         *
//**************************************************************************************************
String nvsgetstr ( const char* key )
{
  static char   nvs_buf[NVSBUFSIZE] ;       // Buffer for contents
  size_t        len = NVSBUFSIZE ;          // Max length of the string, later real length

  nvsopen() ;                               // Be sure to open nvs
  nvs_buf[0] = '\0' ;                       // Return empty string on error
  nvserr = nvs_get_str ( nvshandle, key, nvs_buf, &len ) ;
  if ( nvserr )
  {
    //dbgprint ( "nvs_get_str failed %X for key %s, keylen is %d, len is %d!",
    //           nvserr, key, strlen ( key), len ) ;
    //dbgprint ( "Contents: %s", nvs_buf ) ;
  }
  return String ( nvs_buf ) ;
}


//**************************************************************************************************
//                                      N V S S E T S T R                                          *
//**************************************************************************************************
// Put a key/value pair in nvs.  Length is limited to allow easy read-back.                        *
// No writing if no change.                                                                        *
//**************************************************************************************************
esp_err_t nvssetstr ( const char* key, String val )
{
  String curcont ;                                         // Current contents
  bool   wflag = true  ;                                   // Assume update or new key

  //dbgprint ( "Set string for key %s: %s",
  //           key, val.c_str() ) ;
  if ( val.length() >= NVSBUFSIZE )                        // Limit length of string to store
  {
    dbgprint ( "nvssetstr length failed!" ) ;
    return ESP_ERR_NVS_NOT_ENOUGH_SPACE ;
  }
  if ( nvssearch ( key ) )                                 // Already in nvs?
  {
    curcont = nvsgetstr ( key ) ;                          // Read current value
    wflag = ( curcont != val ) ;                           // Value change?
  }
  if ( wflag )                                             // Update or new?
  {
    nvserr = nvs_set_str ( nvshandle, key, val.c_str() ) ; // Store key and value
    if ( nvserr )                                          // Check error
    {
      dbgprint ( "nvssetstr failed!" ) ;
    }
  }
  return nvserr ;
}


//**************************************************************************************************
//                                      N V S C H K E Y                                            *
//**************************************************************************************************
// Change a keyname in in nvs.                                                                     *
//**************************************************************************************************
void nvschkey ( const char* oldk, const char* newk )
{
  String curcont ;                                         // Current contents

  if ( nvssearch ( oldk ) )                                // Old key in nvs?
  {
    curcont = nvsgetstr ( oldk ) ;                         // Read current value
    nvs_erase_key ( nvshandle, oldk ) ;                    // Remove key
    nvssetstr ( newk, curcont ) ;                          // Insert new
  }
}



//**************************************************************************************************
//                                         C H O M P                                               *
//**************************************************************************************************
// Do some filtering on de inputstring:                                                            *
//  - String comment part (starting with "#").                                                     *
//  - Strip trailing CR.                                                                           *
//  - Strip leading spaces.                                                                        *
//  - Strip trailing spaces.                                                                       *
//**************************************************************************************************
void chomp ( String &str )
{
  int   inx ;                                         // Index in de input string

  if ( ( inx = str.indexOf ( "#" ) ) >= 0 )           // Comment line or partial comment?
  {
    str.remove ( inx ) ;                              // Yes, remove
  }
  str.trim() ;                                        // Remove spaces and CR
}


//**************************************************************************************************
//                                     A N A L Y Z E C M D                                         *
//**************************************************************************************************
// Handling of the various commands from remote webclient.                                         *
// par holds the parametername and val holds the value.                                            *
// "wifi_00" may appear more than once, like wifi_01, wifi_02, etc.                                *
// Examples with available parameters:                                                             *
//   wifi_00    = mySSID/mypassword         // Set WiFi SSID and password *)                       *
//   host       = example.nl                // Set host for WiFi communicatiobs.                   *
//   post       = /stoflamp/stoflamp.php    // Set post commando (without the data)                *
//   upd_server = example.nl                // Server voor update software                         *
//   upd_map    = /downloadmap              // Map op de server waar update staan                  *
//   lora       = 1 or 0                    // Enable/disable LoRa communication                   *
//   wifi       = 1 or 0                    // Enable/disable WiFi communication                   *
//   dimlevel   = 15                        // Zet het lichtniveau                                 *
//   updim                                  // Dimstand 1 stap feller                              *
//   downdim                                // Dimstand 1 stap minder fel                          *
//   test                                   // For test purposes                                   *
//   debug      = 1 or 0                    // Switch debugging on or off                          *
//   reset                                  // Restart the ESP32                                   *
//   lora_appskey = .....                   // LoRa ABP App Session Key obtained from TTN          *
//   lora_devaddr = .....                   // LoRa ABP Device Address obtained from TTN           *
//   lora_nwkskey = .....                   // LoRa ABP Network Session Key obtained from TTN      *
//   lora_deveui  = .....                   // LoRa OTAA DEVEUI                                    *
//   lora_appeui  = .....                   // LoRa OTAA APPEUI                                    *
//   lora_appkey  = .....                   // LoRa OTAA APPKEY                                    *
//   lora_otaa    = 1 or 0                  // Use LoRa OTAA (1=yes, 0=no)                         *
//   regint       = n                       // Registratie interval in seconden                    *
//  Commands marked with "*)" are sensible during initialization only                              *
//**************************************************************************************************
const char* analyzeCmd ( const char* par, const char* val )
{
  String             argument ;                       // Argument as string
  String             value ;                          // Value of an argument as a string
  int                ivalue ;                         // Value of argument as an integer
  static char        reply[180] ;                     // Reply to client, will be returned
  String             tmpstr ;                         // Temporary for value

  sprintf ( reply, "Commando %s geaccepteerd",        // Default reply
            par ) ;
  argument = String ( par ) ;                         // Get the argument
  chomp ( argument ) ;                                // Remove comment and useless spaces
  if ( argument.length() == 0 )                       // Lege commandline (comment)?
  {
    return reply ;                                    // Ignore
  }
  argument.toLowerCase() ;                            // Force to lower case
  value = String ( val ) ;                            // Get the specified value
  chomp ( value ) ;                                   // Remove comment and extra spaces
  ivalue = value.toInt() ;                            // Also as an integer
  ivalue = abs ( ivalue ) ;                           // Make positive
  if ( value.length() )
  {
    tmpstr = value ;                                  // Make local copy of value
    if ( argument.indexOf ( "passw" ) >= 0 )          // Password in value?
    {
      tmpstr = String ( "*******" ) ;                 // Yes, hide it
    }
    dbgprint ( "Command: %s with parameter %s",
               argument.c_str(), tmpstr.c_str() ) ;
  }
  else
  {
    dbgprint ( "Command: %s (without parameter)",
               argument.c_str() ) ;
  }
  if ( argument.startsWith ( "reset" ) )              // Reset request
  {
    resetreq = true ;                                 // Reset all
  }
  else if ( argument.startsWith ( "update" ) )        // Update request
  {
    //updatereq = true ;                              // Set flag, not used sofar
  }
  else if ( argument == "test" )                      // Test command
  {
    dbgprint ( "Stack maintask is %4d",
               uxTaskGetStackHighWaterMark ( xmaintask ) ) ;
    dbgprint ( "Stack regtask  is %4d, count = %5d",
               uxTaskGetStackHighWaterMark ( xregtask ),
               xregcount ) ;
    dbgprint ( "Stack loratask is %4d, count = %5d",
               uxTaskGetStackHighWaterMark ( xloratask ),
               xloracount  ) ;
    dbgprint ( "Stack wifitask is %4d, count = %5d",
               uxTaskGetStackHighWaterMark ( xwifitask ),
               xwificount  ) ;
    dbgprint ( "GPS N is %s", gps_lat_s.c_str() ) ; // Show position for test
    dbgprint ( "GPS E is %s", gps_lon_s.c_str() ) ;
    sprintf ( reply, "WiFi re-connect count is %d",
              reconcount ) ;
    //dbgprint ( reply ) ;
  }
  else if ( argument == "debug" )                     // debug on/off request?
  {
    STDEBUG = ivalue ;                                // Yes, set flag accordingly
  }
  else if ( argument == "lora" )                      // LoRa switch?
  {
    lora = ( ivalue != 0 ) ;                          // Yes, set on or off
  }
  else if ( argument.startsWith ( "lora_" ) )         // TTN stuff?
  {
    if ( argument.endsWith ( "appskey" ) )
    {
      fillttnkey ( APPSKEY, sizeof(APPSKEY),
                   value.c_str() ) ;
    }
    else if ( argument.endsWith ( "devaddr" ) )
    {
      DEVADDR = a8tohex ( value.c_str() ) ;
    }
    else if ( argument.endsWith ( "nwkskey" ) )
    {
      fillttnkey ( NWKSKEY, sizeof(NWKSKEY),
                   value.c_str() ) ;
    }
    else if ( argument.endsWith ( "otaa" ) )          // Using OTAA (or not)?
    {
      OTAA = ( ivalue != 0 ) ;                        // Set on or off
    }
    else if ( argument.endsWith ( "deveui" ) )        // OTAA deveui?
    {
      fillttnkey ( DEVEUI, sizeof(DEVEUI),
                   value.c_str() ) ;
      if ( lamp_id == 0 )                             // lamp_id already set?
      {
        lamp_id = DEVEUI[7] ;
      }
    }
    else if ( argument.endsWith ( "appeui" ) )        // OTAA appeui?
    {
      fillttnkey ( APPEUI, sizeof(APPEUI),
                   value.c_str() ) ;
    }
    else if ( argument.endsWith ( "appkey" ) )        // OTAA appkey?
    {
      fillttnkey ( APPKEY, sizeof(APPKEY),
                   value.c_str() ) ;
    }
  }
  else if ( argument == "wifi" )                      // WiFi switch?
  {
    wifi = ( ivalue != 0 ) ;                          // Yes, set on or off
  }
  else if ( argument == "lamp_id" )                   // Lamp ID?
  {
    lamp_id = ivalue ;                                // Yes, set it
  }
  else if ( argument == "regint" )                    // Registration interval?
  {
    regint = ivalue ;                                 // Yes, set installation/dev ID
  }
  else if ( argument.startsWith ( "upd_" ) )          // Upd(ate software) parameter?
  {
    if ( argument.indexOf ( "server" ) > 0 )          // Ja, server spec?
    {
      upd_server = value ;                            // Ja, set server
    }
    if ( argument.indexOf ( "map" ) > 0 )             // Map spec?
    {
      upd_map = value ;                               // Ja, set map
    }
  }
  else if ( argument == "host" )                      // Host specification?
  {
    host = value ;                                    // Yes set hostname
  }
  else if ( argument == "post" )                      // POST specification?
  {
    post = value ;                                    // Yes, set POST command
  }
  else if ( argument == "updim" )                     // Dimlevel increment?
  {
    if ( dimlevel <= 90.0 )                           // Yes, check for max
    {
      dimlevel += 10.0 ;                              // Okay, add 10 steps
    }
    else
    {
      dimlevel = 100.0 ;                              // No, set to max
    }
    do_leds() ;                                       // Adjust LEDs
    sprintf ( reply, "Dim niveau is nu %d",           // Toon dim niveau
              (int)dimlevel ) ;
  }
  else if ( argument == "downdim" )                   // Dimlevel decrement?
  {
    if ( dimlevel >= 10.0 )                           // Yes, check for min
    {
      dimlevel -= 10.0 ;                              // Okay, subtract 10 steps
    }
    else
    {
      dimlevel = 0.0 ;                                // Minimum reached
    }
    do_leds() ;                                       // Adjust LEDs
    sprintf ( reply, "Dim niveau is nu %d",           // Toon dim niveau
              (int)dimlevel ) ;
  }
  else if ( argument == "dimlevel" )                  // Dimlevel increment?
  {
    if ( ( ivalue >= 0 ) &&                           // Check limits
         (ivalue <= 100 ) )
    {
      dimlevel = ivalue ;                             // Okay, set new level
      do_leds() ;                                     // Adjust LEDs
    }
  }
  else
  {
    sprintf ( reply, "%s aangeroepen met verkeerde parameter: %s",
              NAME, argument.c_str() ) ;
  }
  return reply ;                                      // Return reply to the caller
}


//**************************************************************************************************
//                                     A N A L Y Z E C M D                                         *
//**************************************************************************************************
// Handling of the various commands from remote webclient.                                         *
// Version for handling string with: <parameter>=<value>                                           *
//**************************************************************************************************
const char* analyzeCmd ( const char* str )
{
  char*        value ;                           // Points to value after equalsign in command
  const char*  res ;                             // Result of analyzeCmd

  value = strstr ( str, "=" ) ;                  // See if command contains a "="
  if ( value )
  {
    *value = '\0' ;                              // Separate command from value
    res = analyzeCmd ( str, value + 1 ) ;        // Analyze command and handle it
    *value = '=' ;                               // Restore equal sign
  }
  else
  {
    res = analyzeCmd ( str, "0" ) ;              // No value, assume zero
  }
  return res ;
}


//**************************************************************************************************
//                                        W R I T E P R E F S                                      *
//**************************************************************************************************
// Update the preferences.  Called from the web interface.                                         *
//**************************************************************************************************
void writeprefs ( AsyncWebServerRequest *request )
{
  int        numargs ;
  int        i ;
  uint8_t    winx ;                                     // Index in wifilist
  String     key, contents ;                            // Pair for Preferences entry
  String     dstr ;                                     // Contents for debug
  
  numargs = request->args() ;                           // Haal aantal parameters
  dbgprint ( "Writeprefs call, numargs is %d",
             numargs ) ;
  if ( numargs == 0 )
  {
    return ;
  }
  timerAlarmDisable ( sttimer ) ;                       // Disable the timer
  nvsclear() ;                                          // Remove all preferences
  for ( i = 1 ; i < numargs ; i++ )                     // Scan de parameters
  {
    if ( request->argName(i) == "version" )             // Stop bij "version" parameter
    {
      break ;
    }
    key = request->argName(i) ;                         // Pak commando
    contents = request->arg(i) ;                        // En parameter
    chomp ( key ) ;                                     // Verwijder overvloedige chars
    chomp ( contents ) ;                                // Verwijder overvloedige chars
    dstr = contents ;
    if ( ( key.indexOf ( "wifi_" ) == 0 ) )             // Sensitive info?
    {
      winx = key.substring(5).toInt() ;                 // Get index in wifilist
      if ( ( winx < wifilist.size() ) &&                // Existing wifi spec in wifilist?
           ( contents.indexOf ( wifilist[winx].ssid ) == 0 ) &&
           ( contents.indexOf ( "/" "****" ) > 0 ) )    // Hidden password?
      {
        contents = String ( wifilist[winx].ssid ) +     // Retrieve ssid and password
                   String ( "/" ) +
                   String ( wifilist[winx].passphrase ) ;
        dstr = String ( wifilist[winx].ssid ) +
               String ( "/" "*******" ) ;               // Hide in debug line
      }
    }
    dbgprint ( "writeprefs setstr %s = %s",
               key.c_str(), dstr.c_str() ) ;
    nvssetstr ( key.c_str(), contents ) ;               // Save new pair
  }
  nvs_commit( nvshandle ) ;
  fillkeylist() ;                                       // Fill nvskeys with all keys
  timerAlarmEnable ( sttimer ) ;                        // Enable the timer
}


//**************************************************************************************************
//                                     G E T C O N T E N T T Y P E                                 *
//**************************************************************************************************
// Returns the contenttype of a file to send.                                                      *
//**************************************************************************************************
String getContentType ( String filename )
{
  if      ( filename.endsWith ( ".html" ) ) return "text/html" ;
  else if ( filename.endsWith ( ".png"  ) ) return "image/png" ;
  else if ( filename.endsWith ( ".gif"  ) ) return "image/gif" ;
  else if ( filename.endsWith ( ".jpg"  ) ) return "image/jpeg" ;
  else if ( filename.endsWith ( ".ico"  ) ) return "image/x-icon" ;
  else if ( filename.endsWith ( ".css"  ) ) return "text/css" ;
  else if ( filename.endsWith ( ".zip"  ) ) return "application/x-zip" ;
  else if ( filename.endsWith ( ".gz"   ) ) return "application/x-gzip" ;
  else if ( filename.endsWith ( ".mp3"  ) ) return "audio/mpeg" ;
  else if ( filename.endsWith ( ".pw"   ) ) return "" ;              // Passwords are secret
  return "text/plain" ;
}


// LoRa stuff
//***************************************************************************************************
//                                  O S _ G E T A R T E U I                                         *
//***************************************************************************************************
// Call-back routine for LORa OTAA.                                                                 *
//***************************************************************************************************
void os_getArtEui ( u1_t* buf )
{
  dbgprint ( "LoRa os_getArtEui called" ) ;
  for ( int i = 0 ; i < 8 ; i++ )
  {
    buf[i] = APPEUI[7-i] ;
  }
}

//***************************************************************************************************
//                                  O S _ G E T D E V E U I                                         *
//***************************************************************************************************
// Call-back routine for LORa OTAA.                                                                 *
//***************************************************************************************************
void os_getDevEui ( u1_t* buf ) 
{
  dbgprint ( "LoRa os_getDevEui called" ) ;
  for ( int i = 0 ; i < 8 ; i++ )
  {
    buf[i] = DEVEUI[7-i] ;
  }
}


//***************************************************************************************************
//                                O S _ G E T D E V K E Y                                           *
//***************************************************************************************************
void os_getDevKey ( u1_t* buf )
{
  dbgprint ( "LoRa os_getDevKey called" ) ;
  memcpy ( buf, APPKEY, 16 ) ;                    // Why this call returns APPKEY?
}


//**************************************************************************************************
//                                           A 8 T O H E X                                         *
//**************************************************************************************************
// Convert a string of 8 haxadecimal characters (0..9, A..F, a..f) to its                          *
// hexadecimal value.                                                                              *
//**************************************************************************************************
uint32_t a8tohex ( const char* hstr )
{
  uint8_t  i ;                                             // Loop control
  uint32_t res = 0 ;                                       // 32 bits result
  
  for ( i = 0 ; i < 8 ; i++ )                              // Handle 8 nibbles
  {
    res = ( res << 4 ) | hex ( *hstr++ ) ;                 // Shift next four bits in
  }
  return res ;
}


//***************************************************************************************************
//                                      M E M D M P                                                 *
//***************************************************************************************************
// Dump memory for debug                                                                            *
//***************************************************************************************************
void memdmp ( const char* header, uint8_t* p, uint16_t len )
{
  uint16_t    i ;                                     // Loop control
  static char hexline[ 64 ] ;                         // String with hex chars

  for ( i = 0 ; i < len ; i++ )
  {
    sprintf ( hexline + i * 3, " %02X", *p++ ) ;      // Convert one data byte
  }
  dbgprint ( "%s%s", header, hexline ) ;              // Print result
}


//******************************************************************************
//                              H E X                                          *
//******************************************************************************
// Convert 1 character (0..9, A..F, a..f) to its hexadecimal value.            *
//******************************************************************************
uint8_t hex ( char h )
{
  uint8_t res = 0x0F ;                                   // Function result
  
  if ( h >= '0' && h <= '9' )
  {
    res = h - '0' ;
  }
  else if ( h >= 'A' && h <= 'F' )
  {
    res = h - 'A' + 10 ;
  }
  else if ( h >= 'a' && h <= 'f' )
  {
    res = h - 'a' + 10 ;
  }
  return res ;
}


//**************************************************************************************************
//                                           A 2 T O H E X                                         *
//**************************************************************************************************
// Convert a string of 2 haxadecimal characters (0..9, A..F, a..f) to its                          *
// hexadecimal value.                                                                              *
//**************************************************************************************************
uint32_t a2tohex ( const char* hstr )
{
  uint8_t  i ;                                             // Loop control
  uint32_t res = 0 ;                                       // 32 bits result
  
  for ( i = 0 ; i < 2 ; i++ )                              // Handle 2 nibbles
  {
    res = ( res << 4 ) | hex ( *hstr++ ) ;                 // Shift next four bits in
  }
  return res ;
}


//**************************************************************************************************
//                                     F I L L T T N K E Y                                         *
//**************************************************************************************************
// Fill a TTN key (APPSKEY, NWKSKEY) with a value from NVS.                                        *
//**************************************************************************************************
void fillttnkey ( uint8_t* ttnkey, uint8_t klen, const char* str )
{
  uint8_t i, j ;                                  // Indexes in both arrays

  j = 0 ;                                         // Output index
  for ( i = 0 ; i < 32 ; i += 2 )                 // Scan input string
  {
    ttnkey[j] = a2tohex ( str + i ) ;             // Fill next byte
    if ( ++j == klen )                            // Enough bytes for key?
    {
      break ;                                     // Yes, stop loop
    }
  }
}




//**************************************************************************************************
// Functions for LoRa task.                                                                        *
//**************************************************************************************************

//**************************************************************************************************
//                                         O N E V E N T                                           *
//**************************************************************************************************
// Action on LMIC events.                                                                          *
// Only EV_TXCOMPLETE and the JOIN events are expected. Other events will be represented by their  *
// decimal code.                                                                                   *
// See lmic.h for the translation.                                                                 *
//**************************************************************************************************
void onEvent ( ev_t ev )
{
  if ( ev == EV_TXSTART )
  {
    dbgprint ( "EV_TXSTART" ) ;
  }
  else if ( ev == EV_JOINING )
  {
    dbgprint ( "EV_JOINING" ) ;
  }
  else if ( ev == EV_JOINED )
  {
    dbgprint ( "EV_JOINING" ) ;
    // Disable link check validation (automatically enabled
    // during join, but not supported by TTN at this time).
    LMIC_setLinkCheckMode ( 0 ) ;
//    savreq = true ;                                       // Request to save the keys/seqnr
  }
  else if ( ev == EV_TXCOMPLETE )
  {
    lorabusy = false ;
    loraOK = 'L' ;                                          // Status voor display
    //savreq = true ;                                       // Request to save LoRa data
    dbgprint ( "EV_TXCOMPLETE (includes waiting for RX windows)" ) ;
    if ( LMIC.txrxFlags & TXRX_ACK )
    {
      dbgprint ( "Received ack" ) ;
    }
    if ( LMIC.dataLen )
    {
      dbgprint ( "Received %d bytes of payload",
                 LMIC.dataLen ) ;
    }
  }
  else
  {
    dbgprint ( "Unexpected event %d", ev ) ;            // Report unexpected event
  }
}


//**************************************************************************************************
//                                         D O _ S E N D                                           *
//**************************************************************************************************
// Send a LoRa packet.                                                                             *
//**************************************************************************************************
void do_send ( osjob_t* j )
{
  BitStream packet ( lorabuf, sizeof(lorabuf) ) ;
  int32_t   gpslon, gpslat ;
  int16_t   temp, hum ;
  int16_t   pm10 ;
  int16_t   pm25 = 0 ;                              // Dummy PM2.5
  uint8_t   vcc ;
  uint8_t   retry = 0 ;                             // Retry counter

  if ( lora )
  {
    lorabusy = true ;
    if ( LMIC.opmode & OP_TXRXPEND )                // Check if there's a current TX/RX job running
    {
      dbgprint ( "OP_TXRXPEND set, "
                 "force sending anyway" ) ;         // Problem: TX did not finish!  Reset maybe?
    }
    packet.append ( LORA_FW, 8 ) ;                  // Firmware version
    gpslat = gps_lat * 32768.0 ; 
    gpslon = gps_lon * 32768.0 ;
    packet.append ( gpslat, 24 ) ;
    packet.append ( gpslon, 24 ) ;
    temp = temperatuur * 16.0 ;
    hum = vochtigheid * 16.0 ;
    packet.append ( temp, 12 ) ;
    packet.append ( hum, 12 ) ;
    vcc = 200 ;                                     // ( VCC - 2 ) * 10 Volt
    packet.append ( vcc, 8 ) ;
    pm10 = fijnstof ;
    packet.append ( pm25, 16 ) ;                  // Dummy PM2.5
    packet.append ( pm10, 16 ) ;
    claimIO ( "LoRa send") ;                      // Claim I/O (HF send resource)
    LMIC_setTxData2 ( LORA_PORT, packet.data(),   // Prepare upstream data transmission.
                      packet.byte_size(),
                      0 ) ;
    dbgprint ( "LoRa packet queued" ) ;           // Log for debug
    while ( lorabusy )                            // Wait for completion
    {
      vTaskDelay ( 100 / portTICK_PERIOD_MS ) ;   // 100 ms delay
      if ( retry++ == 100 )                       // No infinite loop
      {
        break ;
      }
    }
    releaseIO() ;                                 // Release resource
    dbgprint ( "LoRa packet send" ) ;             // Log for debug
  }
}


//**************************************************************************************************
//                                     S E T C H A N N E L S                                       *
//**************************************************************************************************
// Set channels to be used..                                                                       *
//**************************************************************************************************
void setchannels()
{
  // Check lmic_project_config.h if you get an error message ( "DR_SF12 not declared..") here!!!!
  LMIC_setupChannel ( 0, 868100000,
                      DR_RANGE_MAP ( DR_SF12, DR_SF7 ),
                      BAND_CENTI ) ;                     // g-band
  LMIC_setupChannel ( 1, 868300000,
                      DR_RANGE_MAP ( DR_SF12, DR_SF7B ),
                      BAND_CENTI ) ;                     // g-band
  LMIC_setupChannel ( 2, 868500000,
                      DR_RANGE_MAP ( DR_SF12, DR_SF7 ),
                      BAND_CENTI ) ;                     // g-band
  LMIC_setupChannel ( 3, 867100000,
                      DR_RANGE_MAP ( DR_SF12, DR_SF7 )
                      ,  BAND_CENTI ) ;                  // g-band
  LMIC_setupChannel ( 4, 867300000,
                      DR_RANGE_MAP ( DR_SF12, DR_SF7 ),
                      BAND_CENTI ) ;                     // g-band
  LMIC_setupChannel ( 5, 867500000,
                      DR_RANGE_MAP ( DR_SF12, DR_SF7 ),
                      BAND_CENTI ) ;                     // g-band
  LMIC_setupChannel ( 6, 867700000,
                      DR_RANGE_MAP ( DR_SF12, DR_SF7 ),
                      BAND_CENTI ) ;                     // g-band
  LMIC_setupChannel ( 7, 867900000,
                      DR_RANGE_MAP ( DR_SF12, DR_SF7 ),
                      BAND_CENTI ) ;                     // g-band
  LMIC_setupChannel ( 8, 868800000,
                      DR_RANGE_MAP ( DR_FSK,  DR_FSK ),
                      BAND_MILLI ) ;                     // g2-band
}


//**************************************************************************************************
//                                     L O R A T A S K                                             *
//**************************************************************************************************
// Handles communication with LoRa.                                                                *
//**************************************************************************************************
void loratask ( void * parameter )
{
  TaskHandle_t    th ;                                   // Own taskhandle
  qdata_struct    reqfunc ;                              // Request function from/to queue
   
  vTaskDelay ( 1000 / portTICK_PERIOD_MS ) ;             // Start delay
  while ( CFG_eu868 != 1 )                               // Check for European band
  {                                                      // See lmic_project_config.h
    dbgprint ( "Wrong band" ) ;                          // Wrong band, hang in here
    vTaskDelay ( 10000 / portTICK_PERIOD_MS ) ;
  }
  th = xTaskGetCurrentTaskHandle() ;                     // Get own task handle
  dbgprint ( "Starting %s running on CPU %d "            // Show activity
             "at %d MHz.  Free memory %d",
             pcTaskGetTaskName ( th ),
             xPortGetCoreID(),
             ESP.getCpuFreqMHz(),
             ESP.getFreeHeap() ) ;                       // Free RAM memory
  // If RFM95 LoRa module fails, the program will hang in the call of os_init() in the next line!
  setchannels() ;                                        // Set LoRa channels
  os_init() ;                                            // Init LMIC
  dbgprint ( "LMIC os_init completed" ) ;                // Show success (if any)
  claimIO ( "LMIC init" ) ;                              // Claim SPI bus
  LMIC_reset() ;                                         // Reset MAC state
  LMIC_setLinkCheckMode ( 1 ) ;                          // Enable link check validation
  LMIC_setAdrMode ( 1 ) ;
  LMIC_setClockError ( MAX_CLOCK_ERROR * 10 / 100 ) ;
  LMIC_setDrTxpow ( DR_SF7, 14 ) ;                       // Set data rate and transmit power
                                                         // Note: txpow seems to be ignored
  if ( OTAA )                                            // OTAA mode?
  {
    dbgprint ( "Start LoRa joining" ) ;                  // Yes, start joining
    // Sending starts joining.   LMIC_startJoining() ;
  }
  else
  {
    dbgprint ( "Set LoRa Session using known keys" ) ;
    memdmp ( "DEVADDR:", (uint8_t*)&DEVADDR, 4 ) ;
    memdmp ( "NWKSKEY:", NWKSKEY, 16 ) ;
    memdmp ( "APPSKEY:", APPSKEY, 16 ) ;
    LMIC_setSession ( 0x1, DEVADDR, NWKSKEY, APPSKEY ) ; // Set session parameters
    LMIC.dn2Dr = DR_SF9 ;                                // TTN uses SF9 for its RX2 window.
  }
  releaseIO() ;                                          // Release I/O resource
  // End of initialization.  Now start the loop.
  while ( true )
  {
    os_runloop_once() ;
    if ( xQueueReceive ( loraqueue, &reqfunc, 10 ) )     // Request in queue?
    {
      if ( lora )                                        // Lora in configuration?
      {
        do_send ( &sendjob ) ;                           // And start send job
      }
    }
//    if ( savreq )                                      // Time to save LoRa keys and seqnr?
//    {
//      savreq = false ;                                 // Yes, reset condition
//      savekeys() ;                                     // Save to EEPROM memory
//    }
    xloracount++ ;                                       // Count number of loops
  }
  //vTaskDelete ( NULL ) ;                               // Will never arrive here
}


//**************************************************************************************************
//                                     N I B 2 A                                                   *
//**************************************************************************************************
// Convert a nibble of 4 bits to a hexadecimal character.                                          *
//**************************************************************************************************
char nib2a ( uint8_t v )
{
  if ( v < 10 )
  {
    return ( '0' + v ) ;
  }
  return ( 'A' + v - 10 ) ;
}


//**************************************************************************************************
//                                     W I F I T A S K                                             *
//**************************************************************************************************
// Handles communication with WiFi.                                                                *
//**************************************************************************************************
void wifitask ( void * parameter )
{
  qdata_struct reqfunc ;                                 // Request function from queue
  String       datastr ;                                 // Data string POST parameters
  TaskHandle_t th ;                                      // Own taskhandle
  ip_addr_t    dnsserver ;                               // IP of DNS server
  IPAddress    remote_addr = { 0,0,0,0} ;                // Address to connect to
  String       str ;                                     // String holding command to host
  String       regreply ;                                // Result of POST
  WiFiClient   client ;                                  // Client connection to host
  uint8_t      i ;                                       // Loop control data string
  uint8_t      retry ;                                   // Retry count for connection
  bool         req = false ;                             // Request pending

  th = xTaskGetCurrentTaskHandle() ;                     // Get own task handle
  dbgprint ( "Starting %s running on CPU %d "            // Show activity
             "at %d MHz.  Free memory %d",
             pcTaskGetTaskName ( th ),
             xPortGetCoreID(),
             ESP.getCpuFreqMHz(),
             ESP.getFreeHeap() ) ;                      // Free RAM memory
  vTaskDelay ( 10000 / portTICK_PERIOD_MS ) ;           // Pause for a short time
  // End of initialization.  Now start the loop.
  while ( true )
  {
    xwificount++ ;                                       // Count number of loops
    if ( remote_addr[0] == 0 )
    {
      dnsserver = dns_getserver ( 0 ) ;                  // Get DNS server of local network
      WiFi.hostByName ( host.c_str(), remote_addr ) ;    // Get IP-adres of external server
      vTaskDelay ( 5000 / portTICK_PERIOD_MS ) ;         // Pause for a short time
      dbgprint ( "IP address of %s is %d.%d.%d.%d",      // Report host IP
                 host.c_str(),
                 remote_addr[0], remote_addr[1],
                 remote_addr[2], remote_addr[3] ) ;
      vTaskDelay ( 5000 / portTICK_PERIOD_MS ) ;         // Pause for a short time
    }
    if ( xQueueReceive ( wifiqueue, &reqfunc, 500 ) )    // Request in queue?
    {
      if ( ! wifi )                                      // WiFi output configured?
      {
        continue ;                                       // No, do nothing
      }
      req = true ;                                       // Send requested
    }
    if ( req )                                           // Pending request?
    {
      claimIO ( "WiFi http" ) ;                         // Claim HF send resource
      for ( retry = 0 ; retry < 3 ; retry++ )
      {
        dbgprint ( "Connect to %s at %d.%d.%d.%d",
                  host.c_str(),
                  remote_addr[0], remote_addr[1],
                  remote_addr[2], remote_addr[3] ) ;
        if ( client.connect ( remote_addr, 80 ) )        // Connect to host
        {
          break ;                                        // Client connection okay
        }
        dbgprint ( "Client connection to %s failed",     // No connection, report
                  host.c_str() ) ;
        connectwifi ( false ) ;                          // Restart WiFi
      }
      datastr = String ( "" ) ;                          // Maak datastring leeg
      for ( i = 0 ; i < DATSIZ ; i++ )                   // Convert the binary data
      {
        datastr += String ( "&X" ) ;                     // Vorm &Xn=xxxx
        datastr += String ( i ) ;
        datastr += String ( "=" ) ;
        datastr += String ( regdata[i] ) ;
      }
      // Connected to HOST
      str =  "POST " ;
      str += post ;                                      // Post like "/page/index.php"
      str += ( String ( "?key=stoflamp" ) + datastr  ) ; // Set data
      dbgprint ( "Host request is %s", str.c_str() ) ;
      str += " HTTP/1.1\r\n" ;
      str += "Host: " ;
      str += host ;
      str += "\r\n" ; 
      str += "Connection: close\r\n\r\n" ;
      client.print ( str ) ;                             // Send request to the external server
      vTaskDelay ( 1000 / portTICK_PERIOD_MS ) ;         // Pause for a short time
      wifiOK = ' ' ;                                     // Status voor display
      // Lees reply van de server
      while ( client.available() )
      {
        regreply = client.readStringUntil ( '\r' ) ;
        // dbgprint ( "Reply from host is %s",
        //           regreply.c_str() ) ;
        wifiOK = 'W' ;                                   // Status voor display
        vTaskDelay ( 100 / portTICK_PERIOD_MS ) ;        // Pause for a short time
      }
      if ( wifiOK == 'W' )                               // Debug resultaat
      {
        dbgprint ( "WiFi registratie is gelukt" ) ;
        req = false ;                                    // Send completed
      }
      else
      {
        dbgprint ( "WiFi registratie mislukt!" ) ;
      }
      client.stop() ;
      releaseIO() ;                                      // Release HF send resource
    }
    if  ( WiFi.status() != WL_CONNECTED )
    {
      dbgprint ( "Reconnecting to WiFi..." ) ;
      connectwifi ( false ) ;
    }
  }
  //vTaskDelete ( NULL ) ;                               // Will never arrive here
}


//**************************************************************************************************
//                                     R E G T A S K                                               *
//**************************************************************************************************
// This task is called (time based) to register the measurements.                                  *
// It will prepare the data to be send by LoRa and/or Wifi en signals the 2 tasks to start the     *
// transfer.                                                                                       *
//**************************************************************************************************
void regtask ( void * parameter )
{
  TaskHandle_t    th ;                                   // Own taskhandle
  qdata_struct    reqfunc ;                              // Request function from/to queue
  

  vTaskDelay ( 1000 / portTICK_PERIOD_MS ) ;             // Start delay
  th = xTaskGetCurrentTaskHandle() ;                     // Get own task handle
  dbgprint ( "Starting %s running on CPU %d "            // Show activity
             "at %d MHz.  Free memory %d",
             pcTaskGetTaskName ( th ),
             xPortGetCoreID(),
             ESP.getCpuFreqMHz(),
             ESP.getFreeHeap() ) ;                       // Free RAM memory
  // End of initialization.  Now start the loop.
  while ( true )
  {
    if ( xQueueReceive ( regqueue, &reqfunc, 500 ) )     // Request in queue?
    {
      dbgprint ( "Registratie request van timer" ) ;     // Yes, show debug info
      regdata[0] = lamp_id ;                             // Prepareer de payload
      regdata[1] = temperatuur * 100.0 ;
      regdata[2] = vochtigheid * 100.0 ;
      regdata[3] = fijnstof * 100.0 ;
      regdata[4] = gps_lat * 1000000.0 ;
      regdata[5] = gps_lon * 1000000.0 ;
      xQueueSend ( wifiqueue, &reqfunc, 200 ) ;          // Send copy of request to WiFi queue
      xQueueSend ( loraqueue, &reqfunc, 200 ) ;          // Send copy of request to LoRa queue
    }
    xregcount++ ;                                        // Count number of loops
  }
  //vTaskDelete ( NULL ) ;                               // Will never arrive here
}
