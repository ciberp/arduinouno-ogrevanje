#include <EtherCard.h>
#include <SPI.h> //http://arduino.cc/en/Reference/SPI
#include <Thermocouple.h> //http://github.com/JChristensen/Thermocouple
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <EEPROM.h>

struct DHTData {
  float DnevnaTemp;
  float DnevnaHum;
  float ZunajTemp;
  float ZunajHum;
  float KurilnicaTemp;
  float KurilnicaHum;
};
DHTData DHTData;
#define DHTDnevnaPin A0
#define DHTZunajPin A1
#define DHTKurilnicaPin A2
#define DHTTYPE DHT22

// Data wire is plugged into pin 9 on the Arduino
#define ONE_WIRE_BUS 9

// test DS18D20 init
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
// Assign the addresses of your 1-Wire temp sensors.
// I   = 0x28, 0xFF, 0x1F, 0x2C, 0x69, 0x14, 0x04, 0xA4 = PecSpredaj
// II  = 0x28, 0xFF, 0xB1, 0x46, 0x64, 0x14, 0x03, 0x2D = PecZadaj
// III = 0x28, 0xFF, 0x68, 0x21, 0x64, 0x14, 0x03, 0x96 = Bojler
// IV  = 0x28, 0xFF, 0xF7, 0x0B, 0x64, 0x14, 0x03, 0x25 = VodaIzPeci
// V   = 0x28, 0xFF, 0x15, 0x0E, 0x64, 0x14, 0x03, 0xFD = VodaVPec
// SOLARNO SPODAJ = 0x28, 0x77, 0x1E, 0xCC, 0x06, 0x00, 0x00, 0x27
// SOLARNO ZGORAJ = 0x28, 0xC4, 0xED, 0xCA, 0x06, 0x00, 0x00, 0x98
// BOJLER2 = 0x28, 0x4B, 0xA2, 0xC5, 0x06, 0x00, 0x00, 0x4A

// Assign the addresses of your 1-Wire temp sensors.
DeviceAddress PecSpredaj = { 0x28, 0xFF, 0x1F, 0x2C, 0x69, 0x14, 0x04, 0xA4 };
DeviceAddress PecZadaj = { 0x28, 0xFF, 0xB1, 0x46, 0x64, 0x14, 0x03, 0x2D };
DeviceAddress Bojler = { 0x28, 0xFF, 0x68, 0x21, 0x64, 0x14, 0x03, 0x96 };
DeviceAddress VodaIzPeci = { 0x28, 0xFF, 0xF7, 0x0B, 0x64, 0x14, 0x03, 0x25 };
DeviceAddress VodaVPec = { 0x28, 0xFF, 0x15, 0x0E, 0x64, 0x14, 0x03, 0xFD };
DeviceAddress SolarSP = { 0x28, 0x77, 0x1E, 0xCC, 0x06, 0x00, 0x00, 0x27 };
DeviceAddress SolarZG = { 0x28, 0xC4, 0xED, 0xCA, 0x06, 0x00, 0x00, 0x98 };
DeviceAddress Bojler2 = { 0x28, 0x4B, 0xA2, 0xC5, 0x06, 0x00, 0x00, 0x4A };

int LowestFreeRAM = 2000;
//int ErrorCode = 0; // Error reporting
// DS error 0xx
// 85 = DS18D20 Error napacno povezan senzor
// DHT errors 1xx
// Thermocouple error 2xx

// POZOR NASTAVITVE ZA PRODUKCIJO
static byte mymac[] = { 0x74,0x69,0x69,0x2D,0x30,0x31 };
static byte myip[] = { 172,22,0,177 };
static byte gwip[] = { 172,22,0,250 };  // gateway address
static byte netmask[] = {255,255,255,0 };  //subnet mask
static byte dnsip[] = {172,22,0,240};  //dns

// POZOR NASTAVITVE ZA TEST
//static byte mymac[] = { 0x74, 0x69, 0x69, 0x2D, 0x30, 0x87 };
//static byte myip[] = { 172, 22, 0, 190 };
//static byte gwip[] = { 172, 22, 0, 250 }; // gateway address
//static byte netmask[] = {255, 255, 255, 0 }; //subnet mask
//static byte dnsip[] = {172, 22, 0, 240}; //dns

//int SrcPort = 10300;
//int DstPort = 514;

byte Ethernet::buffer[400];
//byte Ethernet::buffer[1000];
BufferFiller bfill;

//MAX6675 SPI pin definitions
#define csTC1  10               //chip select for MAX6675
float DimTuljavaTemp = 0;

// moj timer namesto Delay
unsigned long previousMillis = 0;
unsigned long SolarPreviousTime = 0;
//unsigned long VrataPreviousTime = 0;
//unsigned long VrataTimerPreviousTime = 0;
unsigned long LEDPreviousTime = 0;
int LEDBlinkSpeed = 300;
// Data wire is plugged into pin 9 on the Arduino #define ONE_WIRE_BUS 9

// PCF8574 Current relay states
// v pdf-ju za PCF8574 piše
// če se nahaja na 7-bitnem naslovu 20HEX (32DEC)
// potem je naslov za pisanje Write: 40HEX in Read: 41HEX
// tega wire.h ne omogoča, zato samo pišem na naslov 20HEX
// trenutno stanje pa držim v PCF8574CurrentState!!!
byte PCF8574CurrentState;

float PecTemp;
float BojlerTemp;
float VodaIzPeciTemp;
float VodaVPecTemp;
float SolarZgorajTemp;
float SolarSpodajTemp;


// Default settings (POZOR se prepiše z nastavitvami iz EEPROMA!!!)
int BojlerDiffForHeating = 8; //Razlika kdaj ogrevamo Bojler
int PecStartTemp = 60;        //Temp ko pec lahko ogreva ostalo
int PecStopTemp = 40;         //ugasnemo ogrevanje
int PecMaxTemp = 95;          //Pec Max temp
int BojlerMaxTemp = 55;       //Bojler Max temp
//int BojlerMinTemp = 42;       //Min temp bojlerja
int DnevnaMinTemp = 19;       //Min temp v dnevni
int DimTuljavaPoint = 60;     //Meja kdaj se Pec ohlaja/ogreva
int SolarMaxTemp = 80;
int Histereza = 2;            // Histereza
int DefaultsLoaded = 0;        // automatsko naloži defaults ce niso bile se nikdar 

boolean AutoControl = true;    //false=Manual, true=Auto
boolean SolarLock = true;
boolean SolarSistemWorking;
boolean RelayOgrevanje0 = false;
boolean RelayBojler1 = false;
boolean RelaySolar2 = false;
boolean RelayEmergency3 = false;

// Vklop izklop solarnega sistema, pisem v EEPROM
// EEPROM write: EEPROM.write(addr, val);
// EEPROM read:  value = EEPROM.read(address);
int EADD_SolarSistemWorking = 0; // prvi byte v eeprom-u
int EADD_BojlerDiffForHeating = 1;
int EADD_PecStartTemp = 2;
int EADD_PecStopTemp = 3;
int EADD_PecMaxTemp = 4;
int EADD_BojlerMaxTemp = 5;
//int EADD_BojlerMinTemp = 6;
int EADD_DnevnaMinTemp = 7;
int EADD_DimTuljavaPoint = 8;
int EADD_SolarMaxTemp = 9;
int EADD_Histereza = 10;
int EADD_DefaultsLoaded = 11;
//int EADD_VrataTimeToWaitBeforeClose = 12;
// --------- ERROR CODES --------
byte ErrorCodes = 0;
// BIT0 = error code za senzor DHT_Dnevna
// BIT1 = error code za senzor DHT_Zunaj
// BIT2 = error code za senzor DHT_Kurilnica
// BIT3 = error code za senzor MAX_Termocouple
// BIT4 = error code za senzor I2C error
// --------- ERROR CODES --------

int ledPin = 5; 

boolean WebNastavitveLock;
const char pageHEADER[] PROGMEM =
  "<!DOCTYPE html>\r\n"
  "<HTML>\r\n<HEAD><meta charset=\"UTF-8\">\r\n"
  ;
//const char green[] PROGMEM = "<font color=\"green\">";
//const char red[] PROGMEM = "<font color=\"red\">";
//const char blue[] PROGMEM = "<font color=\"blue\">";

const char ButtonHeaderSets[] PROGMEM = 
"<div><style scoped>.g{width:100%;height:300px}h1{font-size:400%;color:blue;}</style>"
;
const char pageVTON[] PROGMEM =
  "<a href=\"/?z1\"><button class=\"g\"><h1>VKLOP VTIČNICE</h1></button></a></div>"
  ;
const char pageVTOFF[] PROGMEM =
  "<a href=\"/?z0\"><button class=\"g\"><h1>IZKLOP VTIČNICE</h1></button></a></div>"
  ;
const char pageMANUALC[] PROGMEM =
  "<a href=\"/?m0\"><button class=\"g\"><h1>OGREVANJE IZKLOP AVTOMATIKE</h1></button></div>"
  ;
const char pageAUTOC[] PROGMEM =
  "<a href=\"/?m1\"><button class=\"g\"><h1>OGREVANJE VKLOP AVTOMATIKE</h1></button></div>"
  ;
const char pagePUMPHEATON[] PROGMEM =
  "<a href=\"/?a1\"><button class=\"g\"><h1>VKLOP ČRP. OGREVANJA</h1></button></div>"
  ;
const char pagePUMPHEATOFF[] PROGMEM =
  "<a href=\"/?a0\"><button class=\"g\"><h1>IZKLOP ČRP. OGREVANJA</h1></button></div>"
  ;
const char pagePUMPBOJLERON[] PROGMEM =
  "<a href=\"/?b1\"><button class=\"g\"><h1>VKLOP ČRP. ZA BOJLER</h1></button></div>"
  ;
const char pagePUMPBOJLEROFF[] PROGMEM =
  "<a href=\"/?b0\"><button class=\"g\"><h1>IZKLOP ČRP. ZA BOJLER</h1></button></div>"
  ;
const char pagePUMPSOLARON[] PROGMEM =
  "<a href=\"/?c1\"><button class=\"g\"><h1>VKLOP SOLARNE ČRP.</h1></button></div>"
  ;
const char pagePUMPSOLAROFF[] PROGMEM =
  "<a href=\"/?c0\"><button class=\"g\"><h1>IZKLOP SOLARNE ČRP.</h1></button></div>"
  ;
const char pageSOLARSISTEMON[] PROGMEM =
  "<a href=\"/?s1\"><button class=\"g\"><h1>VKLOP SOLARNEGA SISTEMA</h1></button></div>"
  ;
const char pageSOLARSISTEMOFF[] PROGMEM =
  "<a href=\"/?s0\"><button class=\"g\"><h1>IZKLOP SOLARNEGA SISTEMA</h1></button></div>"
  ;
const char pagePRIKAZPODATKOV[] PROGMEM =
  "<a href=\"/?sd\"><button class=\"g\"><h1>PRIKAZ PODATKOV</h1></button></div>"
  ;
const char pageNASTAVITVE[] PROGMEM =
"<a href=\"/?ns\"><button class=\"g\"><h1>NASTAVITVE</h1></button></div>"
;
const char pageBACK[] PROGMEM =
  "<meta http-equiv=\"refresh\" content=\"0;url=/\">"
  ;
const char pageBACKTONASTAVITVE[] PROGMEM =
  "<meta http-equiv=\"refresh\" content=\"0;url=/?ns\">"
;
const char pageSHRANI[] PROGMEM =
"<a href=\"/?sv\"><button class=\"g\"><h1>SHRANI</h1></button></div>"
;
const char pageDEFAULT[] PROGMEM =
"<a href=\"/?df\"><button class=\"g\"><h1>PRIVZETO</h1></button></div>"
;
const char pageEND[] PROGMEM = "</BODY></HTML>\r\n";
const char pageREFRESH[] PROGMEM =
  "<style>table,td,th{font-size:160%;text-align:center;}</style>"
  "<script>setInterval(function(){location.reload();},5000);</script>"
  "<table style=\"width:100%\"><tr><th>DNEVNA</th><th>ZUNAJ</th><th>KURILNICA</th></tr>"
;

const char tdtd[] PROGMEM = "</td><td>";
const char tdtr[] PROGMEM = "</td></tr>";
const char trtd[] PROGMEM = "<tr><td>";
const char deg[] PROGMEM = "°C";
const char per[] PROGMEM = "%";
const char trtdspacetdtr[] PROGMEM = "<tr><td>&nbsp;</td></tr>";
const char tableend[] PROGMEM = "</table>";

const char pageREFRESHMID[] PROGMEM = "<tr><th>BOJLER</th><th>PEČ</th><th>TULJAVA</th></tr>";  
const char pageREFRESHMID1[] PROGMEM = "<tr><th>OGREVANJE</th><th>ČRP. BOJLER</th><th>TEMP. ZAŠČITA</th></tr>";
const char pageREFRESHMID2[] PROGMEM = "<tr><th>ČRP. SOLAR</th><th>SOLAR ZG.</th><th>SOLAR SP.</th></tr>";
const char pageREFRESHMID3[] PROGMEM = "<tr><th>PEČ VODA OUT</th><th>PEČ VODA IN</th><th>UPTIME</th></tr>";

const char pageNAZAJ[] PROGMEM = "<a href=\"/\"><button class=\"g\"><h1>NAZAJ</h1></button></div>";

const char on[] PROGMEM = "ON";
const char off[] PROGMEM = "OFF";

const char showdata[]  = "?sd";
const char settings[]  = "?ns";
const char setvton[]  = "?z1";
const char setvtoff[] = "?z0";
const char setAutoControlon[]  = "?m1";
const char setAutoControloff[]  = "?m0";
const char setheaton[]  = "?d0";
const char setheatoff[]  = "?d1";
const char setHeatingPumpon[]  = "?a1";
const char setHeatingPumpoff[]  = "?a0";
const char setBojlerPumpon[]  = "?b1";
const char setBojlerPumpoff[]  = "?b0";
const char setSolarPumpon[]  = "?c1";
const char setSolarPumpoff[]  = "?c0";
const char setSolarSistemon[]  = "?s1";
const char setSolarSistemoff[]  = "?s0";
const char getsettings[]  = "?se";
const char getdata[]  = "?da";
const char get[]  = "GET /";
const char shrani[]  = "?sv";
const char load_default[]  = "?df";

const char Gumb_PM_Tabela[] PROGMEM =
"<style>table,td{font-size:160%;text-align:center;}</style>"
"<table align=\"center\">"
;
const char GUMB_PM_1_3[] PROGMEM = "<a href=\"";
const char GUMB_PM_2_3[] PROGMEM = "\"><button class=\"g\" style=\"width:100px;height:100%\"><h1>";
const char GUMB_PM_3_3[] PROGMEM = "</h1></button></a>";
const char GUMB_PM_PL[] PROGMEM = "+";
const char GUMB_PM_MI[] PROGMEM = "-";

const char u1m[] PROGMEM = "?u1m";
const char u1p[] PROGMEM = "?u1p";
const char u1i[] PROGMEM = "PEČ START:";

const char u2m[] PROGMEM = "?u2m";
const char u2p[] PROGMEM = "?u2p";
const char u2i[] PROGMEM = "PEČ STOP:";

const char u3m[] PROGMEM = "?u3m";
const char u3p[] PROGMEM = "?u3p";
const char u3i[] PROGMEM = "PEČ MAX:";

const char u4m[] PROGMEM = "?u4m";
const char u4p[] PROGMEM = "?u4p";
const char u4i[] PROGMEM = "BOJLER Δ:";

const char u5m[] PROGMEM = "?u5m";
const char u5p[] PROGMEM = "?u5p";
const char u5i[] PROGMEM = "BOJLER MAX:";

//const char u6m[] PROGMEM = "?u6m";
//const char u6p[] PROGMEM = "?u6p";
//const char u6i[] PROGMEM = "BOJLER MIN:";

const char u7m[] PROGMEM = "?u7m";
const char u7p[] PROGMEM = "?u7p";
const char u7i[] PROGMEM = "DNEVNA MAX:"; // v kodi uporabljam spremenljivoko DnevnaMin!

const char u8m[] PROGMEM = "?u8m";
const char u8p[] PROGMEM = "?u8p";
const char u8i[] PROGMEM = "TULJAVA MIN:";

const char u9m[] PROGMEM = "?u9m";
const char u9p[] PROGMEM = "?u9p";
const char u9i[] PROGMEM = "SOLAR MAX:";

const char hm[] PROGMEM = "?hm";
const char hp[] PROGMEM = "?hp";
const char hi[] PROGMEM = "HISTEREZA:";

const char cdays[] PROGMEM = "d";
const char chours[] PROGMEM = "h";
const char cmins[] PROGMEM = "m";
const char csecs[] PROGMEM = "s";

//char buffer[30];

void setup () {
  Serial.begin(115200);
  wdt_disable();
  //Serial.println(F("wdt disabled!"));
  delay(2L * 1000L);
  Serial.println(F("Starting..."));
  pinMode(ledPin, OUTPUT);
  //Serial.print((freeRam()));
  //Serial.println(F(" Free RAM (Starting setup ...) "));
  //Serial.print((freeRam()));
  //Serial.println(F(" Free RAM (Enabled WDTO_8S) "));
  //Serial.print(F("Init Network: "));
  //Serial.println(ether.myip[3]);
  InitNetwork();
  wdt_enable(WDTO_8S);
  //DS20d18 init void setup
  sensors.begin();
  // set the resolution to 10 bit (good enough?)
  sensors.setResolution(PecSpredaj, 10);
  sensors.setResolution(PecZadaj, 10);
  sensors.setResolution(Bojler, 10);
  sensors.setResolution(VodaIzPeci, 10);
  sensors.setResolution(VodaVPec, 10);
  sensors.setResolution(SolarSP, 10);
  sensors.setResolution(SolarZG, 10);
  sensors.setResolution(Bojler2, 10);
  // preberem nastavitve EEPROM-a
  SolarSistemWorking = EEPROM.read(EADD_SolarSistemWorking);
  BojlerDiffForHeating = EEPROM.read(EADD_BojlerDiffForHeating);
  PecStartTemp = EEPROM.read(EADD_PecStartTemp);
  PecStopTemp = EEPROM.read(EADD_PecStopTemp);
  PecMaxTemp = EEPROM.read(EADD_PecMaxTemp);
  BojlerMaxTemp = EEPROM.read(EADD_BojlerMaxTemp);
//  BojlerMinTemp = EEPROM.read(EADD_BojlerMinTemp);
  DnevnaMinTemp = EEPROM.read(EADD_DnevnaMinTemp);
  DimTuljavaPoint = EEPROM.read(EADD_DimTuljavaPoint);
  SolarMaxTemp = EEPROM.read(EADD_SolarMaxTemp);
  Histereza = EEPROM.read(EADD_Histereza);
  DefaultsLoaded = EEPROM.read(EADD_DefaultsLoaded);
  if (DefaultsLoaded != 16) {
    LoadDefaults();
    Serial.println(F("Dfl loaded!"));
  }
  // ob zagonu ugasnem vse releje
  Wire.begin();
  PCF8574CurrentState = 255;
  Wire.beginTransmission(32);
  Wire.write(PCF8574CurrentState);
  Wire.endTransmission();
  byte error;
  error = Wire.endTransmission();
  if (error != 0) {
    Serial.print(F("I2Cerr:"));
    Serial.println(error);
  }
  freeRam();
}

void loop () {
  freeRam();
  // tole se izvaja na 5sek
  //if (millis() - previousMillis > 5000) {
  if (millis() - previousMillis > 5000) {
    previousMillis = millis();
    //Serial.println("");
    //Serial.print(millis()/1000);
    //Serial.println(F("sec uptime"));
    GetTermocoupleTemp();
    GetDHTSensorData();
    sensors.requestTemperatures();
    if (sensors.getTempC(PecSpredaj) != 85) {
      SolarZgorajTemp = sensors.getTempC(PecSpredaj);
    }
    // ker občasno oba javita 85 WTF!!!
    if (sensors.getTempC(SolarZG) != 85) {
      SolarZgorajTemp = sensors.getTempC(SolarZG);
    }
    else {
      //Serial.println(F("SolarZgoraj ERROR read +85"));
      //ErrorCode = 85;
    }
    // ker občasno oba javita 85 WTF!!!
    if (sensors.getTempC(SolarSP) != 85) {
      SolarSpodajTemp = sensors.getTempC(SolarSP);
    }
    else {
      //Serial.println(F("SolarSpodaj ERROR read +85"));
      //ErrorCode = 85;
    }
    //izberem senzor, ki kaže največjo temp (ce slucajno kaksen crkne)
    PecTemp = sensors.getTempC(PecSpredaj);
    if (sensors.getTempC(PecZadaj) > PecTemp) {
      PecTemp = sensors.getTempC(PecZadaj);
    }
    BojlerTemp = sensors.getTempC(Bojler);
    if (sensors.getTempC(Bojler2) > BojlerTemp) {
      BojlerTemp = sensors.getTempC(Bojler2);
    }
    VodaIzPeciTemp = sensors.getTempC(VodaIzPeci);
    VodaVPecTemp = sensors.getTempC(VodaVPec);

    //Serial.println(F("======================================"));
    ether.printIp("IP:\t", ether.myip);
    //ether.printIp("MASK:\t", ether.netmask);
    //ether.printIp("GW:\t", ether.gwip);
    //ether.printIp("DNS:\t", ether.dnsip);
    if (ether.myip[3] == 0) {
      // ce mrezna nima IP naslova (0.0.0.0) potem ponovno init
      InitNetwork();
    }
    Serial.print(F("PecTemp: "));
    Serial.println(PecTemp);
    Serial.print(F("Tuljava: "));
    Serial.println(DimTuljavaTemp);
    Serial.print(F("Dnevna: "));
    Serial.println(DHTData.DnevnaTemp);
//    Serial.print(F("ZunajTemp: "));
//    Serial.println(DHTData.ZunajTemp);
//    Serial.print(F("KurilnicaTemp: "));
//    Serial.println(DHTData.KurilnicaTemp);
    Serial.print(F("SZgoraj: "));
    Serial.println(SolarZgorajTemp);
    Serial.print(F("SSpodaj: "));
    Serial.println(SolarSpodajTemp);
    Serial.print(F("Bojler: "));
    Serial.println(BojlerTemp);
    //Serial.print(F("Bde: "));
    //Serial.println(BojlerDiffForHeating);
    //Serial.print(F("MinFreeRAM: "));
    //Serial.println(LowestFreeRAM);
    //Serial.println(F("======================================"));
    // false=Manual, true=Auto
    
    if ((AutoControl) && (PecMaxTemp - Histereza >= (int) PecTemp)) {
      RelayEmergency3 = false;
      //START ko so plini > 70
      if ((( (int)PecTemp >= PecStartTemp + Histereza) && ( (int)DimTuljavaTemp >= DimTuljavaPoint )) || (( (int)PecTemp > PecStopTemp + Histereza ) && ( (int)DimTuljavaTemp < DimTuljavaPoint ))) {
        // bojler
        if (((int)BojlerTemp + BojlerDiffForHeating <= (int)PecTemp) && ((int)BojlerTemp < BojlerMaxTemp)) {
            RelayBojler1 = true;
            Serial.print(F("Bojler=ON"));
        }
        if ((int)BojlerTemp >= BojlerMaxTemp) {
            RelayBojler1 = false;
            Serial.print(F("Bojler=OFF"));
        }        
        // ogrevanje
        if ((float)DnevnaMinTemp > DHTData.DnevnaTemp - 0.2) {
          RelayOgrevanje0 = true;
          Serial.print(F("Ogrevanje=ON"));
        }
        if ((float)DnevnaMinTemp <= DHTData.DnevnaTemp + 0.2) {
          RelayOgrevanje0 = false;
          Serial.print(F("Ogrevanje=OFF"));
        }
      }
      if  ((( (int)PecTemp <= PecStartTemp - Histereza) && ((int)DimTuljavaTemp >= DimTuljavaPoint )) || (( (int)PecTemp < PecStopTemp - Histereza ) && ( (int)DimTuljavaTemp < DimTuljavaPoint ))) {
          RelayOgrevanje0 = false;
          RelayBojler1 = false;
          RelayEmergency3 = false;
      }
      //-----------------------------------------------------------
      // SOLAR
      if (SolarLock) {
        // overheating
        if (((int)SolarZgorajTemp > PecMaxTemp) && ((int)SolarSpodajTemp > PecMaxTemp)){
          RelaySolar2 = true;
          RelayBojler1 = true;
        }
        // heating in progress
        else if (((int)SolarZgorajTemp >= (int)BojlerTemp + BojlerDiffForHeating) || ((int)SolarSpodajTemp >= (int)BojlerTemp + BojlerDiffForHeating)) {
          SolarLock = false;
          SolarPreviousTime = millis();
          RelaySolar2 = true;
        }
        // warming up pipes
        else if (((int)SolarZgorajTemp >= SolarMaxTemp) && ((int)SolarSpodajTemp <= (int)BojlerTemp + BojlerDiffForHeating)) {
          SolarLock = false;
          SolarPreviousTime = millis();
          RelaySolar2 = true;
        }
        // anti-freeze
        else if (((int)SolarZgorajTemp < 0) || ((int)SolarSpodajTemp < 0)) {
          RelaySolar2 = true;
        }
        // solar temp too low
        else {
          RelaySolar2 = false;
        }
      } // end if SolarLock
      SolarSistemWorking = EEPROM.read(EADD_SolarSistemWorking);
      if (SolarSistemWorking) {
        // ce je sistem vkljucem (true)
        if (RelaySolar2) {
          PCF8574_Write_Pin(32, 2, 0);
        }
        else {
          PCF8574_Write_Pin(32, 2, 1);
        }
      }
      else {
        // Solarni sistem je izkljucen (false)
        PCF8574_Write_Pin(32, 2, 1);
        RelaySolar2 = false; // tudi avtomatika postavi sprem. na ON, jo ponastavim!!!
      }
    } // end if manual/auto
    if (PecMaxTemp + Histereza <= (int) PecTemp ) { // !!! CENTRALNA EMERGENCY OVERHEATING !!!
      RelayOgrevanje0 = true;
      RelayBojler1 = true;
      RelayEmergency3 = true;
    }
    VarToPCF(); // glede na spremenljivke poklicem funkcije za PCF
    wdt_reset(); // na 5 sec resetiramo wdt, da ne resetira arduinota
    LEDBlinkSpeed = 300;
  } // KONEC if za 5 sec interval!!!
  SolarTimer();
  WebServer();
  BlinkLED();
}

void LoadDefaults() {
  BojlerDiffForHeating = 5; //Razlika kdaj ogrevamo Bojler
  PecStartTemp = 65;        //Temp ko pec lahko ogreva ostalo
  PecStopTemp = 40;         //ugasnemo ogrevanje
  PecMaxTemp = 85;          //Pec Max temp
  BojlerMaxTemp = 53;       //Bojler Max temp
//  BojlerMinTemp = 45;       //Min temp bojlerja
  DnevnaMinTemp = 23;       //Min temp v dnevni
  DimTuljavaPoint = 80;     //Meja kdaj se Pec ohlaja/ogreva
  SolarMaxTemp = 80;
  Histereza = 2;
  DefaultsLoaded = 16;
}

void BlinkLED() {
  if (millis() - LEDPreviousTime >= LEDBlinkSpeed) {
    LEDPreviousTime = millis();
    if (digitalRead(ledPin)) {
      digitalWrite(ledPin, LOW);
    }
    else {
      digitalWrite(ledPin, HIGH);
    }
  }
}

void VarToPCF() {
  // OGREVANJE RELAY
  if (RelayOgrevanje0) {
    PCF8574_Write_Pin(32, 0, 0);
  }
  else {
    PCF8574_Write_Pin(32, 0, 1);
  }
  // BOJLER  RELAY
  if (RelayBojler1) {
    PCF8574_Write_Pin(32, 1, 0);
  }
  else {
    PCF8574_Write_Pin(32, 1, 1);
  }
  // SOLAR RELAY
  if (RelaySolar2) {
    PCF8574_Write_Pin(32, 2, 0);
  }
  else {
    PCF8574_Write_Pin(32, 2, 1);
  }
  // EMERGENCY RELAY
  if (RelayEmergency3) {
    PCF8574_Write_Pin(32, 3, 0);
  }
  else {
    PCF8574_Write_Pin(32, 3, 1);
  }
}

void SolarTimer() {
  // warming up timer 5min (300000ms) izven loop-a 5 sek
  if (!SolarLock) {
    if (millis() - SolarPreviousTime >= 300000) {
      SolarLock = true; //resetiram SolarLock po preteku timerja
    }
  }
}


// inicializacija mrezne kartice
void InitNetwork() {
  PrepareSPIForUseETH();
  /* Check that the Ethernet controller exists */
  //Serial.println("Initialising the Ethernet controller");
  if (ether.begin(sizeof Ethernet::buffer, mymac, 8) == 0) {
      Serial.println( "Eth failed");
      //while (true)
          /* MT */ ;
  }
  // inicializacija mrezne
  //ether.begin(sizeof Ethernet::buffer, mymac, 8);
  /* Get a DHCP connection */
  Serial.println("Trying DHCP(timeout 1min)!");
  // Če mrežna ni priključena oz ne dela, tukaj caka 1min in potem gre naprej brez mrezne!!!
  //fixed = false;
  if (ether.dhcpSetup()) {
      ether.printIp("DHCP: ", ether.myip);
  }
  /* If DHCP fails, start with a hard-coded address */
  else {
      ether.staticSetup(myip, gwip, dnsip);
      ether.printIp("Using static IP: ", ether.myip);
      //fixed = true;
  }
}


//
//// inicializacija mrezne kartice
//void InitNetwork() {
//  PrepareSPIForUseETH();
//  if (ether.begin(sizeof Ethernet::buffer, mymac) == 0)
//    Serial.println(F("Eth controller failed"));
//    ether.staticSetup(myip, gwip, dnsip, netmask);
//  if (!ether.dhcpSetup()) {
//    Serial.println(F("DHCP failed"));
//  }
//  ether.printIp("IP:\t", ether.myip);
//  ether.printIp("MASK:\t", ether.netmask);
//  ether.printIp("GW:\t", ether.gwip);
//  ether.printIp("DNS:\t", ether.dnsip);
//  //Serial.print((freeRam()));
//  //Serial.println(F(" Free RAM (Network done) "));
//}

// sprejme argumente I2C address, pin in value (1 ali 0)
void PCF8574_Write_Pin(int address, uint8_t pin, uint8_t value) {
   byte portdata;
   byte error;
   Wire.requestFrom(address,1);
   if(Wire.available())     //If the request is available
   {
     portdata = Wire.read();       //Receive the data
   }
  if (value == 0) {
    PCF8574CurrentState &= ~(1 << pin);
  }
  else
  {
    PCF8574CurrentState |= (1 << pin);
  }
  Wire.beginTransmission(address);
  Wire.write(PCF8574CurrentState);
  //error = Wire.endTransmission(false);
  error = Wire.endTransmission();
  if (error != 0) {
    bitSet(ErrorCodes, 4);
    Serial.print(F("I2Cerr: "));
    Serial.println(error);
  }
  freeRam();
}

//    I2C error types
//    0:success
//    1:data too long to fit in transmit buffer
//    2:received NACK on transmit of address
//    3:received NACK on transmit of data
//    4:other error 
//    The default value is true.
//    Syntax
//
//    Wire.endTransmission()
//    Wire.endTransmission(stop)
//    Parameters
//
//    stop : boolean. true will send a stop message, releasing the bus after transmission. false will send a restart, keeping the connection active. 

void GetTermocoupleTemp() {
  //funkcija tcl.readC() vrne v celzijah, tclreadF() vrne v fahrenheit
  //Do not call readC/F() more than about four times per second,
  //as the MAX6675 has a 220 millisecond conversion time.
  // tocnost +-1,5stopinje, resolucija 0,25 stopinje
  PrepareSPIForUseMAX();
  // Thermocouple init
  Thermocouple tc1_senzor1 = Thermocouple(csTC1);    //instantiate the thermocouple object
  //Serial.print((freeRam()));
  //Serial.println(F(" Free RAM (GetTermocoupleTemp) "));
  if (tc1_senzor1.readC() < 0.0) {
    //MAX6675 gre od 0 in naprej, ce je senzor NC vrne neg. cifro
    Serial.println(F("Err: K-type"));
    bitSet(ErrorCodes, 3);
    //LastError = "Napaka pri branju termoclena K-type preko MAX6675 na SPI!";
    //ErrorCode = 200;
  }
  else {
    DimTuljavaTemp = tc1_senzor1.readC();
  }
  PrepareSPIForUseETH();
}

void PrepareSPIForUseMAX() {
  pinMode(csTC1, OUTPUT);
  digitalWrite(csTC1, HIGH);             //Start de-selected
  //SPI pin setup
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);
  //SPI configuration
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
  SPI.setClockDivider(SPI_CLOCK_DIV16);   //Set SPI clock freq to 1MHz
}

void PrepareSPIForUseETH() {
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT);
  digitalWrite(MOSI, HIGH);
  digitalWrite(MOSI, LOW);
  digitalWrite(SCK, LOW);
  SPCR = bit(SPE) | bit(MSTR); // 8 MHz @ 16
  bitSet(SPSR, SPI2X);
}

int freeRam ()
{
  extern int __heap_start, *__brkval;
  int v;
  if ( LowestFreeRAM > (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval)) {
    LowestFreeRAM = (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
  }
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void GetDHTSensorData() {
  //DHT INIT
  DHT SenzorDnevna(DHTDnevnaPin, DHTTYPE);
  DHT SenzorZunaj(DHTZunajPin, DHTTYPE);
  DHT SenzorKurilnica(DHTKurilnicaPin, DHTTYPE);
  // DHT INIT SETUP
  // DHT ini
  SenzorDnevna.begin();
  SenzorZunaj.begin();
  SenzorKurilnica.begin();
  //Serial.print((freeRam()));
  //Serial.println(F(" Free RAM (DHT done) "));
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slowsensor)
  DHTData.DnevnaHum  = SenzorDnevna.readHumidity();
  DHTData.DnevnaTemp = SenzorDnevna.readTemperature();
  DHTData.ZunajHum   = SenzorZunaj.readHumidity();
  DHTData.ZunajTemp  = SenzorZunaj.readTemperature();
  DHTData.KurilnicaHum   = SenzorKurilnica.readHumidity();
  DHTData.KurilnicaTemp  = SenzorKurilnica.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(DHTData.DnevnaHum) || isnan(DHTData.DnevnaTemp)) {
    Serial.println(F("Err:DHT Dvna!"));
    bitSet(ErrorCodes, 0);
    //ErrorCode = 100;
  }
  if (isnan(DHTData.ZunajHum) || isnan(DHTData.ZunajTemp)) {
    Serial.println(F("Err:DHT Znj!"));
    bitSet(ErrorCodes, 1);
    //ErrorCode = 100;
  }
  if (isnan(DHTData.KurilnicaHum) || isnan(DHTData.KurilnicaTemp)) {
    Serial.println(F("Err:DHT Kla!"));
    bitSet(ErrorCodes, 2);
    //ErrorCode = 100;
  }
  freeRam();
}

static void WebServer() {
  PrepareSPIForUseETH();
  word len = ether.packetReceive();
  word pos = ether.packetLoop(len);
  if (pos) {
    LEDBlinkSpeed = 100;
    if (strstr((char *)Ethernet::buffer + pos, getdata) != 0) {
      SendData();
    }
    else if (strstr((char *)Ethernet::buffer + pos, getsettings) != 0) {
      SendSettings();
    }
    else if (strstr((char *)Ethernet::buffer + pos, setvton) != 0) {
      PCF8574_Write_Pin(32, 4, 0);
      //Serial.println(F("Vticnica = ON"));
    }
    else if (strstr((char *)Ethernet::buffer + pos, setvtoff) != 0) {
      PCF8574_Write_Pin(32, 4, 1);
      //Serial.println(F("Vticnica = OFF"));
    }
    
    // +- PecStartTemp
    else if (strstr((char *)Ethernet::buffer + pos, "?u1m") != 0) {
      if (WebNastavitveLock) {
        PecStartTemp = PecStartTemp - 1;
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // +- PecStartTemp
    else if (strstr((char *)Ethernet::buffer + pos, "?u1p") != 0) {
      if (WebNastavitveLock) {
        PecStartTemp = PecStartTemp + 1;
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // +- PecStopTemp
    else if (strstr((char *)Ethernet::buffer + pos, "?u2m") != 0) {
      if (WebNastavitveLock) {
        PecStopTemp = PecStopTemp - 1;
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // +- PecStopTemp
    else if (strstr((char *)Ethernet::buffer + pos, "?u2p") != 0) {
      if (WebNastavitveLock) {
        PecStopTemp = PecStopTemp + 1;
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // +- PecMaxTemp
    else if (strstr((char *)Ethernet::buffer + pos, "?u3m") != 0) {
      if (WebNastavitveLock) {
        PecMaxTemp = PecMaxTemp - 1;
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // +- PecMaxTemp
    else if (strstr((char *)Ethernet::buffer + pos, "?u3p") != 0) {
      if (WebNastavitveLock) {
        PecMaxTemp = PecMaxTemp + 1;
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // +- BojlerDiffForHeating
    else if (strstr((char *)Ethernet::buffer + pos, "?u4m") != 0) {
      if (WebNastavitveLock) {
        if (BojlerDiffForHeating >= 1) {
          BojlerDiffForHeating = BojlerDiffForHeating - 1;
        }
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // +- BojlerDiffForHeating
    else if (strstr((char *)Ethernet::buffer + pos, "?u4p") != 0) {
      if (WebNastavitveLock) {
        BojlerDiffForHeating = BojlerDiffForHeating + 1;
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // +- BojlerMaxTemp
    else if (strstr((char *)Ethernet::buffer + pos, "?u5m") != 0) {
      if (WebNastavitveLock) {
        BojlerMaxTemp = BojlerMaxTemp - 1;
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // +- BojlerMaxTemp
    else if (strstr((char *)Ethernet::buffer + pos, "?u5p") != 0) {
      if (WebNastavitveLock) {
        BojlerMaxTemp = BojlerMaxTemp + 1;
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
//    // +- BojlerMinTemp
//    else if (strstr((char *)Ethernet::buffer + pos, "?u6m") != 0) {
//      if (WebNastavitveLock) {
//        BojlerMinTemp = BojlerMinTemp - 1;
//        WebNastavitveLock = false;
//      }
//      RedirectToNastavitve();
//    }
//    // +- BojlerMinTemp
//    else if (strstr((char *)Ethernet::buffer + pos, "?u6p") != 0) {
//      if (WebNastavitveLock) {
//        BojlerMinTemp = BojlerMinTemp + 1;
//        WebNastavitveLock = false;
//      }
//      RedirectToNastavitve();
//    }
    // +- DnevnaMinTemp
    else if (strstr((char *)Ethernet::buffer + pos, "?u7m") != 0) {
      if (WebNastavitveLock) {
        DnevnaMinTemp = DnevnaMinTemp - 1;
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // +- DnevnaMinTemp
    else if (strstr((char *)Ethernet::buffer + pos, "?u7p") != 0) {
      if (WebNastavitveLock) {
        DnevnaMinTemp = DnevnaMinTemp + 1;
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // +- DimTuljavaPoint
    else if (strstr((char *)Ethernet::buffer + pos, "?u8m") != 0) {
      if (WebNastavitveLock) {
        DimTuljavaPoint = DimTuljavaPoint - 1;
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // +- DimTuljavaPoint
    else if (strstr((char *)Ethernet::buffer + pos, "?u8p") != 0) {
      if (WebNastavitveLock) {
        DimTuljavaPoint = DimTuljavaPoint + 1;
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // +- SolarMaxTemp
    else if (strstr((char *)Ethernet::buffer + pos, "?u9m") != 0) {
      if (WebNastavitveLock) {
        if (SolarMaxTemp > 20) { 
          SolarMaxTemp = SolarMaxTemp - 1;
        }
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // +- SolarMaxTemp
    else if (strstr((char *)Ethernet::buffer + pos, "?u9p") != 0) {
      if (WebNastavitveLock) {
        SolarMaxTemp = SolarMaxTemp + 1;
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // +- Histereza
    else if (strstr((char *)Ethernet::buffer + pos, "?hp") != 0) {
      if (WebNastavitveLock) {
        Histereza = Histereza + 1;
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // +- Histereza
    else if (strstr((char *)Ethernet::buffer + pos, "?hm") != 0) {
      if (WebNastavitveLock) {
        Histereza = Histereza - 1;
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // SHRANI
    else if (strstr((char *)Ethernet::buffer + pos, shrani) != 0) {
      if (WebNastavitveLock) {
        EEPROM.write(EADD_SolarSistemWorking, SolarSistemWorking);
        EEPROM.write(EADD_BojlerDiffForHeating, BojlerDiffForHeating);
        EEPROM.write(EADD_PecStartTemp, PecStartTemp);
        EEPROM.write(EADD_PecStopTemp, PecStopTemp);
        EEPROM.write(EADD_PecMaxTemp, PecMaxTemp);
        EEPROM.write(EADD_BojlerMaxTemp, BojlerMaxTemp);
        //EEPROM.write(EADD_BojlerMinTemp, BojlerMinTemp);
        EEPROM.write(EADD_DnevnaMinTemp, DnevnaMinTemp);
        EEPROM.write(EADD_DimTuljavaPoint, DimTuljavaPoint);
        EEPROM.write(EADD_SolarMaxTemp, SolarMaxTemp);
        EEPROM.write(EADD_Histereza, Histereza);
        EEPROM.write(EADD_DefaultsLoaded, DefaultsLoaded);
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // DEFAULT 
    else if (strstr((char *)Ethernet::buffer + pos, load_default) != 0) {
      if (WebNastavitveLock) {
        LoadDefaults();
        WebNastavitveLock = false;
      }
      RedirectToNastavitve();
    }
    // VKLOP/IZKLOP CRPALKE ZA SOLARNO
    else if (strstr((char *)Ethernet::buffer + pos, setSolarPumpon) != 0) {
      if (!AutoControl) {
        if (SolarSistemWorking) {
          PCF8574_Write_Pin(32, 2, 0);
          RelaySolar2 = true;
        }
      }
      RedirectToNastavitve();
    }
    // turn off solar pump
    else if (strstr((char *)Ethernet::buffer + pos, setSolarPumpoff) != 0) {
      if (!AutoControl) {
        PCF8574_Write_Pin(32, 2, 1);
        RelaySolar2 = false;
      }
      RedirectToNastavitve();
    }
    // VKLOP/IZKLOP CRPALKE ZA BOJLER-PEC
    else if (strstr((char *)Ethernet::buffer + pos, setBojlerPumpon) != 0) {
      if (!AutoControl) {
        PCF8574_Write_Pin(32, 1, 0);
        RelayBojler1 = true;
      }
      RedirectToNastavitve();
    }
    // turn off bojler pump
    else if (strstr((char *)Ethernet::buffer + pos, setBojlerPumpoff) != 0) {
      if (!AutoControl) {
        PCF8574_Write_Pin(32, 1, 1);
        RelayBojler1 = false;
      }
      RedirectToNastavitve();
    }
    // VKLOP/IZKLOP CRPALKE ZA RADIATORJE
    else if (strstr((char *)Ethernet::buffer + pos, setHeatingPumpon) != 0) {
      if (!AutoControl) {
        PCF8574_Write_Pin(32, 0, 0);
        RelayOgrevanje0 = true;
      }
      RedirectToNastavitve();
    }
    // turn off heating pump
    else if (strstr((char *)Ethernet::buffer + pos, setHeatingPumpoff) != 0) {
      if (!AutoControl) {
        PCF8574_Write_Pin(32, 0, 1);
        RelayOgrevanje0 = false;
      }
      RedirectToNastavitve();
    }
    // popravek default nastavitve za kasnejse tunanje Manual vklop crpalk
    else if (strstr((char *)Ethernet::buffer + pos, setAutoControloff) != 0) {
      AutoControl = false;
      RedirectToNastavitve();
    }
    // popravek default nastavitve za kasnejse tunanje Manual vklop crpalk
    else if (strstr((char *)Ethernet::buffer + pos, setAutoControlon) != 0) {
      AutoControl = true;
      RedirectToNastavitve();
    }
    // SOLAR SISTEM ON/OFF
    else if (strstr((char *)Ethernet::buffer + pos, setSolarSistemoff) != 0) {
      EEPROM.write(EADD_SolarSistemWorking, false);
      PCF8574_Write_Pin(32, 2, 1);
      RelaySolar2 = false;
      RedirectToNastavitve();
    }
    else if (strstr((char *)Ethernet::buffer + pos, setSolarSistemon) != 0) {
      EEPROM.write(EADD_SolarSistemWorking, true);
      RedirectToNastavitve();
    }
    else if (strstr((char *)Ethernet::buffer + pos, showdata) != 0) {
      ShowData();
    }
    else if (strstr((char *)Ethernet::buffer + pos, settings) != 0) {
      Nastavitve();
    }
    else {
      HomePage();
    }
    // auto redirect to Homepage
    if (strstr((char *)Ethernet::buffer + pos, get) != 0) {
      RedirectToHome();
    }
  }
  freeRam();
}  
  
static void SendData() {
  BufferFiller bfill = ether.tcpOffset();
  bfill.emit_p(PSTR(
                 "{"
                 "\"DimTuljavaTemp\": \"$T\", "
                 "\"DnevnaTemp\": \"$T\", "
                 "\"DnevnaHum\": \"$T\", "
                 "\"ZunajTemp\": \"$T\", "
                 "\"ZunajHum\": \"$T\", "
                 "\"KurTemp\": \"$T\", "
                 "\"KurHum\": \"$T\", "
                 "\"IzPeci\": \"$T\", "
                 "\"VPec\": \"$T\", "
                 "\"PecTemp\": \"$T\", "
                 "\"SolarZgoraj\": \"$T\", "
                 "\"SolarSpodaj\": \"$T\", "
                 "\"BojlerTemp\": \"$T\" "
                 "}"
               ), (double) DimTuljavaTemp, (double) DHTData.DnevnaTemp, (double) DHTData.DnevnaHum, (double) DHTData.ZunajTemp, (double) DHTData.ZunajHum, (double) DHTData.KurilnicaTemp, (double) DHTData.KurilnicaHum, (double) VodaIzPeciTemp, (double) VodaVPecTemp, (double) PecTemp, (double) SolarZgorajTemp, (double) SolarSpodajTemp, (double) BojlerTemp);
  ether.httpServerReply(bfill.position());
}

static void SendSettings() {
  BufferFiller bfill = ether.tcpOffset();
  bfill.emit_p(PSTR(
                 "{"
                 "\"Uptime\": \"$L\", "
                 "\"FreeRAM\": \"$D\", "
                 "\"PCF8574\": \"$D\", "
                 "\"PecStartTemp\": \"$D\", "
                 "\"MaxTemp\": \"$D\", "
                 "\"BojlerDiffForHeating\": \"$D\", "
                 "\"DnevnaMinTemp\": \"$D\", "
                 "\"AutoControl\": \"$D\", "
                 "\"Error\": \"$D\" "
                 "}"
               ), (unsigned long) millis() / 1000, LowestFreeRAM, PCF8574CurrentState, PecStartTemp, PecMaxTemp, BojlerDiffForHeating, DnevnaMinTemp, AutoControl, ErrorCodes);
  ether.httpServerReply(bfill.position());
}

//static void SendData() {
//  BufferFiller bfill = ether.tcpOffset();
//  bfill.emit_p(PSTR(
//                 "{"
//                 "\"1\": \"$T\", "  //DimTuljavaTemp
//                 "\"2\": \"$T\", "  //DnevnaTemp
//                 "\"3\": \"$T\", "  //DnevnaHum
//                 "\"4\": \"$T\", "  //ZunajTemp
//                 "\"5\": \"$T\", "  //ZunajHum
//                 "\"6\": \"$T\", "  //KurTemp
//                 "\"7\": \"$T\", "  //KurHum
//                 "\"8\": \"$T\", "  //IzPeci
//                 "\"9\": \"$T\", "  //VPec
//                 "\"a\": \"$T\", "  //PecTemp
//                 "\"b\": \"$T\", "  //SolarZgoraj
//                 "\"c\": \"$T\", "  //SolarSpodaj
//                 "\"d\": \"$T\" "   //BojlerTemp
//                 "}"
//               ), (double) DimTuljavaTemp, (double) DHTData.DnevnaTemp, (double) DHTData.DnevnaHum, (double) DHTData.ZunajTemp, (double) DHTData.ZunajHum, (double) DHTData.KurilnicaTemp, (double) DHTData.KurilnicaHum, (double) VodaIzPeciTemp, (double) VodaVPecTemp, (double) PecTemp, (double) SolarZgorajTemp, (double) SolarSpodajTemp, (double) BojlerTemp);
//  ether.httpServerReply(bfill.position());
//}
//
//static void SendSettings() {
//  BufferFiller bfill = ether.tcpOffset();
//  bfill.emit_p(PSTR(
//                 "{"
//                 "\"1\": \"$L\", " // Uptime
//                 "\"2\": \"$D\", " // FreeRAM
//                 "\"3\": \"$D\", " // PeStartTemp
//                 "\"4\": \"$D\", " // MaxTemp
//                 "\"5\": \"$D\", " // BojlerDiffForHeating
//                 "\"6\": \"$D\", " // DnevnaMinTemp
//                 "\"7\": \"$D\", " // BojlerMinTemp
//                 "\"8\": \"$D\" "  // AutoControl
//                 "}"
//               ), (unsigned long) millis() / 1000, LowestFreeRAM,PecStartTemp, MaxTemp, BojlerDiffForHeating, DnevnaMinTemp, BojlerMinTemp, AutoControl);
//  ether.httpServerReply(bfill.position());
//}

static void Send_ACK() {
  // posljem char iz PROGMEM-a
  ether.httpServerReplyAck(); // send ack to the request
}


static void Send_Part_PROGMEM_FIN(const char *part, int size) {
  // posljem char iz PROGMEM-a
  memcpy_P(ether.tcpOffset(), part, size);
  ether.httpServerReply_with_flags(size - 1, TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V);
}
      
void Send_Part_PROGMEM_SEQ(const char *part, int size) {
  // posljem char iz PROGMEM-a
  memcpy_P(ether.tcpOffset(), part, size);
  ether.httpServerReply_with_flags(size - 1, TCP_FLAGS_ACK_V);
}

static void Send_Part_SRAM_SEQ(char *part, int size) {
  // posljem char iz SRAM-a
  memcpy(ether.tcpOffset(), part, size);
  ether.httpServerReply_with_flags(size - 1, TCP_FLAGS_ACK_V);
}

static void RedirectToHome() {
  Send_ACK();
  Send_Part_PROGMEM_SEQ(pageHEADER, sizeof pageHEADER);
  Send_Part_PROGMEM_SEQ(pageBACK, sizeof pageBACK);      // redirect na http:/ip/
  Send_Part_PROGMEM_FIN(pageEND, sizeof pageEND);
}

static void RedirectToNastavitve() {
  Send_ACK();
  Send_Part_PROGMEM_SEQ(pageHEADER, sizeof pageHEADER);
  Send_Part_PROGMEM_SEQ(pageBACKTONASTAVITVE, sizeof pageBACKTONASTAVITVE);   // redirect na http:/ip/nastavitve
  Send_Part_PROGMEM_FIN(pageEND, sizeof pageEND);
}

static void HomePage() {
  Send_ACK();
  Send_Part_PROGMEM_SEQ(pageHEADER, sizeof pageHEADER);
  if (bitRead(PCF8574CurrentState, 4)) {
    Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
    Send_Part_PROGMEM_SEQ(pageVTON, sizeof pageVTON);     // gumb vtičnica ON
  }
  else {
    Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
    Send_Part_PROGMEM_SEQ(pageVTOFF, sizeof pageVTOFF);    // gumb vtičnica OFF
  }
  Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
  Send_Part_PROGMEM_SEQ(pagePRIKAZPODATKOV, sizeof pagePRIKAZPODATKOV); // gumb prikaz podatkov
  Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
  Send_Part_PROGMEM_SEQ(pageNASTAVITVE, sizeof pageNASTAVITVE);     // gumb nastavitve
  Send_Part_PROGMEM_FIN(pageEND, sizeof pageEND);
}

static void ShowData() {
  Send_ACK();
  Send_Part_PROGMEM_SEQ(pageHEADER, sizeof pageHEADER);
  Send_Part_PROGMEM_SEQ(pageREFRESH, sizeof pageREFRESH);
  Send_Part_PROGMEM_SEQ(trtd, sizeof trtd);
  char temp[7];
  dtostrf(DHTData.DnevnaTemp, 5, 1, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp);
//  if (DHTData.DnevnaTemp < DnevnaMinTemp) {
//    Send_Part_PROGMEM_SEQ(red, sizeof red);
//  }
//  else {
//    Send_Part_PROGMEM_SEQ(green, sizeof green);
//  }
  Send_Part_PROGMEM_SEQ(deg, sizeof deg);
  Send_Part_PROGMEM_SEQ(tdtd, sizeof tdtd);
  dtostrf(DHTData.ZunajTemp, 5, 1, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp);
  Send_Part_PROGMEM_SEQ(deg, sizeof deg);
  Send_Part_PROGMEM_SEQ(tdtd, sizeof tdtd);
  dtostrf(DHTData.KurilnicaTemp, 5, 1, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp);
  Send_Part_PROGMEM_SEQ(deg, sizeof deg);
  Send_Part_PROGMEM_SEQ(tdtr, sizeof tdtr);
  Send_Part_PROGMEM_SEQ(trtd, sizeof trtd);
  dtostrf(DHTData.DnevnaHum, 5, 1, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp);
  Send_Part_PROGMEM_SEQ(per, sizeof per);
  Send_Part_PROGMEM_SEQ(tdtd, sizeof tdtd);
  dtostrf(DHTData.ZunajHum, 5, 1, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp);
  Send_Part_PROGMEM_SEQ(per, sizeof per);
  Send_Part_PROGMEM_SEQ(tdtd, sizeof tdtd);
  dtostrf(DHTData.KurilnicaHum, 5, 1, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp);
  Send_Part_PROGMEM_SEQ(per, sizeof per);
  Send_Part_PROGMEM_SEQ(tdtr, sizeof tdtr);
  Send_Part_PROGMEM_SEQ(trtdspacetdtr, sizeof trtdspacetdtr);
  Send_Part_PROGMEM_SEQ(pageREFRESHMID, sizeof pageREFRESHMID);
  Send_Part_PROGMEM_SEQ(trtd, sizeof trtd);
  dtostrf(BojlerTemp, 4, 2, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp); 
  Send_Part_PROGMEM_SEQ(deg, sizeof deg);
  Send_Part_PROGMEM_SEQ(tdtd, sizeof tdtd);
  dtostrf(PecTemp, 4, 2, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp); 
  Send_Part_PROGMEM_SEQ(deg, sizeof deg);
  Send_Part_PROGMEM_SEQ(tdtd, sizeof tdtd);
  dtostrf(DimTuljavaTemp, 4, 2, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp);
  Send_Part_PROGMEM_SEQ(deg, sizeof deg);
  Send_Part_PROGMEM_SEQ(tdtr, sizeof tdtr);
  Send_Part_PROGMEM_SEQ(trtdspacetdtr, sizeof trtdspacetdtr);
  Send_Part_PROGMEM_SEQ(pageREFRESHMID1, sizeof pageREFRESHMID1);
  Send_Part_PROGMEM_SEQ(trtd, sizeof trtd);
  if (RelayOgrevanje0) {
    Send_Part_PROGMEM_SEQ(on, sizeof on);
  }
  else {
    Send_Part_PROGMEM_SEQ(off, sizeof off);
  }
  Send_Part_PROGMEM_SEQ(tdtd, sizeof tdtd);
  if (RelayBojler1) {
    Send_Part_PROGMEM_SEQ(on, sizeof on);
  }
  else {
    Send_Part_PROGMEM_SEQ(off, sizeof off);
  }
  Send_Part_PROGMEM_SEQ(tdtd, sizeof tdtd);
  if (RelayEmergency3) {
    Send_Part_PROGMEM_SEQ(on, sizeof on);
  }
  else {
    Send_Part_PROGMEM_SEQ(off, sizeof off);
  }
  Send_Part_PROGMEM_SEQ(tdtr, sizeof tdtr);
  Send_Part_PROGMEM_SEQ(trtdspacetdtr, sizeof trtdspacetdtr);
  Send_Part_PROGMEM_SEQ(pageREFRESHMID2, sizeof pageREFRESHMID2);
  Send_Part_PROGMEM_SEQ(trtd, sizeof trtd);
  if (RelaySolar2) {
    Send_Part_PROGMEM_SEQ(on, sizeof on);
  }
  else {
    Send_Part_PROGMEM_SEQ(off, sizeof off);
  }
  Send_Part_PROGMEM_SEQ(tdtd, sizeof tdtd);
  dtostrf(SolarZgorajTemp, 4, 2, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp);
  Send_Part_PROGMEM_SEQ(deg, sizeof deg);
  Send_Part_PROGMEM_SEQ(tdtd, sizeof tdtd);
  dtostrf(SolarSpodajTemp, 4, 2, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp);
  Send_Part_PROGMEM_SEQ(deg, sizeof deg);
  Send_Part_PROGMEM_SEQ(tdtr, sizeof tdtr);
  Send_Part_PROGMEM_SEQ(trtdspacetdtr, sizeof trtdspacetdtr);
  Send_Part_PROGMEM_SEQ(pageREFRESHMID3, sizeof pageREFRESHMID3);
  Send_Part_PROGMEM_SEQ(trtd, sizeof trtd);
  dtostrf(VodaIzPeciTemp, 4, 2, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp);
  Send_Part_PROGMEM_SEQ(deg, sizeof deg);
  Send_Part_PROGMEM_SEQ(tdtd, sizeof tdtd);
  dtostrf(VodaVPecTemp, 4, 2, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp);
  Send_Part_PROGMEM_SEQ(deg, sizeof deg);
  Send_Part_PROGMEM_SEQ(tdtd, sizeof tdtd);
//  long t = millis() / 1000;
//  word h = t / 3600;
//  byte m = (t / 60) % 60;
//  byte s = t % 60;
  long days = 0;
  long hours = 0;
  long mins = 0;
  unsigned long secs = 0;
  secs = millis()/1000; //convert milliseconds to seconds
  mins=secs/60; //convert seconds to minutes
  hours=mins/60; //convert minutes to hours
  days=hours/24; //convert hours to days
  secs=secs-(mins*60); //subtract the coverted seconds to minutes in order to display 59 secs max
  mins=mins-(hours*60); //subtract the coverted minutes to hours in order to display 59 minutes max
  hours=hours-(days*24); //subtract the coverted hours to days in order to display 23 hours max
  dtostrf(days, 6, 0, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp);
  Send_Part_PROGMEM_SEQ(cdays, sizeof cdays);
  dtostrf(hours, 6, 0, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp);
  Send_Part_PROGMEM_SEQ(chours, sizeof chours);
  dtostrf(mins, 6, 0, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp);
  Send_Part_PROGMEM_SEQ(cmins, sizeof cmins);
  dtostrf(secs, 6, 0, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp);
  Send_Part_PROGMEM_SEQ(csecs, sizeof csecs);
  Send_Part_PROGMEM_SEQ(tdtr, sizeof tdtr);
  Send_Part_PROGMEM_SEQ(trtdspacetdtr, sizeof trtdspacetdtr);
  Send_Part_PROGMEM_SEQ(tableend, sizeof tableend);
  Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
  Send_Part_PROGMEM_SEQ(pageNAZAJ, sizeof pageNAZAJ);
  Send_Part_PROGMEM_FIN(pageEND, sizeof pageEND);
}

static void Gumb_PM(const char *znak, int znak_size, const char *link, int link_size) {
  Send_Part_PROGMEM_SEQ(GUMB_PM_1_3, sizeof GUMB_PM_1_3);
  Send_Part_PROGMEM_SEQ(link, link_size);
  Send_Part_PROGMEM_SEQ(GUMB_PM_2_3, sizeof GUMB_PM_2_3);
  Send_Part_PROGMEM_SEQ(znak, znak_size);
  Send_Part_PROGMEM_SEQ(GUMB_PM_3_3, sizeof GUMB_PM_3_3);
}
static void Nastavitve_Tabela_Gumbov_PM() {
  Send_Part_PROGMEM_SEQ(Gumb_PM_Tabela, sizeof Gumb_PM_Tabela);
}
static void Nastavitve_Tabela_Vrstica(const char *ime,int ime_size, const char *url_gumb_plus, int url_gumb_plus_size, const char *url_gumb_minus, int url_gumb_minus_size, int var, const char *enota, int enota_size) {
  Send_Part_PROGMEM_SEQ(trtd, sizeof trtd);
  Send_Part_PROGMEM_SEQ(ime, ime_size);
  Send_Part_PROGMEM_SEQ(tdtd, sizeof tdtd);
  Gumb_PM(GUMB_PM_MI, sizeof GUMB_PM_MI, url_gumb_minus, url_gumb_minus_size);
  Send_Part_PROGMEM_SEQ(tdtd, sizeof tdtd);
  char temp[7];
  dtostrf(var, 6, 0, temp);
  Send_Part_SRAM_SEQ(temp, sizeof temp);
  Send_Part_PROGMEM_SEQ(enota, enota_size);
  Send_Part_PROGMEM_SEQ(tdtd, sizeof tdtd);
  Gumb_PM(GUMB_PM_PL, sizeof GUMB_PM_PL, url_gumb_plus, url_gumb_plus_size);
  Send_Part_PROGMEM_SEQ(tdtr, sizeof tdtr);
}


static void Nastavitve() {
  WebNastavitveLock = true;
  Send_ACK();
  Send_Part_PROGMEM_SEQ(pageHEADER, sizeof pageHEADER);
  Send_Part_PROGMEM_SEQ(Gumb_PM_Tabela, sizeof Gumb_PM_Tabela);
  Nastavitve_Tabela_Vrstica(u1i,sizeof u1i,u1p,sizeof u1p,u1m,sizeof u1m,PecStartTemp, deg, sizeof deg);
  Nastavitve_Tabela_Vrstica(u2i,sizeof u2i,u2p,sizeof u2p,u2m,sizeof u2m,PecStopTemp, deg, sizeof deg);
  Nastavitve_Tabela_Vrstica(u3i,sizeof u3i,u3p,sizeof u3p,u3m,sizeof u3m,PecMaxTemp, deg, sizeof deg);
  Nastavitve_Tabela_Vrstica(u4i,sizeof u4i,u4p,sizeof u4p,u4m,sizeof u4m,BojlerDiffForHeating, deg, sizeof deg);
  Nastavitve_Tabela_Vrstica(u5i,sizeof u5i,u5p,sizeof u5p,u5m,sizeof u5m,BojlerMaxTemp, deg, sizeof deg);
//  Nastavitve_Tabela_Vrstica(u6i,sizeof u6i,u6p,sizeof u6p,u6m,sizeof u6m,BojlerMinTemp, deg, sizeof deg);
  Nastavitve_Tabela_Vrstica(u7i,sizeof u7i,u7p,sizeof u7p,u7m,sizeof u7m,DnevnaMinTemp, deg, sizeof deg);
  Nastavitve_Tabela_Vrstica(u8i,sizeof u8i,u8p,sizeof u8p,u8m,sizeof u8m,DimTuljavaPoint, deg, sizeof deg);
  Nastavitve_Tabela_Vrstica(u9i,sizeof u9i,u9p,sizeof u9p,u9m,sizeof u9m,SolarMaxTemp, deg, sizeof deg);
  Nastavitve_Tabela_Vrstica(hi,sizeof hi,hp,sizeof hp,hm,sizeof hm,Histereza, deg, sizeof deg);
  Send_Part_PROGMEM_SEQ(tableend, sizeof tableend);
  Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
  Send_Part_PROGMEM_SEQ(pageSHRANI, sizeof pageSHRANI);
  Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
  Send_Part_PROGMEM_SEQ(pageDEFAULT, sizeof pageDEFAULT);
  if (AutoControl) {
    Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
    Send_Part_PROGMEM_SEQ(pageMANUALC, sizeof pageMANUALC);       // gumb Manual control
  }
  else {
    Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
    Send_Part_PROGMEM_SEQ(pageAUTOC,sizeof pageAUTOC);         // gumb Auto control
    if (bitRead(PCF8574CurrentState, 0)) {
      Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
      Send_Part_PROGMEM_SEQ(pagePUMPHEATON, sizeof pagePUMPHEATON);  // gumb Vklop pump ogrevanje
    }
    else {
      Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
      Send_Part_PROGMEM_SEQ(pagePUMPHEATOFF, sizeof pagePUMPHEATOFF); // gumb Izklop pump ogrevanje
    }
    if (bitRead(PCF8574CurrentState, 1)) {
      Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
      Send_Part_PROGMEM_SEQ(pagePUMPBOJLERON, sizeof pagePUMPBOJLERON);// gumb Vklop pump bojler
    }
    else {
      Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
      Send_Part_PROGMEM_SEQ(pagePUMPBOJLEROFF, sizeof pagePUMPBOJLEROFF); // gumb Izklop pump bojler
    }
    SolarSistemWorking = EEPROM.read(EADD_SolarSistemWorking);
    if (SolarSistemWorking) {
      if (bitRead(PCF8574CurrentState, 2)) {
        Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
        Send_Part_PROGMEM_SEQ(pagePUMPSOLARON, sizeof pagePUMPSOLARON); // gumb Vklop pump solar
      }
      else {
        Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
        Send_Part_PROGMEM_SEQ(pagePUMPSOLAROFF, sizeof pagePUMPSOLAROFF); // gumb Izklop pump solar
      }
    }
  }
  SolarSistemWorking = EEPROM.read(EADD_SolarSistemWorking);
  if (SolarSistemWorking) {
    Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
    Send_Part_PROGMEM_SEQ(pageSOLARSISTEMOFF, sizeof pageSOLARSISTEMOFF); // gumb Izklop Solar sistema
  }
  else {
    Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
    Send_Part_PROGMEM_SEQ(pageSOLARSISTEMON, sizeof pageSOLARSISTEMON);  // gumb Vklop Solar sistema
  }
  Send_Part_PROGMEM_SEQ(tableend, sizeof tableend);
  Send_Part_PROGMEM_SEQ(ButtonHeaderSets, sizeof ButtonHeaderSets);
  Send_Part_PROGMEM_SEQ(pageNAZAJ, sizeof pageNAZAJ);
  Send_Part_PROGMEM_FIN(pageEND, sizeof pageEND);
}
