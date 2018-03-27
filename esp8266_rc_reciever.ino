
#include <SoftwareSerial.h>                             // Software Serial Library so we can use other Pins for communication with the GPS module
#include <TinyGPS++.h>                                  // Tiny GPS Plus Library
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Servo.h>
#define RXPin 12
#define TXPin 13
#define MAPin 14
#define M1Pin 5
#define M2Pin 4
#define M3Pin 0
#define M4Pin 2
#define MBPin 15
static const uint32_t GPSBaud = 9600;                   // Ublox GPS default Baud Rate is 9600
const double Home_LAT = 48.004607;                      // Your Home Latitude
const double Home_LNG = -122.523667;                    // Your Home Longitude
TinyGPSPlus gps;                                        // Create an Instance of the TinyGPS++ object called gps
SoftwareSerial ss(RXPin, TXPin);                        // The serial connection to the GPS device

const char *ssid = "speedboatcontwifi";  // You will connect your phone to this Access Point
const char *pw = "pasword213"; // and this is the password
IPAddress ip(192, 168, 0, 1); // From RoboRemo app, connect to this IP
IPAddress netmask(255, 255, 255, 0);
const int port = 9876; // and this port
//legacy
const int chCount = 0; // 0 channels, you can add more if you have GPIOs :)
Servo servoCh[chCount]; // will generate 0 servo PWM signals
int chPin[] = {}; // ESP pins: GPIO 0, 2, 14, 12
int chVal[] = {1500, 1500, 1500, 1500}; // default value (middle)
int usMin = 700; // min pulse width
int usMax = 2300; // max pulse width
//end
WiFiServer server(port);
WiFiClient client;
char cmd[100]; // stores the command chars received from RoboRemo
int cmdIndex;
unsigned long lastCmdTime = 60000;
unsigned long aliveSentTime = 0;
boolean cmdStartsWith(const char *st) { // checks if cmd starts with st
  for(int i=0; ; i++) {
    if(st[i]==0) return true;
    if(cmd[i]==0) return false;
    if(cmd[i]!=st[i]) return false;;
  }
  return false;
}

void exeCmd() { // executes the command from cmd    
  lastCmdTime = millis();

  //BEGIN Jeromes Addition to this code..
  if(!strcmp(&cmd[3],"w")){
    Serial.println("w FORWARD allout");
    leftForward(1023);
    rightForward(1023);
  }
  if(!strcmp(&cmd[3],"a")){
    Serial.println("a LEFT allout");
    leftForward(1023);
    rightStop();
  }
  if(!strcmp(&cmd[3],"d")){
    Serial.println("d RIGHT allout");
    rightForward(1023);
    leftStop();
  }
  if(!strcmp(&cmd[3],"s")){
    Serial.println("s STOP");
    allStop();
  }
  
  if(!strcmp(&cmd[3],"t")){
    Serial.println("t FORWARD half");
    leftForward(511);
    rightForward(511);
  }
  if(!strcmp(&cmd[3],"f")){
    Serial.println("f LEFT half");
    leftForward(511);
    rightStop();
  }
  if(!strcmp(&cmd[3],"g")){
    Serial.println("g RIGHT half");
    rightForward(511);
    leftStop();
  }
  if(!strcmp(&cmd[3],"y")){
    Serial.println("y FORWARD low");
    leftForward(100);
    rightForward(100);
  }
  if(!strcmp(&cmd[3],"h")){
    Serial.println("h LEFT low");
    leftForward(100);
    rightStop();
  }
  if(!strcmp(&cmd[3],"j")){
    Serial.println("j RIGHT low");
    rightForward(100);
    leftStop();
  }
  //END

  // example: set RoboRemo slider id to "ch0", set min 1000 and set max 2000
  if( cmdStartsWith("ch") ) {
    int ch = cmd[2] - '0';
    if(ch>=0 && ch<=9 && cmd[3]==' ') {
      chVal[ch] = (int)atof(cmd+4);
      if(!servoCh[ch].attached()) {
        servoCh[ch].attach(chPin[ch], usMin, usMax);
      }   
      servoCh[ch].writeMicroseconds(chVal[ch]);
    }
  }
  
  // invert channel:
  // example: set RoboRemo slider id to "ci0", set min -2000 and set max -1000
  if( cmdStartsWith("ci") ) {
    int ch = cmd[2] - '0';
    if(ch>=0 && ch<=9 && cmd[3]==' ') {
      chVal[ch] = -(int)atof(cmd+4);
      if(!servoCh[ch].attached()) {
        servoCh[ch].attach(chPin[ch], usMin, usMax);
      }   
      servoCh[ch].writeMicroseconds(chVal[ch]);
    }
  }
  
  // use accelerometer:
  // example: set RoboRemo acc y id to "ca1"
  if( cmdStartsWith("ca") ) {
    int ch = cmd[2] - '0';
    if(ch>=0 && ch<=9 && cmd[3]==' ') {
      chVal[ch] = (usMax+usMin)/2 + (int)( atof(cmd+4)*51 ); // 9.8*51 = 500 => 1000 .. 2000
      if(!servoCh[ch].attached()) {
        servoCh[ch].attach(chPin[ch], usMin, usMax);
      }   
      servoCh[ch].writeMicroseconds(chVal[ch]);
    }
  }
  
  // invert accelerometer:
  // example: set RoboRemo acc y id to "cb1"
  if( cmdStartsWith("cb") ) {
    int ch = cmd[2] - '0';
    if(ch>=0 && ch<=9 && cmd[3]==' ') {
      chVal[ch] = (usMax+usMin)/2 - (int)( atof(cmd+4)*51 ); // 9.8*51 = 500 => 1000 .. 2000
      if(!servoCh[ch].attached()) {
        servoCh[ch].attach(chPin[ch], usMin, usMax);
      }   
      servoCh[ch].writeMicroseconds(chVal[ch]);
    }
  }
}

void allStop(){
  leftStop();
  rightStop();  
}
void leftForward(int mspeed){
  digitalWrite(M1Pin, HIGH);
  digitalWrite(M2Pin, LOW);
  analogWrite(MAPin, mspeed);
}
void leftStop(){
  digitalWrite(M1Pin, LOW);
  digitalWrite(M2Pin, LOW);
  analogWrite(MAPin, 0);  
}
void rightForward(int mspeed){
  digitalWrite(M3Pin, HIGH);
  digitalWrite(M4Pin, LOW);
  analogWrite(MBPin, mspeed);
}
void rightStop(){
  digitalWrite(M3Pin, LOW);
  digitalWrite(M4Pin, LOW);
  analogWrite(MBPin, 0);  
}

void setup()
{  
  Serial.begin(115200);  
  //delay(1500);                                          // Pause 1.5 seconds  
  Serial.println("*ignore above jibberish (if any)");
  Serial.println("hello world!!!");

  pinMode(MAPin,OUTPUT);
  pinMode(M1Pin,OUTPUT);
  pinMode(M2Pin,OUTPUT);
  pinMode(M3Pin,OUTPUT);
  pinMode(M4Pin,OUTPUT);
  pinMode(MBPin,OUTPUT);
  allStop();
  
  Serial.println(TinyGPSPlus::libraryVersion());
  ss.begin(GPSBaud);                                    // Set Software Serial Comm Speed   
  /*for(int i=0; i<chCount; i++) {
    // attach channels to pins
    servoCh[i].attach(chPin[i], usMin, usMax);
    // initial value = middle
    chVal[i] = (usMin + usMax)/2;
    // update
    servoCh[i].writeMicroseconds( chVal[i] );
  }*/
  cmdIndex = 0;

  WiFi.softAPConfig(ip, ip, netmask); // configure ip address for softAP 
  WiFi.softAP(ssid, pw); // configure ssid and password for softAP
  server.begin(); // start TCP server

  Serial.println("ESP8266 RC receiver 1.1 powered by RoboRemo");
  Serial.println((String)"SSID: " + ssid + "  PASS: " + pw);
  Serial.println((String)"RoboRemo app must connect to " + ip.toString() + ":" + port);
}

void loop(){    
  /*
  Serial.print("Latitude  : ");
  Serial.println(gps.location.lat(), 5);
  Serial.print("Longitude : ");
  Serial.println(gps.location.lng(), 4);
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.print("Elevation : ");
  Serial.print(gps.altitude.feet());
  Serial.println("ft"); 
  Serial.print("Time UTC  : ");
  Serial.print(gps.time.hour());                       // GPS time UTC 
  Serial.print(":");
  Serial.print(gps.time.minute());                     // Minutes
  Serial.print(":");
  Serial.println(gps.time.second());                   // Seconds
  Serial.print("Heading   : ");
  Serial.println(gps.course.deg());
  Serial.print("Speed     : ");
  Serial.println(gps.speed.mph());
  
  unsigned long Distance_To_Home = (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),Home_LAT, Home_LNG);
  Serial.print("KM to Home: ");                        // Have TinyGPS Calculate distance to home and display it
  Serial.println(Distance_To_Home);
  
  delay(200); 
  
  smartDelay(500);                                      // Run Procedure smartDelay

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println("No GPS data received: check wiring!!!");
    */

    // if contact lost for more than half second
  if(millis() - lastCmdTime > 500) {  
    for(int i=0; i<chCount; i++) {
      // set all values to middle
      servoCh[i].writeMicroseconds( (usMin + usMax)/2 );
      servoCh[i].detach(); // stop PWM signals
    }
  }

  if(!client.connected()) {
    client = server.available();
    return;
  }

  // here we have a connected client
  if(client.available()) {
    char c = (char)client.read(); // read char from client (RoboRemo app)

    if(c=='\n') { // if it is command ending
      cmd[cmdIndex] = 0;
      exeCmd();  // execute the command
      cmdIndex = 0; // reset the cmdIndex
    } else {      
      cmd[cmdIndex] = c; // add to the cmd buffer
      if(cmdIndex<99) cmdIndex++;
    }
  } 

  if(millis() - aliveSentTime > 500) { // every 500ms
    Serial.println("alive 1\n");
    client.write("alive 1\n");
    // send the alibe signal, so the "connected" LED in RoboRemo will stay ON
    // (the LED must have the id set to "alive")
    
    aliveSentTime = millis();
    // if the connection is lost, the RoboRemo will not receive the alive signal anymore,
    // and the LED will turn off (because it has the "on timeout" set to 700 (ms) )
  }
}

static void smartDelay(unsigned long ms){               // This custom version of delay() ensures that the gps object is being "fed".
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
