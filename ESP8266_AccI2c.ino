
//
// Teensy based controllerWiFi interface ESP8266
// I2C communication with Accelerometer MMA8653FC
// Interrupt driven with Tint= 
//  May 2016
// Authors: Vindhya
//

#include <i2c_t3.h>
#define USB_Serial Serial
#define WiFi_Serial Serial1
#define ACC_SETUP_DELAY 2000 //Accelerometer Initialization wait
// Create an IntervalTimer object
IntervalTimer myTimer;
// Timer ISR definitions
#define MS_INT 20   // millisecond 
#define TICKS_PER_SEC 50 // Ticks per second for TickCounter
// Give credentials to network
#define ssid "Your SSID" // SSID
#define pass "Password!" // PASSWORD
#define DST_IP "Server_IP_ADDRESS" //server, replace with yours

// Accelerometer MMA8653FC definitions
#define I2C_ADDRESS_ACC 0x1D // address of Acc
#define OUT_X_MSB 0x01  // X Component MSB address 
#define OUT_X_LSB 0x02  // X Component LSB address 
#define OUT_Y_MSB 0x03  // Y Component MSB address 
#define OUT_Y_LSB 0x04  // Y Component LSB address 
#define OUT_Z_MSB 0x05  // Z Component MSB address  
#define OUT_Z_LSB 0x06  // Z Component LSB address 
#define F_READ   0x2A // Data format is limited to single byte
#define DATA_STATUS 0x00 // Data Status of XYZ components
#define XYZ_DATA_CFG 0x0E //configuration register
#define SYSMODE 0x0B // System Mode/ Read register
#define OFF_X 0x2F // Offset address for X Component
#define OFF_Y 0x30 // Offset address for Y Component
#define OFF_Z 0x31 // Offset address for Z Component
#define CTRL_REG1 0x2A // Control Register to activate the Speed of measurements
#define ctrl_reg_address 0x00 // to understand why 0x00 and not 0x01, look at the data-sheet p.19 or on the comments of the sub. 
                              //This is valid only becaus we use auto-increment

#define SEC_PER_EVENT 59
#define MAX_WIFI_DELAY 10 // Max WiFi Delay

int AccX, AccY, AccZ; // accelerometer components
float fAccX, fAccY, fAccZ; // float accelerometer components

String serverLoginCredentials = ""//" Your credentials"; // Login Creditentials To server for data storage

// use volatile for shared variables
volatile unsigned long TickCounter = 0;//Counting interrupts during 1second period
unsigned long msTimer = 0; // milli second timer
unsigned long secTimer = 0; // counting seconds from the start of the program
unsigned long transaction_time = 0; // keep count of transaction time
unsigned long timer_transaction = 0; // keep count of transaction time
unsigned long WiFi_ready_wait=0;
unsigned long ts;
unsigned long t_delay; // delay for recovery from LPM
unsigned long tsTransmit = 0; // scheduling transmit time
volatile int timerFlag = 0; // volatile synchronization flag

// Substrings definitions for extracting data from server response
String authenticate_cmd;String WriteDB_cmd;
static boolean ledState;


// WiFi messages definitions
char c;
String WiFiMsgbuffer;String read_buffer; 
String cmd; String authenticationBuf;String measurements;

// Pin Definitions
#define WIFI_CE 2
#define WIFI_RST 3

// srvComState of WiFi
enum srvStates{
  INIT,
  WiFi_INIT,
  WiFi_ready,
  HW_WiFi_RST,
  Check_AP_Connection,
  W_Check_AP,
  Setup_AP_Connectivity,
  W_Setup_AP,
  Change_Default_AP,
  Check_TCP_Connection,
  W_Check_TCP,
  Setup_TCP_Connectivity,
  W_Setup_TCP,
  Check_Token_expiretime,
  Write_Authenticate_CMD_length,
  W_AuthenticateCMDL_ACK,
  Authentication,
  W_Authentication,
  
 };

// initialize server state 
srvStates srvComState = INIT;

//*************************
void setup(void) {

 pinMode(LED_BUILTIN, OUTPUT); // inbuilt LED pin OUTPUT
 pinMode(WIFI_CE, OUTPUT);  //  WiFi Chip enable 
 pinMode(WIFI_RST, OUTPUT); // WiFi hardware RST pin 
 digitalWrite(WIFI_CE, HIGH); // Set CE high for communication with ESP8266
 WiFi_HardReset(); // Hardware RESET ESP8266 module to establish communication
 Serial1.begin(115200); // hardware serial connects to esp8266 module
 Serial.begin(115200); // usb serial connects to to pc
 myTimer.begin(timerISR, MS_INT * 1000); // run timer interrupt every 20 ms (Fs=50Hz)
 ledState = LOW; // Initialize state of LED
 //Start of I2C communication of MMA8653FC 3-Axis accelerometer
  Serial.println("Initializing I2C Communication to MMA8653FC ");
  // I2C setup for Master mode, pins 18/19, internal pullups, 400kHz
  Wire.begin(I2C_MASTER, I2C_ADDRESS_ACC , I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
  Wire.beginTransmission(I2C_ADDRESS_ACC);
  Wire.send(1);  // register to read
  Wire.endTransmission();
  ACC_INIT(); // Accelerometer Initialization
  delay(ACC_SETUP_DELAY);//wait for usb serial enumeration on 'Serial' & device startup
}

// Timer ISR

void timerISR(void) {
  toggleLED();
  timerFlag = true;
  TickCounter++;
  if ( TickCounter > TICKS_PER_SEC) {
    secTimer++;
    ts++;  // local tsTimer
    msTimer = 0;
    TickCounter = 0;

  } // end second processing
  else {
    msTimer += MS_INT;
  } // end ms processing
} // end timerISR


//  toggle LED
void toggleLED(){
     
     // if the LED is off turn it on and vice-versa:
     if (ledState == LOW)
       ledState = HIGH;
     else
       ledState = LOW;
       
     // set the LED with the ledState of the variable:
     //digitalWrite(LED_BUILTIN, ledState); 
  }
  
//******** The main program 
void loop(void) {

  
  if (timerFlag) {  // interrupt generated in the meantime
    toggleLED(); // LED indicate start of processing
    timerFlag = false;
    // read Accelerometer 
    readAcc();
    Serial.print("AccX:");
    Serial.print(fAccX);
    Serial.print(" ");
    Serial.print("AccY:");
    Serial.print(" ");
    Serial.print(fAccY);
    Serial.print(" ");
    Serial.print("AccZ:");
    Serial.println(fAccZ);
    
  }
  
  getWiFiMsg(); // Store all the message
  switch( srvComState){
   case INIT:
      //   Wait for ready response from ESP8266 to continue with further communication
    if(WiFiMsgbuffer.indexOf("ready") > 0){
      Serial.println(WiFiMsgbuffer);
      WiFiMsgbuffer=""; //clear WiFiMsg buffer in every state
      srvComState = WiFi_INIT;// if module is responding go ahead to do to software RESET
    }
    //   Timeout
    if(ts > WiFi_ready_wait){
      Serial.println(WiFiMsgbuffer);
      WiFiMsgbuffer=""; //clear WiFiMsg buffer in every state
      //Reset unsuccesful, Hardware RESET module and try again
      srvComState = HW_WiFi_RST;
    }
    break; // end of INIT

   case WiFi_INIT: 
    
    reset_WiFi(); // Reset module send AT command 
    // Initialize max wait_time to wait for response
    WiFi_ready_wait = ts + MAX_WIFI_DELAY;
    srvComState = WiFi_ready;// if module is responding go ahead to do to software RESET
       
    break; // end of INIT
     
   case WiFi_ready:
    // Wait till WiFi Responsed OK after Reset 
    
    if(WiFiMsgbuffer.indexOf("OK") > 0){
      Serial.println(WiFiMsgbuffer);
      WiFiMsgbuffer=""; //clear WiFiMsg buffer in every state
      //Reset succesful, continue to CheckWiFi()     
      WiFi_ready_wait = ts + MAX_WIFI_DELAY;  // Initialize max wait_time to wait for response in next state
      srvComState = Check_AP_Connection;
    }

    //  Time outs
    if(ts > WiFi_ready_wait){
      Serial.println(WiFiMsgbuffer);
      WiFiMsgbuffer=""; //clear WiFiMsg buffer in every state
      //Reset unsuccesful, Hardware RESET module and try again
      srvComState = HW_WiFi_RST;
    }
    
    break; // end of WiFi_ready
      
    case HW_WiFi_RST:
     // Hardware Reset module and wait for responses;
      WiFi_HardReset();
      WiFi_ready_wait = ts + MAX_WIFI_DELAY;  // Initialize max wait_time to wait for response in next state
      srvComState = INIT;

    break; // end of HW_WiFi_RST

    case Check_AP_Connection:
    // Send AT command to Check AP Connection 
      CheckWiFi(); 
      WiFi_ready_wait = ts + MAX_WIFI_DELAY; // Initialize max wait_time to wait for response in next state
      srvComState = W_Check_AP;

    break; // end of Check_AP_Connection

    case W_Check_AP:
    // wait for response to check WiFi connectivity
    if ((WiFiMsgbuffer.indexOf("No AP")>0)&&(WiFiMsgbuffer.indexOf("OK")>0)) {
      //NO AP, continue with the setup to AP
      Serial.println(WiFiMsgbuffer);
      WiFiMsgbuffer=""; //clear WiFiMsg buffer in every state
      WiFi_ready_wait = ts + MAX_WIFI_DELAY;  // Initialize max wait_time to wait for response in next state
      srvComState = Setup_AP_Connectivity; // go to setup AP if No AP
    }
    
    if ((WiFiMsgbuffer.indexOf(ssid)>0)&&(WiFiMsgbuffer.indexOf("OK")>0)) {
      // AP still active procced to TCP connection
      Serial.println(WiFiMsgbuffer);
      WiFiMsgbuffer="";
      // Initialize max wait_time to wait for response in next state
      WiFi_ready_wait = ts + MAX_WIFI_DELAY;
      srvComState = Check_TCP_Connection;
    }

    //   Timeout
    if(ts > WiFi_ready_wait){
      Serial.println(WiFiMsgbuffer);
      WiFiMsgbuffer=""; //clear WiFiMsg buffer in every state
      //Reset unsuccesful, Hardware RESET module and try again
      srvComState = HW_WiFi_RST;
    }
    
    break; // end of W_Check_AP
     
    case Setup_AP_Connectivity:
      // Send AT command to setup with SSID & PASSWORD
      setupWiFi(); // setup wifi sending AT commands 
      srvComState = W_Setup_AP;
      
      break; // end of Setup_AP_Connectivity

    case W_Setup_AP:
      // wait for response from WiFi after Setup
     if(WiFiMsgbuffer.indexOf("WIFI GOT IP")> 0){
        Serial.println(WiFiMsgbuffer);
        WiFiMsgbuffer=""; //clear WiFiMsg buffer in every state
        srvComState = Check_TCP_Connection;
      }

      // if ERROR change credentials 
      if(WiFiMsgbuffer.indexOf("ERROR")> 0){
        Serial.println(WiFiMsgbuffer);
        WiFiMsgbuffer=""; //clear WiFiMsg buffer in every state
        srvComState = Change_Default_AP;
      }

      //   Timeout
       if(ts > WiFi_ready_wait){
        Serial.println(WiFiMsgbuffer);
        WiFiMsgbuffer=""; //clear WiFiMsg buffer in every state
        //Reset unsuccesful, Hardware RESET module and try again
        srvComState = HW_WiFi_RST;
      }
      break; // end of state W_Setup_AP

    case Change_Default_AP:
    //     ssid = 
   //      pass = 
      srvComState = Check_AP_Connection;

      break; // end of state Change_Default_AP

    case Check_TCP_Connection: 
      // send AT command to check TCP connection status
      checkServer(); // connected to WiFi, Check if connection with server is still active
      WiFi_ready_wait = ts + MAX_WIFI_DELAY; // Initialize max wait_time to wait for response in next state
      srvComState = W_Check_TCP;

    break; // end of Check_TCP_Connection

    case W_Check_TCP:
    // wait for response from module to check TCP connection
    
    // 3- Indicating Connection is Active
      if((WiFiMsgbuffer.indexOf("3")> 0)&&(WiFiMsgbuffer.indexOf("OK")>0)){
        Serial.println(WiFiMsgbuffer);
        WiFiMsgbuffer=""; //clear WiFiMsg buffer in every state
        WiFi_ready_wait = ts + MAX_WIFI_DELAY;  // Initialize max wait_time to wait for response in next state
        // After Connection established start GET/POST requests based on server requirements 
      }
      
      // 2- Indicating WIFI GOT IP and go ahead and setup TCP
      if((WiFiMsgbuffer.indexOf("2")> 0)&&(WiFiMsgbuffer.indexOf("OK")>0)) {
        // connection with TCP is not active
        Serial.println(WiFiMsgbuffer);
        WiFiMsgbuffer=""; // clear WiFiMsg buffer in every state
        WiFi_ready_wait = ts + MAX_WIFI_DELAY; // Initialize max wait_time to wait for response in next state
        srvComState = Setup_TCP_Connectivity;
      }
      
      // 4- Indicating TCP connection established previously is Disconnected
      if((WiFiMsgbuffer.indexOf("4")> 0) &&(WiFiMsgbuffer.indexOf("OK")>0)) {
        // connection with TCP is not active
        Serial.println(WiFiMsgbuffer);
        WiFiMsgbuffer=""; // clear WiFiMsg buffer in every state
        WiFi_ready_wait = ts + MAX_WIFI_DELAY; // Initialize max wait_time to wait for response in next state
        srvComState = Setup_TCP_Connectivity;
      }

      //   Timeout
      if(ts > WiFi_ready_wait){
        Serial.println(WiFiMsgbuffer);
        WiFiMsgbuffer=""; //clear WiFiMsg buffer in every state
        //Reset unsuccesful, Hardware RESET module and try again
        srvComState = HW_WiFi_RST;
      }
      break; // end of W_Check_AP
    
    case Setup_TCP_Connectivity:
      // Setup TCP connection with AT commands
      connect2server(); // Connect to server, estabhlish TCP channel;
      WiFi_ready_wait = ts + MAX_WIFI_DELAY; // Initialize max wait_time to wait for response in next state
      srvComState = W_Setup_TCP; // wait for response in next state

    break; // end of Setup_TCP_Connectivity

    case W_Setup_TCP:
    // wait for TCP setup response 
    
      // TCP CONNECTION established
      if((WiFiMsgbuffer.indexOf("CONNECT") > 0)&& (WiFiMsgbuffer.indexOf("OK") > 0)){
        Serial.println(WiFiMsgbuffer);
        WiFiMsgbuffer=""; //clear WiFiMsg buffer in every state
        // Initialize max wait_time to wait for response in next state
        WiFi_ready_wait = ts + MAX_WIFI_DELAY;
        srvComState = Check_Token_expiretime;
      }
      
     // Failed to establish connection
      if(WiFiMsgbuffer.indexOf("ERROR") > 0) {
        Serial.println(WiFiMsgbuffer);
        WiFiMsgbuffer=""; //clear WiFiMsg buffer in every state
        // Initialize max wait_time to wait for response in next state
        WiFi_ready_wait = ts + MAX_WIFI_DELAY;
        srvComState = Check_AP_Connection;
      }

        //   Timeout
      if(ts > WiFi_ready_wait){
        Serial.println(WiFiMsgbuffer);
        WiFiMsgbuffer=""; //clear WiFiMsg buffer in every state
        //Reset unsuccesful, Hardware RESET module and try again
        srvComState = HW_WiFi_RST;
      }
      break; // end of state W_Setup_TCP

   
    default: 
      // Initialize max wait_time to wait for response in next state
      WiFi_ready_wait = ts + MAX_WIFI_DELAY;
      srvComState = INIT;
    break;
  }// end of switch case


       
  
 
} // end loop

//////*****************Storing response of Wifi***********************////////////
void getWiFiMsg(void) {
  while (WiFi_Serial.available()) //new cahracters received?
  {
    c = WiFi_Serial.read(); //print to console
    WiFiMsgbuffer += c;//Serial.write(c);
  }
}// end of getWiFiMSg

//reset to give new credentials
void reset_WiFi(void)
{
  cmd = "AT+RST"; 
  Serial1.println(cmd); // Send SOftware RESET command to module
  //Serial.println(cmd);
 
}

/*************************Hardware WiFi Reset*************************/
void WiFi_HardReset(void) {
  digitalWrite(WIFI_RST, LOW);
  delay(1);
  digitalWrite(WIFI_RST, HIGH);
}
/********************** setupWiFi******************************/
void setupWiFi(void)
{
  cmd = "AT+CWJAP=\""; //form eg: AT+CWJAP="SSID","Password"
  cmd += ssid;
  cmd += "\",\"";
  cmd += pass;
  cmd += "\"";

  Serial1.println(cmd); // send command to WiFi module
  //Serial.println(cmd);
 
}

//************Setup Wifi With credentials and check if connected************///
void CheckWiFi(void)
{
  cmd = "AT+CWJAP?"; //form eg: AT+CWJAP="SSID","Password"
  Serial1.println(cmd);
  //Serial.println(cmd);
} // end of CheckWifi

//***************Check TCP connection status****************************///
void checkServer(void)
{
  cmd = "AT+CIPSTATUS"; //form eg: AT+CWJAP="SSID","Password"
  Serial1.println(cmd);
  //Serial.println(cmd);
  
}

//**** Establish TCP/IP connection
void connect2server(void){
  cmd = "AT+CIPSTART=\"TCP\",\"";  //make this command: AT+CPISTART="TCP","146.227.57.195",80
  cmd += DST_IP;
  cmd += "\",80";
  Serial1.println(cmd);  //send command to device
  //Serial.println(cmd);  //send command to device
} //*** end establishing server connection


//********Putting WiFi to deep sleep mode
void WiFi_deep_sleep(void){
  cmd = "AT+GSLP=57000"; // Put WiFi to sleep mode for 57000 
  Serial1.println(cmd);
}

// Procedure to send length of POST/GET request
//********** Send Command length
 void authenticate_cmd_length(String serverLoginCredentials){
  authenticate_cmd = "POST /smartcup/authentication/login HTTP/1.1\r\n";
  authenticate_cmd  += "Host: telerehab.us-rnd.com\r\n";
  authenticate_cmd  += "Content-Type: application/json\r\n";
  authenticate_cmd  += "Content-Length: ";
  authenticate_cmd  += serverLoginCredentials.length();
  authenticate_cmd  += "\r\n\r\n";
  authenticate_cmd  += serverLoginCredentials;
  authenticate_cmd  += "\r\n";
  Serial1.print("AT+CIPSEND="); //esp8266 needs to know message length of incoming message - .length provides this
  Serial1.println(authenticate_cmd.length());
 } // end of authenticate cmd length

/*************************Server Authentication **************************/

void authenticate(void) {
    Serial1.println(authenticate_cmd);
  //  Serial.println(authenticate_cmd);
} // end of authentication
  
//************************8ACCELEROMETER INITIALIZATION**********************************//
void ACC_INIT() {
  writeRegister(CTRL_REG1, 0x00 ); // Put it in off mode
  delay(50);
  writeRegister(XYZ_DATA_CFG, 0x00); //Configure acceleromter +-2g // 2G full range mode
  //0x01 for +-4g //0x02for +-8g
  writeRegister(CTRL_REG1, 0x01 ); // Put it in active mode for measurements
}// end of accelerometer initilization

//****************** Read current values from ACCELEROMETER in milli-Gs*****************//
void readAcc(void)
{

  Wire.beginTransmission (I2C_ADDRESS_ACC);    //= ST + (Device Adress+W(0)) + wait for ACK
  Wire.write (ctrl_reg_address);    // store the register to read in the buffer of the wire library
  Wire.endTransmission (I2C_NOSTOP, 1000); // actually send the data on the bus -note: returns 0 if transmission OK-
  delayMicroseconds (2);            //
  Wire.requestFrom (I2C_ADDRESS_ACC, 7);    // read a number of byte and store them in wire.read (note: by nature, this is called an "auto-increment register adress")

  Wire.read();   // discard first byte (status)

  // read six bytes
  // note:  2G is 32768, -2G is -32768

  AccX = ((int16_t)(((unsigned char)Wire.read() << 8) | (unsigned char)Wire.read()) * 1000) / (32768 / 2); // MSB first
  AccY = ((int16_t)(((unsigned char)Wire.read() << 8) | (unsigned char)Wire.read()) * 1000) / (32768 / 2); // MSB first
  AccZ = ((int16_t)(((unsigned char)Wire.read() << 8) | (unsigned char)Wire.read()) * 1000) / (32768 / 2); // MSB first
    fAccX = (float)(AccX) / (1000);
    fAccY = (float)(AccY) / (1000);
    fAccZ = (float)(AccZ) / (1000);
}  // MMA8653FC_read()

//Write data to a register on capacative sensor connected over I2C
void writeRegister(unsigned char r, unsigned char v)
{
  Wire.beginTransmission(I2C_ADDRESS_ACC);
  Wire.send(r);
  Wire.send(v);
  Wire.endTransmission();
}
