//Teensy based controller WiFi interface ESP8266
// I2C communication with Accelerometer MMA8653FC

 Reading MMA8653FC accelerometer and establishing I2C using wirelibrary support from Arduino.

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

