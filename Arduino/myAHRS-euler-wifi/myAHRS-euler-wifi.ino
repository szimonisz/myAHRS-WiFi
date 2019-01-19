#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define MYAHRS_I2C_ADDRESS           0x20
#define MYMOTION_WHO_AM_I_VALUE      0xB1
#define I2CAddressESPWifi 8

WiFiUDP Udp;
// WiFi Setup
char* wifiSSID = "INSERT SSID HERE";
char* wifiPassword = "INSERT WIFI PASSWORD HERE";

// Designated UDP port
unsigned int localUdpPort = 4210;

// UDP message buffer
char message[256];

// The SDA and SCL pins of the Mini D1's I2C bus (D2,D1 respectively)
const int sdaPin = D2;
const int sclPin = D1;

// Sets the refresh rate of the sensor data read (in ms) through I2C
// myAHRS+ is capable of up to 1khz I2C 
// (Set to 10hz by default)
// Essentially just sets the main program's loop delay
const int refreshRateMilliseconds = 100; //100ms -> 10hz

// myAHRS_plus register map
// I2C register mapping (provided by myAHRS+ sample code)
enum {
    I2C_SLAVE_REG_WHO_AM_I = 0x01,
    I2C_SLAVE_REG_REV_ID_MAJOR,
    I2C_SLAVE_REG_REV_ID_MINOR,
    I2C_SLAVE_REG_STATUS,

    // RAW DATA
    I2C_SLAVE_REG_I_ACC_X_LOW = 0x10,
    I2C_SLAVE_REG_I_ACC_X_HIGH,
    I2C_SLAVE_REG_I_ACC_Y_LOW,
    I2C_SLAVE_REG_I_ACC_Y_HIGH,
    I2C_SLAVE_REG_I_ACC_Z_LOW,
    I2C_SLAVE_REG_I_ACC_Z_HIGH,
    I2C_SLAVE_REG_I_GYRO_X_LOW,
    I2C_SLAVE_REG_I_GYRO_X_HIGH,
    I2C_SLAVE_REG_I_GYRO_Y_LOW,
    I2C_SLAVE_REG_I_GYRO_Y_HIGH,
    I2C_SLAVE_REG_I_GYRO_Z_LOW,
    I2C_SLAVE_REG_I_GYRO_Z_HIGH,
    I2C_SLAVE_REG_I_MAGNET_X_LOW,
    I2C_SLAVE_REG_I_MAGNET_X_HIGH,
    I2C_SLAVE_REG_I_MAGNET_Y_LOW,
    I2C_SLAVE_REG_I_MAGNET_Y_HIGH,
    I2C_SLAVE_REG_I_MAGNET_Z_LOW,
    I2C_SLAVE_REG_I_MAGNET_Z_HIGH,

    // COMPENSATED DATA
    I2C_SLAVE_REG_C_ACC_X_LOW,
    I2C_SLAVE_REG_C_ACC_X_HIGH,
    I2C_SLAVE_REG_C_ACC_Y_LOW,
    I2C_SLAVE_REG_C_ACC_Y_HIGH,
    I2C_SLAVE_REG_C_ACC_Z_LOW,
    I2C_SLAVE_REG_C_ACC_Z_HIGH,
    I2C_SLAVE_REG_C_GYRO_X_LOW,
    I2C_SLAVE_REG_C_GYRO_X_HIGH,
    I2C_SLAVE_REG_C_GYRO_Y_LOW,
    I2C_SLAVE_REG_C_GYRO_Y_HIGH,
    I2C_SLAVE_REG_C_GYRO_Z_LOW,
    I2C_SLAVE_REG_C_GYRO_Z_HIGH,
    I2C_SLAVE_REG_C_MAGNET_X_LOW,
    I2C_SLAVE_REG_C_MAGNET_X_HIGH,
    I2C_SLAVE_REG_C_MAGNET_Y_LOW,
    I2C_SLAVE_REG_C_MAGNET_Y_HIGH,
    I2C_SLAVE_REG_C_MAGNET_Z_LOW,
    I2C_SLAVE_REG_C_MAGNET_Z_HIGH,
    I2C_SLAVE_REG_C_TEMPERATURE_LOW,
    I2C_SLAVE_REG_C_TEMPERATURE_HIGH,

    // Attitude - Euler angle
    I2C_SLAVE_REG_ROLL_LOW,
    I2C_SLAVE_REG_ROLL_HIGH,
    I2C_SLAVE_REG_PITCH_LOW,
    I2C_SLAVE_REG_PITCH_HIGH,
    I2C_SLAVE_REG_YAW_LOW,
    I2C_SLAVE_REG_YAW_HIGH,
    
    //Attitude - Quaternion
    I2C_SLAVE_REG_QUATERNIAN_X_LOW,
    I2C_SLAVE_REG_QUATERNIAN_X_HIGH,
    I2C_SLAVE_REG_QUATERNIAN_Y_LOW,
    I2C_SLAVE_REG_QUATERNIAN_Y_HIGH,
    I2C_SLAVE_REG_QUATERNIAN_Z_LOW,
    I2C_SLAVE_REG_QUATERNIAN_Z_HIGH,
    I2C_SLAVE_REG_QUATERNIAN_W_LOW,
    I2C_SLAVE_REG_QUATERNIAN_W_HIGH,
};

void setup()
{
  //Initialization of I2C master (D! Mini)
  Wire.begin(sdaPin, sclPin); 

  //Login to WiFi and bind to specified UDP port
  Udp.begin(localUdpPort);
  WiFi.begin(wifiSSID,wifiPassword);

  //infinite loop, waits for WiFi to connect successfully
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
  }
  
  //Send a UDP Packet notifying the user that WiFi connection was a success
  Udp.beginPacket(Udp.remoteIP(),4210);
  Udp.write("Connected!");
  Udp.endPacket();

  
  who_am_i();
  read_rev_id();
  
  delay(1000); //Wait for 1 second.
}


bool sensor_init()
{
  uint8_t buf_whomi[1];
  uint8_t buf_stat[1];
    
  if(read(I2C_SLAVE_REG_WHO_AM_I, buf_whomi, 1) != 0xB1) {
    return false;
  }
  if(read(I2C_SLAVE_REG_STATUS, buf_stat, 1) != 0x80) {
    return false;
  }
  return true;
}

// I2C read function
// Parameter 'buff' will be filled with the sensor data read from the I2C Wire read()
// Returns TRUE if I2C read was successful, FALSE if unsuccessful
// Should only return FALSE due to a bad connection between SDA/SLC pins of master and slave

bool read(uint8_t reg_add, uint8_t* buff , uint8_t len)
{
    Wire.beginTransmission((uint8_t)MYAHRS_I2C_ADDRESS);  
        
    Wire.write(reg_add); 
    
    Wire.endTransmission(false); 

    Wire.requestFrom((uint8_t)MYAHRS_I2C_ADDRESS, len);

    uint8_t cnt = 0;
    while(Wire.available()) {
        buff[cnt++] = Wire.read();
    }
    // if the buffer was filled with expected length of sensor data
    // if false, we know the I2C read was unsuccessful 
    return (cnt == len);
}

// Read I2C raw data from sensor, myAHRS+ sample code
int read_raw_data()
{
  uint8_t buf_raw_data[18];
  
  read(I2C_SLAVE_REG_I_ACC_X_LOW, buf_raw_data, 18);
  
  //Little endian
  int16_t acc_x = (buf_raw_data[1]<<8) | buf_raw_data[0];
  int16_t acc_y = (buf_raw_data[3]<<8) | buf_raw_data[2];
  int16_t acc_z = (buf_raw_data[5]<<8) | buf_raw_data[4];
  int16_t gyro_x = (buf_raw_data[7]<<8) | buf_raw_data[6];
  int16_t gyro_y = (buf_raw_data[9]<<8) | buf_raw_data[8];
  int16_t gyro_z = (buf_raw_data[11]<<8) | buf_raw_data[10];
  int16_t mag_x = (buf_raw_data[13]<<8) | buf_raw_data[12];
  int16_t mag_y = (buf_raw_data[15]<<8) | buf_raw_data[14];
  int16_t mag_z = (buf_raw_data[17]<<8) | buf_raw_data[16];
}

// Read I2C compensated data from sensor, myAHRS+ sample code
int read_compensated_data()
{
  uint8_t buf_comp_data[18];
  
  read(I2C_SLAVE_REG_C_ACC_X_LOW, buf_comp_data, 18);
  
  int16_t acc_c_x = (buf_comp_data[1]<<8) | buf_comp_data[0];
  int16_t acc_c_y = (buf_comp_data[3]<<8) | buf_comp_data[2];
  int16_t acc_c_z = (buf_comp_data[5]<<8) | buf_comp_data[4];
  int16_t gyro_c_x = (buf_comp_data[7]<<8) | buf_comp_data[6];
  int16_t gyro_c_y = (buf_comp_data[9]<<8) | buf_comp_data[8];
  int16_t gyro_c_z = (buf_comp_data[11]<<8) | buf_comp_data[10];
  int16_t mag_c_x = (buf_comp_data[13]<<8) | buf_comp_data[12];
  int16_t mag_c_y = (buf_comp_data[15]<<8) | buf_comp_data[14];
  int16_t mag_c_z = (buf_comp_data[17]<<8) | buf_comp_data[16];
  
  float comp_acc_x = (float)acc_c_x * 16.0 / 32767;
  float comp_acc_y = (float)acc_c_y * 16.0 / 32767;
  float comp_acc_z = (float)acc_c_z * 16.0 / 32767;
  float comp_gyro_x = (float)gyro_c_x * 2000 / 32767;
  float comp_gyro_y = (float)gyro_c_y * 2000 / 32767;
  float comp_gyro_z = (float)gyro_c_z * 2000 / 32767;
  float comp_mag_x = (float)mag_c_x * 0.3;
  float comp_mag_y = (float)mag_c_y * 0.3;
  float comp_mag_z = (float)mag_c_z * 0.3;
}

// EULER ANGLE(RPY)
// Reads Euler Angle data from orientation sensor via I2C protocol
// Formats the bytes read from I2C into proper Euler Roll/Pitch/Yaw
// If the I2C read was successful, it a packet with XYZ will be sent to designated UDP port
int read_euler()
{
  uint8_t buf_euler[6];
  
  bool i2cSuccess = read(I2C_SLAVE_REG_ROLL_LOW, buf_euler, 6);
  
  int16_t euler_x = (buf_euler[1]<<8) | buf_euler[0];
  int16_t euler_y = (buf_euler[3]<<8) | buf_euler[2];
  int16_t euler_z = (buf_euler[5]<<8) | buf_euler[4];

  float roll = (float)euler_x * 180 / 32767;
  float pitch = (float)euler_y * 180 / 32767;
  float yaw = (float)euler_z * 180 / 32767;

  // If sensor data was pulled from I2C successfully
  if(i2cSuccess == true){
    // Send a UDP packet to port 4210 specifying
    Udp.beginPacket(Udp.remoteIP(),localUdpPort);
    sprintf(message,"ROLL: %f\nPITCH: %f\nYAW: %f\n",roll,pitch,yaw);
    Udp.write(message);
    Udp.endPacket();
  }
}

// Read quaternion values from sensor through I2C protocol, myAHRS+ sample code
int read_quat()
{
  uint8_t buf_quat[8];
  
  read(I2C_SLAVE_REG_QUATERNIAN_X_LOW, buf_quat, 8);
  
  int16_t quat_x = (buf_quat[1]<<8) | buf_quat[0];
  int16_t quat_y = (buf_quat[3]<<8) | buf_quat[2];
  int16_t quat_z = (buf_quat[5]<<8) | buf_quat[4];
  int16_t quat_w = (buf_quat[7]<<8) | buf_quat[6];
  
  float quaternion_x = (float)quat_x / 32767;
  float quaternion_y = (float)quat_y / 32767;
  float quaternion_z = (float)quat_z / 32767;
  float quaternion_w = (float)quat_w / 32767;  
}

//READ REVISION ID
int read_rev_id()
{
  uint8_t id_1 = 0;
  uint8_t id_2 = 0;  
  read(I2C_SLAVE_REG_REV_ID_MAJOR, &id_1, 1);
  read(I2C_SLAVE_REG_REV_ID_MAJOR, &id_2, 1);
}

// Grabs and defines I2C address of myAHRS+ 
int who_am_i()
{
  uint8_t whomi = 0;
  read(I2C_SLAVE_REG_WHO_AM_I, &whomi, 1);
}


void loop()
{
  //read_raw_data();
  //read_compensated_data();
  read_euler();
  //read_quat();

  delay(refreshRateMilliseconds); // 10Hz
}
