/*****************************************************
Arduino Pin Defines
*****************************************************/
#define Receiver_Pin 2      //Defines where the radio PPM SUM is connected to

/*****************************************************
Radio Parsing Defines
*****************************************************/
#define Radio_Min 1125      //Minimum duration of raw radio pulse
#define Radio_Max 1860      //Maximum duration of raw radio pulse
#define Throt_Min 0         //Minimum value of parsed throttle
#define Throt_Max 1000      //Maximum value of parsed throttle
#define Parse_Min -500      //Minimum value of parsed radio
#define Parse_Max 500       //Maximum value of parsed radio

#define CH1 0    //defines CH number to numeric values for easy code reading
#define CH2 1
#define CH3 2
#define CH4 3
#define CH5 4
#define CH6 5

/*****************************************************
MPU Parsing Defines
*****************************************************/
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/*****************************************************
Global Variables
*****************************************************/
int radio[8];  //array for storing raw radio input data
int parsed_radio[6];  //array for storing parsed radio data

int16_t ax, ay, az;
int16_t gx, gy, gz;

float acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;
float base_ax, base_ay, base_az, base_gx, base_gy, base_gz;
float acc_angle_x, acc_angle_y, acc_angle_z;
float gyro_angle_x, gyro_angle_y, gyro_angle_z;
unsigned long last_read_time;
float last_x, last_y, last_z;
float dt, t_now;
/*****************************************************
Classes
*****************************************************/
MPU6050 accelgyro;

void setup()
{
  Serial.begin(115200);        //begins serial data stream
  Serial.println("ready");     //I am ready!
  radio_init();                 //Initializes radio input codes
  MPU_Init();
}

void loop()
{
  MPU_Getdata();
  parse();          //Parses raw radio data
  serial_radio();   //Shows serial data to PC
  serial_mpu();
  serial_deg();
  delay(100);  //wait wait wait
}

void serial_mpu()
{
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
}

void serial_deg()
{
    Serial.print("x y z:\t");
    Serial.print(last_x*180.0/3.1416); Serial.print("\t");
    Serial.print(last_y*180.0/3.1416); Serial.print("\t");
    Serial.println(last_z*180.0/3.1416);
}



void calib()
{
  int samples = 10;
  float acc_x_calib = 0;
  float acc_y_calib = 0;
  float acc_z_calib = 0;
  float gyro_x_calib = 0;
  float gyro_y_calib = 0;
  float gyro_z_calib = 0;
  int16_t Ax, Ay, Az, Gx, Gy, Gz;
  for(int i=0; i<samples;i++)
  {
    accelgyro.getMotion6(&Ax, &Ay, &Az, &Gx, &Gy, &Gz);
    acc_x_calib += float(Ax);
    acc_y_calib += float(Ay);
    acc_z_calib += float(Az);
    gyro_x_calib += float(Gx);
    gyro_y_calib += float(Gy);
    gyro_z_calib += float(Gz);
    delay(50);
  }
  base_ax = acc_x_calib/samples;
  base_ay = acc_y_calib/samples;
  base_az = acc_z_calib/samples;
  base_gx = gyro_x_calib/samples;
  base_gy = gyro_y_calib/samples;
  base_gz = gyro_z_calib/samples;
}

void set_last_read(unsigned long time, float x, float y, float z) 
{
  last_read_time = time;
  last_x = x;
  last_y = y;
  last_z = z; 
}

inline unsigned long get_last_time() { return last_read_time;}
inline float get_last_x() { return last_x; }
inline float get_last_y() { return last_y; }
inline float get_last_z() { return last_z; }

void MPU_Getdata()
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    unsigned long t_now = millis();
    
    float acc_x = float(ax)/16384;
    float acc_y = float(ay)/16384;
    float acc_z = float(az)/16384;
        
    float gyro_x = (float(gx) - base_gx)/131;
    float gyro_y = (float(gy) - base_gy)/131;
    float gyro_z = (float(gz) - base_gz)/131;
    
    //filter gyro
    float dt = (t_now - get_last_time())/1000.0;
    float gyro_angle_x = gyro_x*dt + get_last_x();
    float gyro_angle_y = gyro_y*dt + get_last_y();
    float gyro_angle_z = gyro_z*dt + get_last_z(); 
    
    //Complementary
    float alpha = 0.96;
    float angle_x = alpha*gyro_angle_x + (1 - alpha)*acc_angle_x;
    float angle_y = alpha*gyro_angle_y + (1 - alpha)*acc_angle_y;
    float angle_z = gyro_angle_z;
    
    set_last_read(t_now, angle_x, angle_y, angle_z);
}

void MPU_Init(){
 // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    
    calib();
    set_last_read(millis(), 0, 0, 0);

    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */
}


void parse()  //takes data from raw radio data and parse them into usable form and saves them on parsed_radio
{
   parsed_radio[CH1] = map(radio[CH1], Radio_Min, Radio_Max, Parse_Min, Parse_Max);
   parsed_radio[CH2] = map(radio[CH2], Radio_Min, Radio_Max, Parse_Min, Parse_Max);
   parsed_radio[CH3] = map(radio[CH3], Radio_Min, Radio_Max, Throt_Min, Throt_Max);
   parsed_radio[CH4] = map(radio[CH4], Radio_Min, Radio_Max, Parse_Min, Parse_Max);
   parsed_radio[CH5] = map(radio[CH5], Radio_Min, Radio_Max, Parse_Min, Parse_Max);
   parsed_radio[CH6] = map(radio[CH6], Radio_Min, Radio_Max, Throt_Min, Throt_Max);
}

void serial_radio()     //Outputs Parsed Radio data to Serial
{
  for(int CH = 0; CH < 6; CH++)
  {
    Serial.print("CH");
    Serial.print(CH+1);
    Serial.print(": ");
    Serial.print(parsed_radio[CH]);
    Serial.print("  ");
  }
  Serial.println("");
}


/*****************************************************
Radio Receiving Code has been forked from
http://code.google.com/p/read-any-ppm/
*****************************************************/

void radio_init()
{
  pinMode(Receiver_Pin, INPUT);
  attachInterrupt(Receiver_Pin - 2, read_ppm, CHANGE);
  TCCR1A = 0;  //reset timer1
  TCCR1B = 0;
  TCCR1B |= (1 << CS11);  //set timer1 to increment every 0,5 us
}

void read_ppm(){  //leave this alone
  static unsigned int pulse;
  static unsigned long counter;
  static byte channel;

  counter = TCNT1;
  TCNT1 = 0;

  if(counter < 1020){  //must be a pulse if less than 510us
    pulse = counter;
  }
  else if(counter > 3820){  //sync pulses over 1910us
    channel = 0;
  }
  else{  //servo values between 510us and 2420us will end up here
    radio[channel] = (counter + pulse)/2;
    channel++;
  }
}


