/*****************************************************
Arduino Pin Defines
*****************************************************/
#define Receiver_Pin 2      //Defines where the radio PPM SUM is connected to
#define Servo_Pin 5      //Defines where the servo is connected to
#define Motor_Pin_One 9      //Defines where the motor is connected to
#define Motor_Pin_Two 10      //Defines where the motor is connected to

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

#define MOT_UP 1
#define MOT_DOWN 0

/*****************************************************
MPU Parsing Defines
Class documentation
http://www.i2cdevlib.com/docs/html/class_m_p_u6050.html

Gyro Readings
FS_SEL | Full Scale Range   | LSB Sensitivity
-------+--------------------+----------------
0      | +/- 250 degrees/s  | 131 LSB/deg/s		(DEFAULT)
1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
3      | +/- 2000 degrees/s | 16.4 LSB/deg/s

AFS_SEL | Full Scale Range | LSB Sensitivity
--------+------------------+----------------
0       | +/- 2g           | 8192 LSB/mg 		(DEFAULT)
1       | +/- 4g           | 4096 LSB/mg
2       | +/- 8g           | 2048 LSB/mg
3       | +/- 16g          | 1024 LSB/mg

*****************************************************/
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Servo.h"
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "Timer.h"

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

float base_ax, base_ay, base_gx, base_gy, base_gz;

long serial_deg_delay;
long kalman_timer;

double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double roll, pitch;

unsigned int rate_counter = 0;

int roll_P = 2;
unsigned long osc_period = 100;
int osc_status;

/*****************************************************
Classes
*****************************************************/
MPU6050 accelgyro;
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
Servo bae;
Timer osc_motor;


void setup()
{
  Serial.begin(115200);        //begins serial data stream
  Serial.println("ready");     //I am ready!
  radio_init();                 //Initializes radio input codes
  MPU_Init();
  kalman_init();
  osc_motor.every(osc_period, motor_osc);
  bae.attach(Servo_Pin, 1000, 2000);
}

void loop()
{
  MPU_Getdata();
  parse();          //Parses raw radio data
  bae_write();
  rate_counter++;
  //serial_radio();   //Shows serial data to PC
  //serial_mpu();
  serial_deg(50);
  osc_motor.update();
  // delay(100);  //wait wait wait
}

void motor_osc()
{
	if (osc_status == MOT_UP)
	{
		digitalWrite(Motor_Pin_One, HIGH);
		digitalWrite(Motor_Pin_Two, LOW);
		osc_status = MOT_DOWN;
	}
	else if (osc_status == MOT_DOWN)
	{
		digitalWrite(Motor_Pin_One, LOW);
		digitalWrite(Motor_Pin_Two, HIGH);
		osc_status = MOT_UP;
	}


}

void bae_write()
{
	bae.write(90 - roll_P * kalAngleX);
}

void serial_mpu() //Writes raw sensor values to serial
{
    Serial.print("a/g:\t");
    Serial.print((float)ax / 16384); Serial.print("\t");
	Serial.print((float)ay / 16384); Serial.print("\t");
	Serial.print((float)az / 16384); Serial.print("\t");
	Serial.print((float)gx / 131); Serial.print("\t");
	Serial.print((float)gy / 131); Serial.print("\t");
	Serial.println((float)gz / 131);
}

void serial_deg()
{
    Serial.print("x y:\t");
	Serial.print(kalAngleX); Serial.print("\t");
	Serial.println(kalAngleY);
}

void serial_deg(int delay)
{
	if (millis() - serial_deg_delay > delay)
	{
		Serial.print("x y:\t");
		Serial.print(kalAngleX); Serial.print("\t");
		Serial.println(kalAngleY);
		serial_deg_delay = millis();
		Serial.print("current rate : ");
		Serial.println(rate_counter);
		rate_counter = 0;
	}
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
    accelgyro.getMotion6(&Ay, &Ax, &Az, &Gy, &Gx, &Gz);
    acc_x_calib += float(Ax);
    acc_y_calib += float(Ay);
    gyro_x_calib += float(Gx);
    gyro_y_calib += float(Gy);
    gyro_z_calib += float(Gz);
    delay(50);
  }
  base_ax = acc_x_calib/samples;
  base_ay = acc_y_calib/samples;
  base_gx = gyro_x_calib/samples;
  base_gy = gyro_y_calib/samples;
  base_gz = gyro_z_calib/samples;
}

void kalman_init()
{
	accelgyro.getMotion6(&ay, &ax, &az, &gy, &gx, &gz);

	float acc_x = float(ax) / 8192;
	float acc_y = float(ay) / 8192;
	float acc_z = float(az) / 8192;

	roll = atan2(acc_y, acc_z) * RAD_TO_DEG;
	pitch = atan(-acc_x / sqrt(acc_y * acc_y + acc_z * acc_z)) * RAD_TO_DEG;

	kalmanX.setAngle(roll); // Set starting angle
	kalmanY.setAngle(pitch);
	kalman_timer = micros();

}

void MPU_Getdata()
{
    accelgyro.getMotion6(&ay, &ax, &az, &gy, &gx, &gz);
    
    float acc_x = float(ax)/8192;
    float acc_y = float(ay)/8192;
    float acc_z = float(az)/8192;
        
    float gyro_x = (float(gx) - base_gx)/131;
    float gyro_y = (float(gy) - base_gy)/131;
    float gyro_z = (float(gz) - base_gz)/131;

	double dt = (double)(micros() - kalman_timer) / 1000000; // Calculate delta time
	kalman_timer = micros();

	roll = atan2(acc_y, acc_z) * RAD_TO_DEG;
	pitch = atan(-acc_x / sqrt(acc_y * acc_y + acc_z * acc_z)) * RAD_TO_DEG;

	if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
		kalmanX.setAngle(roll);
		kalAngleX = roll;
	}
	else
		kalAngleX = kalmanX.getAngle(roll, gyro_x, dt); // Calculate the angle using a Kalman filter

	if (abs(kalAngleX) > 90)
		gyro_y = -gyro_y; // Invert rate, so it fits the restriced accelerometer reading

	kalAngleY = kalmanY.getAngle(pitch, gyro_y, dt);
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


