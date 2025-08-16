#include "Adafruit_VL53L0X.h"
#include<QuadEncoder.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;




// address we will assign if multi sensor is present

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x32
#define LOX3_ADDRESS 0x35
// #define LOX4_ADDRESS 0x38


// set the pins to shutdown
#define SHT_LOX1 37
#define SHT_LOX2 38
#define SHT_LOX3 39
// #define SHT_LOX4 40

#define OUTPUT_READABLE_YAWPITCHROLL // mpu 



// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
// Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();



// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
// VL53L0X_RangingMeasurementData_t measure4;

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */


//Motor control pins
const int M1A = 6;  // IN1 motor 1
const int M1B = 7;  // IN2 motor 1
const int M2A = 8;  // IN1 motor 2
const int M2B = 9;  // IN2 motor 2 


const int myEnc1PhaseA = 0;
const int myEnc1PhaseB = 1;
const int myEnc2PhaseA = 2;
const int myEnc2PhaseB = 3;


int previousMillis = 0;
int currentMillis = 0;


// Declare encoder globally so it can be used in loop
QuadEncoder myEnc1(1, myEnc1PhaseA, myEnc1PhaseB);
QuadEncoder myEnc2(2, myEnc2PhaseA, myEnc2PhaseB);



// mpu variables 
int const INTERRUPT_PIN = 2;  // Define the interruption #0 pin

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer
bool blinkState;

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector


/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady()
{
  MPUInterrupt = true;

}





void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  // digitalWrite(SHT_LOX4, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  // digitalWrite(SHT_LOX4, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2 and LOX3
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  // digitalWrite(SHT_LOX4, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX3
  if(!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while(1);
  }
  delay(10);

  //  // activating LOX4
  // digitalWrite(SHT_LOX4, HIGH);
  // delay(10);

  // //initing LOX4
  // if(!lox4.begin(LOX4_ADDRESS)) {
  //   Serial.println(F("Failed to boot fourth VL53L0X"));
  //   //while(1);
  // }
  // delay(10);
}

void read_multi_sensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false);// pass in 'true' to get debug data printout!
  lox3.rangingTest(&measure3, false);// pass in 'true' to get debug data printout!
  // lox4.rangingTest(&measure4, false);// pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print(F("1: "));
  if(measure1.RangeStatus != 4) {     // if not out of range
    Serial.print(measure1.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if(measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.print(F(" "));

  // print sensor three reading
  Serial.print(F("3: "));
  if(measure3.RangeStatus != 4) {
    Serial.print(measure3.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }

  // // print sensor fourth reading
  // Serial.print(F("4: "));
  // if(measure4.RangeStatus != 4) {
  //   Serial.println(measure4.RangeMilliMeter);
  // } else {
  //   Serial.print(F("Out of range"));
  // }
  
}


void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }
 
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  // pinMode(SHT_LOX4, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  // digitalWrite(SHT_LOX4, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));
  
  
  Serial.println(F("Starting..."));
  setID();


  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);

  myEnc1.write(0);
  myEnc1.init();
  myEnc2.write(0);
  myEnc2.init();

  // mpu part 
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
   Wire.setSDA(17);
   Wire.setSCL(16);
   Wire.begin();
    
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

   while (!Serial);

  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    //while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }

   /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0.15);
  mpu.setYGyroOffset(0.2);
  mpu.setZGyroOffset(0);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0)
  {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  } 
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
 pinMode(LED_BUILTIN, OUTPUT);
}


 

void loop()
{
  
  currentMillis = millis();

  if((currentMillis - previousMillis)>10){

    read_multi_sensors();


  previousMillis = currentMillis;

  }

 int forwardTicks1 = myEnc1.read();
  Serial.print(" Motor_1_forwardTicks = ");
  Serial.print(forwardTicks1);

  int forwardTicks2 = myEnc2.read();
  Serial.print(" Motor_2_forwardTicks = ");
  Serial.println(forwardTicks2);
  
   //mpu 
   if (!DMPReady) return; // Stop the program if DMP programming fails.
    
  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer))
   { // Get the Latest packet 
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      /* Display Euler angles in degrees */
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] ); 
      Serial.print("\t");
      Serial.print(ypr[1] );
      Serial.print("\t");
      Serial.println(ypr[2] );
    #endif

    /* Blink LED to indicate activity */
  blinkState = !blinkState;
  digitalWrite(LED_BUILTIN, blinkState);
  }

}