//Use Nucleo L432KC to interface with BNO-055 and motors for the EW309 Turret
// J. Bradshaw 20230215

#include <Wire.h>
#include <Adafruit_Sensor.h>    // https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_BNO055.h>    // https://github.com/adafruit/Adafruit_BNO055/blob/master/Adafruit_BNO055.h
 // Note that the OPERATION_MODE_NDOF_FMC_OFF is not inside the Adafruit_BNO055 class in the latest version
 // of the library so usage should be:  bno.setMode(OPERATION_MODE_NDOF_FMC_OFF);
#include <utility/imumaths.h>
#include <PID_v1.h>             // https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h

#define CONTROL_LOOP_DELAY_MS 20
#define YAW640_RATIO    (.981746f)      //for some reason, the YAW RADIAN measurement from the BNO
                                        // yeilds 0.0 - 6.40 instead of 2*PI, this scales it accordingly
// Setup digital I/O mapping
#define FIRE_PIN  D3
#define FEED_PIN  D11

#define TO_RAD  0.017453292 //PI/180.0

// BNO-055 I2C Inertial Mearurement Unit
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Timer variables for PID loops
unsigned long pitchTimeLast;
unsigned long yawTimeLast;

//Specify the links and initial tuning parameters
double yawSetpoint, yawInput, yawOutput = 0.0;
volatile float yaw_co = 0.0;
float yaw_temp=0.0;     //make global for now for testing purposes
float yaw_cor=0.0;      //corrected yaw heading in radians

double pitchSetpoint, pitchInput, pitchOutput = 0.0;
volatile float pitch_co = 0.0;
float pitch_temp=0.0;     //make global for now for testing purposes

double yawKp=4.5, yawKi=0.6, yawKd=0.5;
double pitchKp=6.0, pitchKi=0.6, pitchKd=0.1;

PID yawPID(&yawInput, &yawOutput, &yawSetpoint, yawKp, yawKi, yawKd, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, pitchKp, pitchKi, pitchKd, DIRECT);

// feed motor variables
#define MAX_FEED_TIME 3.0
int feed_flag = 0;
float feed_time = 0.0;  // time in seconds to feed motor on (max is MAX_FEED_TIME)
unsigned long feedTimeLast;

// fire motor variables
#define FIRE_TIME_DELAY 500
#define MAX_FIRE_TIMES 10
int fire_flag = 0;
int fireNumTimes = 0;
int firing = 0; // 1 is currently firing, 0 is not firing

void mot_control_yaw(float dc){    
  if(dc>1.0)
      dc=1.0;
  if(dc<-1.0)
      dc=-1.0;
    
  dc *= 255.0;    // this scale may be incorrect with 20KHz PWM freq
         
  if(dc > 0.001){
    digitalWrite(A2, LOW);
    digitalWrite(A0, HIGH);
    analogWrite(A1, dc);
  }
  else if(dc < -0.001){
    digitalWrite(A0, LOW);
    digitalWrite(A2, HIGH);       
    analogWrite(A1, abs(int(dc)));
  }
  else{
    digitalWrite(A2, LOW);
    digitalWrite(A0, LOW);
    analogWrite(A1, 0);
  }         
}

void mot_control_pitch(float dc){    
  if(dc>1.0)
      dc=1.0;
  if(dc<-1.0)
      dc=-1.0;
    
  dc *= 255.0;    // this scale may be incorrect with 20KHz PWM freq
         
  if(dc > 0.001){
    digitalWrite(A3, LOW);
    digitalWrite(A4, HIGH);
    analogWrite(A5, dc);
  }
  else if(dc < -0.001){
    digitalWrite(A4, LOW);
    digitalWrite(A3, HIGH);       
    analogWrite(A5, abs(int(dc)));
  }
  else{
    digitalWrite(A3, LOW);
    digitalWrite(A4, LOW);
    analogWrite(A5, 0);
  }         
}

void testMotors(void){
  while(1){
    unsigned long time_ms = millis();
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  
    float motP_dc = .8 * sinf(float(time_ms)*.005);
    float motY_dc = .8 * cosf(float(time_ms)*.005);
    
    //motPitch.mot_control(motP_dc);
    //motYaw.mot_control(motY_dc);
    mot_control_yaw(motY_dc);
    mot_control_pitch(motP_dc);
    //Serial.printf("%7.2f %7.2f %7.2f \r\n\r\n", event.orientation.x, event.orientation.y, event.orientation.z);
    Serial.printf("%7.2f %7.2f %7.2f\r\n\r\n", orientationData.orientation.x, orientationData.orientation.y, orientationData.orientation.z);
    delay(10);   
  
    if(Serial.available()){
      char c = Serial.read();
      if(c == 'q' || c == 'Q')
        mot_control_pitch(0.0);
        mot_control_yaw(0.0);
        return;  
    }  // q for quit
  }// while(1)
}// test motors function


int con_state;
float yaw_err_unwrapped;
float yawContoller(float yaw_cor){
//first check for the sign, which direction is faster to turn in
    
//    //set the set point
    float yaw_sp = yawSetpoint;
    float yaw_error = 0.0;
    float yaw_sign_calc = 0.0;
    float yaw_temp_error = 0.0;
    
    yaw_err_unwrapped = yaw_sp - yaw_cor;
    
    if(yaw_sp >= yaw_cor){                   //If the set point is greater then the corrected heading
        yaw_error = yaw_sp - yaw_cor;       //get the difference
        
        if(yaw_error <= PI){        //Turn left
            con_state = 1;
            yaw_sign_calc=1.0; 
            
            //is the error < -PI
            if((yaw_error < 0.0) && (fabs(yaw_error) > PI))
              yaw_error += 2.0*PI;
              
             //is the error < -PI
            if((yaw_error > 0.0) && (fabs(yaw_error) > PI))
              yaw_error = 2.0*PI - yaw_error;
            
            //calculate the heading offset from error relative to setpoint for controller  
            if(yaw_error < 0.0)
                yaw_temp_error = yaw_sp + yaw_error;
            else
                yaw_temp_error = yaw_sp - yaw_error;       
        }                  
        else if(yaw_error > PI){       //Turn right
            con_state = 2;
            yaw_sign_calc=-1.0;
            
            //is the error < -PI
            if((yaw_error < 0.0) && (fabs(yaw_error) > PI))
              yaw_error += 2.0*PI;
              
             //is the error < -PI
            if((yaw_error > 0.0) && (fabs(yaw_error) > PI))
              yaw_error = 2.0*PI - yaw_error;
            
            //calculate the heading offset from error relative to setpoint for controller  
            if(yaw_error < 0.0)
                yaw_temp_error = yaw_sp - yaw_error;
            else
                yaw_temp_error = yaw_sp + yaw_error; 
        }
    }    
    else if(yaw_sp < yaw_cor){
        yaw_error = yaw_cor - yaw_sp;
        if(yaw_error <= PI){    //difference is
            con_state = 3; 
            yaw_sign_calc=-1.0;              //Turn left

            //is the error < -PI
            if((yaw_error < 0.0) && (fabs(yaw_error) > PI))
              yaw_error += 2.0*PI;
                            //is the error < -PI
            if((yaw_error > 0.0) && (fabs(yaw_error) > PI))
              yaw_error = 2.0*PI - yaw_error;
            
            //calculate the heading offset from error relative to setpoint for controller  
            if(yaw_error < 0.0)
                yaw_temp_error = yaw_sp - yaw_error;
            else
                yaw_temp_error = yaw_sp + yaw_error;                    
        }
        else if(yaw_error > PI){   //180
            con_state = 4;
            yaw_sign_calc=1.0;           //turn right
            //is the error < -PI
            if((yaw_error < 0.0) && (fabs(yaw_error) > PI))
              yaw_error += 2.0*PI;
                            //is the error < -PI
            if((yaw_error > 0.0) && (fabs(yaw_error) > PI))
              yaw_error = 2.0*PI - yaw_error;
            
            //calculate the heading offset from error relative to setpoint for controller  
            if(yaw_error < 0.0)
                yaw_temp_error = yaw_sp + yaw_error;
            else
                yaw_temp_error = yaw_sp - yaw_error;
        }
    }        
    return yaw_temp_error;
}

void setup() {
  delay(200);
  Serial.begin(115200);  
  Serial.print("EW309 Motor Test running.");
  Wire.setSDA(D0); // set the I2C pins from default to D0-SDA
  Wire.setSCL(D1); // D1-SCL
  Wire.begin();

  analogWriteFrequency(20000); // set output PWM frequency to 20KHz

  // Setup Motor Control Outputs for Pitch
  //digitalWrite(A5, LOW);   // sets the ME output LOW initially  
  digitalWrite(A3, LOW);   // sets the IN1 ouput LOW
  digitalWrite(A4, LOW);   // sets the IN2 ouput LOW
  //pinMode(A5, OUTPUT);  // PWM output for L289
  pinMode(A3, OUTPUT);  // IN1
  pinMode(A4, OUTPUT);  // IN2 
  
  // Setup Motor Control Outputs for Yaw
  //digitalWrite(A1, LOW);   // sets the ME output LOW initially  
  digitalWrite(A2, LOW);   // sets the IN1 ouput LOW
  digitalWrite(A0, LOW);   // sets the IN2 ouput LOW
  //pinMode(A1, OUTPUT);  // PWM output for L289
  pinMode(A2, OUTPUT);  // IN1
  pinMode(A0, OUTPUT);  // IN2 

  
  // setup the Nerf Gun mosfet output driver digital I/O
  digitalWrite(FIRE_PIN, LOW);
  digitalWrite(FEED_PIN, LOW);
  pinMode(FIRE_PIN, OUTPUT);    //
  pinMode(FEED_PIN, OUTPUT);
  
    /* Initialise the sensor */
  if(!bno.begin())
  {
    while(1){
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("\r\nOoops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      delay(1000);
    }
  }
  delay(200);
  bno.setExtCrystalUse(true);
  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1); // P0 - P7, see dataseet
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P1);    // P0 - P7, see dataseet
  bno.setMode(OPERATION_MODE_NDOF_FMC_OFF); // turn off the fusion mode
  
  //turn the PID on yaw
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(-250.0, 250.0); // min and max output limits (must change as defaults to 0, 255
  yawPID.SetSampleTime(20); //sets the frequency, in Milliseconds
  //turn the PID on pitch
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-250.0, 250.0); // min and max output limits (must change as defaults to 0, 255
  pitchPID.SetSampleTime(20); //sets the frequency, in Milliseconds
}

void loop() {         
  char str[30];
  unsigned long time_ms = millis();
  static float yawMeas, pitchMeas;
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  
  yawMeas = orientationData.orientation.x;
  yawMeas *= TO_RAD;
  if(yawMeas > PI)
        yawMeas = -(PI - yawMeas) - PI;
  //Serial.printf("%7.2f %7.2f %7.2f \r\n\r\n", event.orientation.x, event.orientation.y, event.orientation.z);

  pitchMeas = orientationData.orientation.y;
  pitchMeas *= TO_RAD;
//  if(pitchMeas > PI)
//        pitchMeas = -(PI - yawMeas) - PI;
        
  Serial.printf("%7.2f %7.2f yKp=%.2f yKi=%.2f yKd=%.2f pSP=%.3f ySP=%.3f\r\n", 
        yawMeas,
        pitchMeas, 
        yawKp, yawKi, yawKd,
        pitchSetpoint,  
        yawSetpoint); 
  
  //loop for yaw control - 20 millisecond update rate
  if(millis() >= yawTimeLast + CONTROL_LOOP_DELAY_MS){ 
    yawTimeLast = millis();

    // convert the degrees output to radians and correct to give +/-
    yaw_cor = orientationData.orientation.x * TO_RAD;
    //yaw_cor = bno.euler.yaw * YAW640_RATIO; //correct the yaw for radian measurement
    if(yaw_cor > PI)
        yaw_cor = -(PI - yaw_cor) - PI;

    float tempCorrectedHeading = yawContoller(yaw_cor);
    yawInput = tempCorrectedHeading;
    yawPID.Compute(); // calculates and auto updates the yawOutput
    //motYaw.mot_control(-yawOutput);
    mot_control_yaw(-yawOutput);
  }

  //loop for pitch control
  if(millis() >= pitchTimeLast + CONTROL_LOOP_DELAY_MS){
    pitchTimeLast = millis();

    pitchInput = pitchMeas; //pitchSetpoint - pitchMeas;
    pitchPID.Compute(); // calculates and auto updates the yawOutput
    //motPitch.mot_control(pitchOutput);    
    mot_control_pitch(pitchOutput);
  }
  
  // receive serial command
  if(Serial.available()){
      if(Serial.peek() == '?'){ // ? display commands
        int dummy = Serial.read();  
        Serial.printf(" SINGLE CHARACTER COMMANDS  \r\n");
        Serial.printf("'w' - move gun pitch up\r\n");
        Serial.printf("'z' - move gun pitch down\r\n");
        Serial.printf("'a' - move gun yaw left\r\n");
        delay(10);
        Serial.printf("'s' - move gun yaw right\r\n");
        Serial.printf("'r' - reset the BNO-055 IMU\r\n\r\n");
        Serial.printf(" STRING COMMANDS (follo0wed by carriage return '\\r') \r\n");
        Serial.printf("testmot\\r - test the motors on the pan/tilt head\r\n");
        delay(10);
        Serial.printf("yawsp x.x\\r - Set the yaw set point to degrees\r\n");
        Serial.printf("yawp x.x\\r - Set the proportional gain for the yaw axis\r\n");
        Serial.printf("yawi x.x\\r - Set the integral gain for the yaw axis\r\n");
        Serial.printf("yawd x.x\\r - Set the derivative gain for the yaw axis\r\n");
        delay(10);
        Serial.printf("pitchsp x.x\\r - Set the pitch set point to degrees\r\n");
        Serial.printf("pitchp x.x\\r - Set the proportional gain for the pitch axis\r\n");
        Serial.printf("pitchi x.x\\r - Set the integral gain for the pitch axis\r\n");
        Serial.printf("pitchd x.x\\r - Set the derivative gain for the pitch axis\r\n");
        delay(10);
        Serial.printf("feed x.x\\r - turn on feed motor for x.x seconds\r\n");
        Serial.printf("fire x\\r - Fire x number of times\r\n");
        Serial.printf("fire\\r - Fire one shot\r\n");

        // clar out any additional characters in the Serial buffer
        while(Serial.available()){
          char c = Serial.read();
        }

        // wait for single character press to exit menu
        while(!Serial.available()); 
        char c = Serial.read();       // expunge new character entered in buffer
      }    
      if(Serial.peek() == 'w'){ // up key
        int keylow = Serial.read();  
        pitchSetpoint += 10.0 * (PI/180.0);
      }
      if(Serial.peek() == 'z'){ // down key
        int keylow = Serial.read();  
        pitchSetpoint -= 10.0 * (PI/180.0);
      }
      if(Serial.peek() == 'a'){ // left key
        int keylow = Serial.read();  
        yawSetpoint += 10.0 * (PI/180.0);
      }
      if(Serial.peek() == 's'){ // right key
        int keylow = Serial.read();  
        yawSetpoint -= 10.0 * (PI/180.0);        
      }      
      //Serial.printf("key = %2X\r\n");
      //delay(2000);
      if(Serial.peek() == 'r'){ // reset BNO
        mot_control_yaw(0.0);
        mot_control_pitch(0.0);
        
        Serial.printf("Reset BNO-055\r\n");
          if(!bno.begin()){
          while(1){
            /* There was a problem detecting the BNO055 ... check your connections */
            Serial.print("\r\nOoops, no BNO055 detected ... Check your wiring or I2C ADDR!");
            delay(1000);
          }
        }
        delay(200);
        bno.setExtCrystalUse(true);
        bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1); // P0 - P7, see dataseet
        bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P1);    // P0 - P7, see dataseet
        bno.setMode(OPERATION_MODE_NDOF_FMC_OFF); // turn off the fusion mode

        yawSetpoint = 0.0;
        pitchSetpoint = 0.0;
      }    
      Serial.readBytesUntil('\r', str, 30);
      
      if(strncmp(str, "testmot", 7) == 0){   
        testMotors();
      }
      
      if(strncmp(str, "yawsp ", 6) == 0){ 
        float tempf=0.0;
      
        int numvals = sscanf(str, "yawsp %f\r", &tempf);
        if(numvals == 1){                
          if(tempf < -45.0)
            tempf = -45.0;
          if(tempf > 45.0)
            tempf = 45.0;
  
          tempf *= TO_RAD;
          if(tempf > PI)
            tempf = -(PI - tempf) - PI;
          yawSetpoint = tempf;
          Serial.printf("\r\nSetpointYaw = %7.3f\r\n", yawSetpoint);
          //delay(2000);
        }
        else{
          Serial.printf("\r\nBad yaw command received\r\n");
        }
      }// yaw setpoint command
  
      //Set PID gains for yaw
      // Kp
      if(strncmp(str, "yawp ", 5) == 0){ 
        float tempf=0.0;
      
        int numvals = sscanf(str, "yawp %f\r", &tempf);
        if(numvals == 1){                
          if(tempf < 0.0)
            tempf = 0.0;
          if(tempf > 20.0)
            tempf = 20.0;
  
          yawKp = tempf;
          Serial.printf("\r\nYaw Kp= %7.3f\r\n", yawKp);
          yawPID.SetTunings(yawKp, yawKi, yawKd);
        }
        else{
          Serial.printf("\r\nBad yaw command received\r\n");
        }
      }// change yaw Kp gain
  
      // Ki
      if(strncmp(str, "yawi ", 5) == 0){ 
        float tempf=0.0;
      
        int numvals = sscanf(str, "yawi %f\r", &tempf);
        if(numvals == 1){                
          if(tempf < 0.0)
            tempf = 0.0;
          if(tempf > 20.0)
            tempf = 20.0;
  
          yawKi = tempf;
          Serial.printf("\r\nYaw Ki= %7.3f\r\n", yawKi);
          yawPID.SetTunings(yawKp, yawKi, yawKd);
        }
        else{
          Serial.printf("\r\nBad yaw command received\r\n");
        }
      }
  
      // Kd
      if(strncmp(str, "yawd ", 5) == 0){ 
        float tempf=0.0;
      
        int numvals = sscanf(str, "yawd %f\r", &tempf);
        if(numvals == 1){                
          if(tempf < 0.0)
            tempf = 0.0;
          if(tempf > 20.0)
            tempf = 20.0;
  
          yawKd = tempf;
          Serial.printf("\r\nYaw Kd= %7.3f\r\n", yawKd);
          yawPID.SetTunings(yawKp, yawKi, yawKd);
        }
        else{
          Serial.printf("\r\nBad yaw command received\r\n");
        }
      }// change yaw Ki gain 

         // pitch controller
      if(strncmp(str, "pitchsp ", 8) == 0){ 
        float tempf=0.0;
      
        int numvals = sscanf(str, "pitchsp %f\r", &tempf);
        if(numvals == 1){                
          if(tempf < -30.0)
            tempf = -30.0;
          if(tempf > 30.0)
            tempf = 30.0;
  
          tempf *= TO_RAD;
//          if(tempf > PI)
//            tempf = -(PI - tempf) - PI;
          pitchSetpoint = tempf;
          Serial.printf("\r\nSetpointpitch = %7.3f\r\n", pitchSetpoint);
          //delay(2000);
        }
        else{
          Serial.printf("\r\nBad pitch command received\r\n");
        }
      }// pitch setpoint command
  
      //Set PID gains for pitch
      // Kp
      if(strncmp(str, "pitchp ", 5) == 0){ 
        float tempf=0.0;
      
        int numvals = sscanf(str, "pitchp %f\r", &tempf);
        if(numvals == 1){                
          if(tempf < 0.0)
            tempf = 0.0;
          if(tempf > 20.0)
            tempf = 20.0;
  
          pitchKp = tempf;
          Serial.printf("\r\npitch Kp= %7.3f\r\n", pitchKp);
          pitchPID.SetTunings(pitchKp, pitchKi, pitchKd);
        }
        else{
          Serial.printf("\r\nBad pitch command received\r\n");
        }
      }// change pitch Kp gain
  
      // Ki
      if(strncmp(str, "pitchi ", 5) == 0){ 
        float tempf=0.0;
      
        int numvals = sscanf(str, "pitchi %f\r", &tempf);
        if(numvals == 1){                
          if(tempf < 0.0)
            tempf = 0.0;
          if(tempf > 20.0)
            tempf = 20.0;
  
          pitchKi = tempf;
          Serial.printf("\r\npitch Ki= %7.3f\r\n", pitchKi);
          pitchPID.SetTunings(pitchKp, pitchKi, pitchKd);
        }
        else{
          Serial.printf("\r\nBad pitch command received\r\n");
        }
      }
  
      // Kd - pitch
      if(strncmp(str, "pitchd ", 7) == 0){ 
        float tempf=0.0;
      
        int numvals = sscanf(str, "pitchd %f\r", &tempf);
        if(numvals == 1){                
          if(tempf < 0.0)
            tempf = 0.0;
          if(tempf > 5.0)
            tempf = 5.0;
  
          pitchKd = tempf;
          Serial.printf("\r\nPitch Kd= %7.3f\r\n", pitchKd);
          pitchPID.SetTunings(pitchKp, pitchKi, pitchKd);
        }
        else{
          Serial.printf("\r\nBad pitch command received\r\n");
        }
      }// change pitch Ki gain         

      // feed motor drive on for time X (max 3 sec)
      if(strncmp(str, "feed ", 5) == 0){ 
//        digitalWrite(FEED_PIN, HIGH);
//        delay(2000);
//        digitalWrite(FEED_PIN, LOW);   
        
        float timef=0.0;        
      
        int numvals = sscanf(str, "feed %f\r", &timef);
        if(numvals == 1){                
          if(timef < 0.0)
            timef = 0.0;
          if(timef > MAX_FEED_TIME)
            timef = MAX_FEED_TIME;

          feed_time = (unsigned long)(timef * 1000.0) + millis(); // update the global variable for feedtime
          feed_flag = 1;
          Serial.printf("\r\nFeed T=%7.3f\r\n", timef);
        }
        else{
          Serial.printf("\r\nBad feed command received\r\n");
        }
      }// motor feed command

      // fire trigger function
      if(strncmp(str, "fire ", 5) == 0){ 
        digitalWrite(FEED_PIN, HIGH);     
      
        int numvals = sscanf(str, "fire %d\r", &fireNumTimes);
        if(numvals == 1){                
          if(fireNumTimes < 0)
            fireNumTimes = 0;
          if(fireNumTimes > MAX_FIRE_TIMES)
            fireNumTimes = MAX_FIRE_TIMES;

          Serial.printf("\r\nFire times =%d\r\n", fireNumTimes);
        }
        else{
          Serial.printf("\r\nBad feed command received\r\n");
        }
        digitalWrite(FEED_PIN, HIGH);
        delay(1200);
        feedTimeLast = millis();
        firing = 1;
      }// motor "fire X" command
      else{
          // fire trigger function single
        if(strncmp(str, "fire", 4) == 0){ 
          digitalWrite(FEED_PIN, HIGH);
          delay(1200);
          digitalWrite(FIRE_PIN, HIGH);
          delay(200);
          digitalWrite(FIRE_PIN, LOW);
          digitalWrite(FEED_PIN, LOW);
          delay(200);
        }
      }
      //clear out any garbage in buffer
      while(Serial.available()){
        char c = Serial.read();
      }         
  }// if serial command received
 
  // if the feed and fire command is triggered
  if(firing == 1){
    //Serial.printf("Entered firing!\r\n");
    if(fireNumTimes > 0){
      //Serial.printf("Entered Fire Num Times");
      //Serial.printf("%d     Fire Num Times = %d!\r\n", millis(), fireNumTimes);
      digitalWrite(FEED_PIN, HIGH);
      delay(500);
      if(millis() > (feedTimeLast + FIRE_TIME_DELAY)){
          Serial.printf("Fire Num Times = %d!\r\n", fireNumTimes);
          digitalWrite(FIRE_PIN, !digitalRead(FIRE_PIN));
          feedTimeLast = millis();
          fireNumTimes--;
      }
           
    }// number of times fired > 0
    else{
        feedTimeLast = 0;
        feed_flag = 0; // end the 
        firing = 0;
        digitalWrite(FIRE_PIN, LOW);
        digitalWrite(FEED_PIN, LOW);
      }
      delay(20);
  } // feed flag is set

  
  delay(10);
} // loop()
// END FILE
