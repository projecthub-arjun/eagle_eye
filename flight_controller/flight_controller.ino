// Eagle Eye Drone Flight Controller
// Set the optimization level
#pragma GCC optimize ("-O3")

// Wire Library is used to communicate with the Gyroscope (MPU6050)
// This library provides API's for I2C communication
#include <Wire.h>

// Registers of MPU 6050
#define MPU_6050_ADDR 0x68
#define MPU_6060_PWR_DWN_REG_ADDR 0x6B
#define MPU_6060_ACC_CONFIG_REG_ADDR 0x1C
#define MPU_6060_GYRO_CONFIG_REG_ADDR 0x1B
#define MPU_6060_DATA_START_ADDR 0x3B

#define MPU_6050_REG_SET 0x01
#define MPU_6050_REG_UNSET 0x00
#define MPU_6050_ACC_CONFIG_2 0x10
#define MPU_6050_GYRO_CONFIG_2 0x08
#define MPU_BYTES_TO_READ 14

// ESC pin mask for setting pins high
#define ESC_PD_RR_H B01000000         // Pin 6
#define ESC_PD_RF_H B00100000         // Pin 5
#define ESC_PB_LF_H B00001000         // Pin 11
#define ESC_PB_LR_H B00000100         // Pin 10

// ESC pin mask for setting pins Low
#define ESC_PD_RR_L B10111111         // Pin 6
#define ESC_PD_RF_L B11011111         // Pin 5
#define ESC_PB_LF_L B11110111         // Pin 11
#define ESC_PB_LR_L B11111011         // Pin 10

// Tramitter button values
#define BUTTON_A       128
#define BUTTON_B       64
#define BUTTON_X       32
#define BUTTON_Y       16
#define BUTTON_BACK    2
#define BUTTON_L1      8
#define BUTTON_R1      4

// Limiting Values
#define MAX_THROTTLE 1600

// Receiver Channel 1 --> Roll
// Receiver Channel 2 --> Pitch
// Receiver Channel 3 --> Throttle
// Receiver Channel 4 --> Yaw
// These values will simulate that the sticks are in center position
// Range of receiver channel [1000-2000]
unsigned int receiver_channel_1 = 1500;
unsigned int receiver_channel_2 = 1500;
unsigned int receiver_channel_3 = 1500;
unsigned int receiver_channel_4 = 1500;
byte button_data = 0;

// This integer is used for calibration of ESC's and Gyroscope
int cal_int = 0;

// This variable will hold the start condition
int start = 0;

// Variables to hold ESC values (1000 -2000)
int esc_1, esc_2, esc_3, esc_4;

// This integer hold the temperate value from gyroscope
int temperature = 0;

// Varaibles to hold the raw accelerometer data
long acc_x = 0, acc_y = 0, acc_z = 0, acc_total_vector = 0;

// Variables for holding gyro calibration offset values
long gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0;

// Varaibles to hold raw gyroscope data
int gyro_x = 0, gyro_y = 0, gyro_z = 0;

// Variables for holding the pitch and roll angles
double angle_pitch_acc = 0, angle_roll_acc = 0, angle_pitch = 0, angle_roll = 0;

// This varaible is used to maintain the refresh rate of 4ms
unsigned long loop_timer = 0;

// These varaibles are used for the esc pulse generation loop
unsigned long esc_loop_timer = 0;
unsigned long esc_1_timer = 0, esc_2_timer = 0, esc_3_timer = 0, esc_4_timer = 0;

// Receiver Variables
int throttle = 1000;

// This variable stores the battery voltage
int battery_voltage = 0;

// When the drone is started, initialize the gyro values with acc angles
bool gyro_angles_set = false;

// Variables for PID calculations
float pid_error_temp = 0;
float pid_i_mem_roll = 0, pid_roll_setpoint = 0, gyro_roll_input = 0, pid_output_roll = 0, pid_last_roll_d_error = 0;
float pid_i_mem_pitch = 0, pid_pitch_setpoint = 0, gyro_pitch_input = 0, pid_output_pitch = 0, pid_last_pitch_d_error = 0;
float pid_i_mem_yaw = 0, pid_yaw_setpoint = 0, gyro_yaw_input = 0, pid_output_yaw = 0, pid_last_yaw_d_error = 0;

// PID Limit Settings and Gain
float pid_p_gain_roll = 1.0;                    // Gain setting for the roll P-controller //1.3
float pid_i_gain_roll = 0.0;                    // Gain setting for the roll I-controller
float pid_d_gain_roll = 0.0;                   // Gain setting for the roll D-controller
static const int pid_max_upper_roll = 400;      // Maximum output of the PID-controller (+/-)
static const int pid_max_lower_roll = -400;     // Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;                   // Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;                   // Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;                   // Gain setting for the pitch D-controller.
static const int pid_max_upper_pitch = pid_max_upper_roll;  // Maximum output of the PID-controller (+/-)
static const int pid_max_lower_pitch = pid_max_lower_roll;  // Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                     // Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;                    // Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                     // Gain setting for the pitch D-controller.
static const int pid_max_upper_yaw = 400;       // Maximum output of thes PID-controller (+/-)
static const int pid_max_lower_yaw = -400;      // Maximum output of the PID-controller (+/-)

// Varaibles for auto levelling the drone
float roll_level_adjust = 0, pitch_level_adjust = 0;
unsigned long gyro_time = 0;

// Buffer to hold the feedback data to the ground station
unsigned char feedback[17] = {0};
void setup()
{   
    // Start the serial communication with the sky station
    Serial.begin(115200);

    // Start the wire library
    Wire.begin();
    //Wire.setClock(400000L);
    // Set the I2C clock speed to 400kHz.
    TWBR = 12;

    // Configure the ESC's pins as output pins
    // Configure pins 12,13 as output, for LED's
    // ESC's are connected to pins 5,6,10,11
    DDRB |= B00111100;  // X, X, 13, 12, 11, 10, 9, 8
    DDRD |= B01100000;  // 7, 6, 5, 4, 3, 2, 1, 0
    
    // Siwtch on the Blue LED pin 13
    PORTB |= B00100000;
    
    // Initialize the MPU 6050 Gyroscope
    GyroInitialize();
    
    // Initalize the ESC's by sending a 1000us pulse
    // lowest value, to prevent them from beeping crazy
    // Write a 1000us pulse on pins 5,6,10,11 for 5s.
    // 4000us * 1250 = 5000ms (5s)
    for(cal_int = 0; cal_int < 1250; cal_int++)
    {
        // This Loop simulates a refresh rate of 4ms
        // The total length of the pulses are 4000us (4ms)
        PORTD |= B01100000;                 // Set pins 5,6 High
        PORTB |= B00001100;                 // Set pins 10,11 High
        delayMicroseconds(1000);            // Wait for 1000us
        PORTD &= B10011111;                 // Set pins 5,6 Low
        PORTB &= B11110011;                 // Set pins 10,11 Low
        delayMicroseconds(3000);            // Wait 3000us to maintain refresh rate (4ms)
    }        
   
    // Now we need to calibrate the gyro, this simply means 
    // to find the offset of the gyro when it is still
    Serial.println("Calibrating Gyro...");
    for (cal_int = 0; cal_int < 2000 ; cal_int ++)
    {                           
        if(cal_int % 15 == 0)
        {
          // Change the led status to indicate calibration
          // Using digital read and write functions here is
          // fine because refresh rate doesn't matter in this loop
          // as we are not updating the ESC's or flying the drone.
          digitalWrite(13, !digitalRead(13));                
        }
        // Read the gyro values
        GyroRead();
        
        gyro_x_cal += gyro_x;               // X Axis offset --> Pitch
        gyro_y_cal += gyro_y;               // Y Axis offset --> Roll
        gyro_z_cal += gyro_z;               // Z Axis offset --> Yaw

        // Feed the ESC's with a 1000us pulse as before to prevent them
        // from beeping annoyingly, the refresh rate is maintained at 4ms
        PORTD |= B01100000;                 // Set pins 5,6 High
        PORTB |= B00001100;                 // Set pins 10,11 High
        delayMicroseconds(1000);            // Wait for 1000us
        PORTD &= B10011111;                 // Set pins 5,6 Low
        PORTB &= B11110011;                 // Set pins 10,11 Low
        delayMicroseconds(3000);            // Wait 3000us to maintain refresh rate (4ms)
    }
    // Find the average offset
    gyro_x_cal /= 2000;
    gyro_y_cal /= 2000;
    gyro_z_cal /= 2000;
    
    // Wait till the throttle stick is at the lowest position
    while(receiver_channel_3 > 1020)
    {
        ReceiveTransmitter();
        start++;
        // Feed the ESC's with a 1000us pulse as before to prevent them
        // from beeping annoyingly, the refresh rate is maintained at 4ms
        PORTD |= B01100000;                 // Set pins 5,6 High
        PORTB |= B00001100;                 // Set pins 10,11 High
        delayMicroseconds(1000);            // Wait for 1000us
        PORTD &= B10011111;                 // Set pins 5,6 Low
        PORTB &= B11110011;                 // Set pins 10,11 Low
        delayMicroseconds(3000);            // Wait 3000us to maintain refresh rate (4ms)
        if(start == 125)
        {
            // Blink the Blue LED to let the user know it's waiting
            // to be armed
            digitalWrite(13, !digitalRead(13));   
            start = 0; 
        }
    }
    // Reset the start variable
    start = 0;

    //Load the battery voltage to the battery_voltage variable.
    //12.6V equals ~5V @ Analog 0.
    //12.6V equals 1023 analogRead(0).
    //1260 / 1023 = 1.2317.
    //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
    battery_voltage = (analogRead(0)) * 1.2317;

    // Start the loop_timer
    loop_timer = micros();
    
    // Turn the Blue LED off to indicae that the drone is armed
    digitalWrite(13,LOW);
}

void loop()
{
    // Calculate the input values of the PID controller
    // This is just a complementary filter that takes 70% 
    // of the old gyro value and adds 30% new value
    // Divide the raw gyro values by 65.5 to convert it to degress/s
    //Serial.print(gyro_x);Serial.print(" ");Serial.print(gyro_y);Serial.print(" ");Serial.println(gyro_z);
    gyro_pitch_input = (gyro_pitch_input * 0.7) + (gyro_x * 0.00458); // ((gyro_x / 65.5) * 0.3);
    gyro_roll_input  = (gyro_roll_input * 0.7)  + (gyro_y * 0.00458); // ((gyro_y / 65.5) * 0.3);
    gyro_yaw_input   = (gyro_yaw_input * 0.7)   + (gyro_z * 0.00458); // ((gyro_z / 65.5) * 0.3);

    /******************************************************************
     * This is the part where the pitch and roll angels are calculated
     ******************************************************************/
    // Gyro angle calculations
    // To get the total angle travesered by the gyro
    // we need to add up the raw values and divide it by 65.5
    // According to the datasheet fof MPU 6050 and the set configuration
    // 1 degree/s of rotation corresponds to a raw value of 65.5
    // A value is read every 4ms ( 250 times a second ) and 1 degree/s corresponds
    // to 65.5, that is why raw pitch and roll values are multiplied with 0.0000611 
    // 0.0000611 = 1 / (250Hz / 65.5)
    // Total pitch and roll angles
    angle_pitch += gyro_x * 0.0000611;
    angle_roll += gyro_y * 0.0000611;
    
    // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr)
    // The Arduino sin function is in radians
    // If the IMU has yawed transfer the roll angle to the pitch angel
    // and pitch angle to the roll angel, the relationship between them varies
    // as a function of sin
    angle_pitch += angle_roll * sin( gyro_z * 0.000001066 );
    angle_roll -= angle_pitch * sin( gyro_z * 0.000001066 );
      
    // Accelerometer angle calculations
    // Calculate the total accelerometer vector
    acc_total_vector = sqrt( ( acc_x * acc_x ) + ( acc_y * acc_y ) + ( acc_z * acc_z ) );
    
    // Prevent the asin function to produce a NaN
    // 57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
    if(abs(acc_y) < acc_total_vector)
    {
       // Calculate the pitch angle.
      angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;         
    }
    // Prevent the asin function to produce a NaN
    if(abs(acc_x) < acc_total_vector)
    {                      
      // Calculate the roll angle.    
      angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          
    }
    
    // Place the MPU-6050 spirit level and note the values
    // Accelerometer calibration value for pitch and roll
    angle_pitch_acc -= 0.0;
    angle_roll_acc -= 2.67;
    
    // If the IMU is already started
    // Correct the drift of the gyro pitch angle with the accelerometer pitch ang
    // Correct the drift of the gyro roll angle with the accelerometer roll angle
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

    // If the IMU is already started
    // Correct the drift of the gyro pitch angle with the accelerometer pitch ang
    // Correct the drift of the gyro roll angle with the accelerometer roll angle
    if( gyro_angles_set )
    {
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
    }
    // At first start
    // Set the gyro pitch angle equal to the accelerometer pitch angle
    // Set the gyro roll angle equal to the accelerometer roll angle
    // Set the IMU started flag
    else
    {
        angle_pitch = angle_pitch_acc;
        angle_roll = angle_roll_acc;
        gyro_angles_set = true;
    }

    
    /******************************************************************
     * This is the part where the rest happens
     ******************************************************************/
    //Serial.print("P= ");Serial.print(angle_pitch);Serial.print(" ");Serial.println(angle_roll);
    pitch_level_adjust = angle_pitch * 15;         // Calculate the pitch angle correction
    roll_level_adjust = angle_roll * 15;           // Calculate the roll angle correction

    // This is a fail safe, start the motors only if the value
    // of the start variable is 2, to start the motors press L1
    if(button_data == BUTTON_R1)
    {
        start = 2;
        
        // Reset the gyro values
        angle_pitch = angle_pitch_acc;
        angle_roll = angle_roll_acc;
        gyro_angles_set = true;

        // Reset the PID values for a bumpless start.
        pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
    }
    // To disable the motors press R1
    if(button_data == BUTTON_L1)
    {
        start = 0;
    }
    
    // The PID set point in degrees per second is determined by the roll receiver input.
    // In the case of dividing by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_roll_setpoint = 0;
    
    //We need a little dead band of 16us for better results.
    if(receiver_channel_1 > 1508)
    {
        pid_roll_setpoint = ((float)receiver_channel_1 - 1508.0);
    }
    else if(receiver_channel_1 < 1492)
    {
        pid_roll_setpoint = ((float)receiver_channel_1 - 1492.0);
    }
        
    
    pid_roll_setpoint -= roll_level_adjust;
    pid_roll_setpoint /= 3.0;    

    // The PID set point in degrees per second is determined by the pitch receiver input.
    // In the case of dividing by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_pitch_setpoint = 0;
    
    // We need a little dead band of 16us for better results.
    if(receiver_channel_2 > 1508)
    {
        pid_pitch_setpoint = ((float)receiver_channel_2 - 1508.0);
    }
    else if(receiver_channel_2 < 1492)
    {
        pid_pitch_setpoint = ((float)receiver_channel_2 - 1492.0);
    }
    
    pid_pitch_setpoint -= pitch_level_adjust;       
    pid_pitch_setpoint /= 3.0;                      


    // The PID set point in degrees per second is determined by the yaw receiver input.
    // In the case of dividing by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_yaw_setpoint = 0;
    
    // We need a little dead band of 16us for better results.
    if(receiver_channel_4 > 1508)
    {
        pid_yaw_setpoint = ((float)receiver_channel_4 - 1508.0) / 3.0;
    }
    else if(receiver_channel_4 < 1492)
    {
        pid_yaw_setpoint = ((float)receiver_channel_4 - 1492.0) / 3.0;
    }
    
    //Serial.print("R=");Serial.print(pid_roll_setpoint);Serial.print(" ");
    //Serial.print("P=");Serial.print(pid_pitch_setpoint);Serial.print(" ");
    //Serial.print("Y=");Serial.println(pid_yaw_setpoint);
    // Now that the set points are known, we can calculate the ouput of
    // the PID controller
    CalculatePID(); 

    //The battery voltage is needed for compensation.
    //A complementary filter is used to reduce noise.
    //0.09853 = 0.08 * 1.2317.
    battery_voltage = battery_voltage * 0.92 + (analogRead(0)) * 0.09853;
    
    //Turn on the led if battery voltage is to low.
    if(battery_voltage < 1000 && battery_voltage > 600)
    {
        //PORTB |= B00100000;
    }

    if(start == 2)
    {
        // Limit the max throttle value, for more control
        throttle = ((receiver_channel_3 > MAX_THROTTLE)? MAX_THROTTLE : receiver_channel_3);
        //Serial.println(throttle);
        //Serial.print(pid_output_pitch);Serial.print(" R = ");Serial.print(pid_output_roll);Serial.print(" Y = ");Serial.println(pid_output_yaw);
        esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
        esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
        esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
        esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
       
        // Is the battery connected?
        if (battery_voltage < 1240 && battery_voltage > 800)
        {
            esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
            esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
            esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
            esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
                     
        } 
         

        if (esc_1 < 1050) esc_1 = 1050;                                         //Keep the motors running.
        if (esc_2 < 1050) esc_2 = 1050;                                         //Keep the motors running.
        if (esc_3 < 1050) esc_3 = 1050;                                         //Keep the motors running.
        if (esc_4 < 1050) esc_4 = 1050;                                         //Keep the motors running.

        if(esc_1 > 1800) esc_1 = 1800;                                           //Limit the esc-1 pulse to 2000us.
        if(esc_2 > 1800) esc_2 = 1800;                                           //Limit the esc-2 pulse to 2000us.
        if(esc_3 > 1800) esc_3 = 1800;                                           //Limit the esc-3 pulse to 2000us.
        if(esc_4 > 1800) esc_4 = 1800;                                           //Limit the esc-4 pulse to 2000us.
      /*
       Serial.print(esc_1);Serial.print(" ");
       Serial.print(esc_2);Serial.print(" ");
       Serial.print(esc_3);Serial.print(" ");
       Serial.println(esc_4);*/
       //esc_1 = esc_2 = esc_3 = esc_4 = 2000;
    }
    else
    {
        esc_1 = 1000;
        esc_2 = 1000;
        esc_3 = 1000;
        esc_4 = 1000;
    }

    // Because of the angle calculation the loop time is getting very important. 
    // If the loop time is longer or shorter than 4ms the angle calculation is off. 
    // If you modify the code make sure that the loop time is still 4ms and no longer!
    if(micros() - loop_timer > 4050)
    {
        // Turn the Blue LED back on if the refresh rate is slow
        PORTB |= B00100000;
    }
    //Serial.println(micros() - loop_timer);
    // Wait until 4ms is up
    while(micros() - loop_timer < 4000)
    {
        
    }
    loop_timer = micros();              // Refresh the loop timer


    PORTD |= B01100000;                 // Set pins 5,6 High
    PORTB |= B00001100;                 // Set pins 10,11 High

    esc_1_timer = esc_1 + loop_timer;
    esc_2_timer = esc_2 + loop_timer;
    esc_3_timer = esc_3 + loop_timer;
    esc_4_timer = esc_4 + loop_timer;
    
    GyroRead();

    // Wait until the bits of pins 5,6,10,11 are all LOW
    while((PORTD & B01100000) || (PORTB & B00001100))
    {
        esc_loop_timer = micros();
        if(esc_1_timer <= esc_loop_timer)
        {
           PORTD &= ESC_PD_RF_L;            // Set Pin 5 Low
        }
        if(esc_2_timer <= esc_loop_timer)
        {
           PORTD &= ESC_PD_RR_L;            // Set Pin 6 Low
        }
        if(esc_3_timer <= esc_loop_timer)
        {
           PORTB &= ESC_PB_LR_L;            // Set Pin 10 Low
        }
        if(esc_4_timer <= esc_loop_timer)
        {
           PORTB &= ESC_PB_LF_L;            // Set Pin 11 Low
        }
    }
    // Read the Gryo and Transmitter Here
    ReceiveTransmitter();
}
void CalculatePID()
{
  // Roll calculations
  // Calculate the error (difference b/w actual roll value and desired value)
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;

  // Calculate the I controller value by adding up the errors over timer
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  
  // Limit the values b/w upper and lower limit
  // This ensures that the drone does not go crazy
  // and also ensures that calculation time is under control
  if(pid_i_mem_roll > pid_max_upper_roll)
  {
    pid_i_mem_roll = pid_max_upper_roll;
  }
  else if(pid_i_mem_roll < pid_max_lower_roll)
  {
    pid_i_mem_roll = pid_max_lower_roll;
  }
  
  // Calculate the ouput PID roll value
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  
  // Limit the roll value b/w upper and lower limit
  if(pid_output_roll > pid_max_upper_roll)
  {
      pid_output_roll = pid_max_upper_roll; 
  }
  else if(pid_output_roll < pid_max_lower_roll)
  {
      pid_output_roll = pid_max_lower_roll;     
  }

  // Store this error value for calculating the 
  // next I and D values
  pid_last_roll_d_error = pid_error_temp;

  // Pitch calculations
  // Calculate the error (difference b/w actual pitch and desired value)
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;

  // Calculate the I controller value by adding up the errors over timer
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  
  // Limit the values b/w upper and lower limit
  // This ensures that the drone does not go crazy
  // and also ensures that calculation time is under control
  if(pid_i_mem_pitch > pid_max_upper_pitch)
  {
      pid_i_mem_pitch = pid_max_upper_pitch;
  }
  else if(pid_i_mem_pitch < pid_max_lower_pitch)
  {
      pid_i_mem_pitch = pid_max_lower_pitch;
  }

  // Calculate the ouput PID pitch value
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  
  // Limit the pitch value b/w upper and lower limit
  if(pid_output_pitch > pid_max_upper_pitch)
  {
      pid_output_pitch = pid_max_upper_pitch;
  }
  else if(pid_output_pitch < pid_max_lower_pitch)
  {
      pid_output_pitch = pid_max_lower_pitch;
  }

  // Store this error value for calculating the 
  // next I and D values
  pid_last_pitch_d_error = pid_error_temp;

  // Yaw calculations
  // Calculate the error (difference b/w actual yaw and desired value)
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;

  // Calculate the I controller value by adding up the errors over timer
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;

  // Limit the values b/w upper and lower limit
  // This ensures that the drone does not go crazy
  // and also ensures that calculation time is under control
  if(pid_i_mem_yaw > pid_max_upper_yaw)
  {
      pid_i_mem_yaw = pid_max_upper_yaw;
  }
  else if(pid_i_mem_yaw < pid_max_lower_yaw)
  {
      pid_i_mem_yaw = pid_max_lower_yaw;
  }

  // Calculate the ouput PID yaw value
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);

  // Limit the pitch value b/w upper and lower limit
  if(pid_output_yaw > pid_max_upper_yaw)
  {
      pid_output_yaw = pid_max_upper_yaw;
  }
  else if(pid_output_yaw < pid_max_lower_yaw)
  {
      pid_output_yaw = pid_max_lower_yaw;
  }
  
  // Store this error value for calculating the 
  // next I and D values
  pid_last_yaw_d_error = pid_error_temp;
}
void ReceiveTransmitter()
{
  // Packet format from sky station
  // [HEADER] 
  // [PITCH_HIGH]    [PITCH_LOW] 
  // [ROLL_HIGH]     [ROLL_LOW] 
  // [YAW_HIGH]      [YAW_LOW]
  // [THROTTLE_HIGH] [THROTTLE_LOW]
  // [BUTTONS]
  if(Serial.available())
  {
    // Check if a header byte is received
    byte header = Serial.read();
    if(1 == header)
    {
      // Wait until an entire frame is available (9 bytes)
      while(Serial.available() < 9)
      {
      }
      
      // Combine the high and low byte to get a value in the range
      // [2000-4000] , divide this by 2 to make the range [1000-2000]
      receiver_channel_2  = (Serial.read() << 8 | Serial.read()) >> 1;
      receiver_channel_1  = (Serial.read() << 8 | Serial.read()) >> 1;
      receiver_channel_4  = (Serial.read() << 8 | Serial.read()) >> 1;
      receiver_channel_3  = (Serial.read() << 8 | Serial.read()) >> 1;
      button_data         =  Serial.read();
    }
  }
}
void GyroInitialize()
{
  // By default the MPU 6050 is in power down mode
  // To wake up the MPU set the power down register
  // value to zero.
  Wire.beginTransmission( MPU_6050_ADDR );
  Wire.write( MPU_6060_PWR_DWN_REG_ADDR );
  Wire.write( MPU_6050_REG_UNSET );
  Wire.endTransmission();

  // Configure the accelerometer ( +/-8g )
  // 1g -> 4096
  Wire.beginTransmission( MPU_6050_ADDR );
  Wire.write( MPU_6060_ACC_CONFIG_REG_ADDR );
  Wire.write( MPU_6050_ACC_CONFIG_2 );
  Wire.endTransmission();

  // Configure the gyro ( 500dps full scale )
  Wire.beginTransmission( MPU_6050_ADDR );
  Wire.write( MPU_6060_GYRO_CONFIG_REG_ADDR );
  Wire.write( MPU_6050_GYRO_CONFIG_2 );
  Wire.endTransmission();
}

void GyroRead()
{
  // Read the Acc, Temp, gyro values
  Wire.beginTransmission( MPU_6050_ADDR );
  Wire.write( MPU_6060_DATA_START_ADDR );
  Wire.endTransmission();
  Wire.requestFrom( MPU_6050_ADDR, MPU_BYTES_TO_READ );

  // Wait till 14 bytes are available

  while( Wire.available() < MPU_BYTES_TO_READ );

  // The First 6 bytes is the accelerometer value
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();

  // The 4th and 5th byte is the temperature value
  temperature = Wire.read() << 8 | Wire.read();

  // The remaining 6 bytes is the gyroscope value
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();

  // Adjust the gyro values with the calculated offset
  if(cal_int == 2000)
  {
    gyro_x -= gyro_x_cal;
    gyro_y -= gyro_y_cal;
    gyro_z -= gyro_z_cal;
  }
}
