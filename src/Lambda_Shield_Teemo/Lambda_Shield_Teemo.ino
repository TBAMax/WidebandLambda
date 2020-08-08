// firmware for TeemoBW lambda shield
// Teemo Vaas 7.2020

#include <SPI.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>

//Define CJ125 registers used.
#define           CJ125_IDENT_REG_REQUEST             0x4800        /* Identify request, gives revision of the chip. */
#define           CJ125_DIAG_REG_REQUEST              0x7800        /* Dignostic request, gives the current status. */
#define           CJ125_INIT_REG1_REQUEST             0x6C00        /* Requests the first init register. */
#define           CJ125_INIT_REG2_REQUEST             0x7E00        /* Requests the second init register. */
#define           CJ125_INIT_REG1_MODE_CALIBRATE      0x569D        /* Sets the first init register in calibration mode. */
#define           CJ125_INIT_REG1_MODE_NORMAL_V8      0x5688        /* Sets the first init register in operation mode. V=8 amplification. */
#define           CJ125_INIT_REG1_MODE_NORMAL_V17     0x5689        /* Sets the first init register in operation mode. V=17 amplification. */
#define           CJ125_DIAG_REG_STATUS_OK            0x28FF        /* The response of the diagnostic register when everything is ok. */
#define           CJ125_DIAG_REG_STATUS_NOPOWER       0x2855        /* The response of the diagnostic register when power is low. */
#define           CJ125_DIAG_REG_STATUS_NOSENSOR      0x287F        /* The response of the diagnostic register when no sensor is connected. */
#define           CJ125_INIT_REG1_STATUS_0            0x2888        /* The response of the init register when V=8 amplification is in use. */
#define           CJ125_INIT_REG1_STATUS_1            0x2889        /* The response of the init register when V=17 amplification is in use. */

//Define output pin assignments.
//Pins 11;12;13 are reserved for hardware SPI
//Pins 
#define           CJ125_NSS_PIN                       10            /* Pin used for chip select in SPI communication. */
#define           LED_STATUS                          8             /* Pin used for the status LED*/
#define           HEATER_OUTPUT_PIN                   3             /* Pin used for the PWM output to the heater circuit. */
#define           NB_OUTPUT_PIN                       A3             /* Pin used for the PWM to the 0-1V analog output. */
#define           NB_OUTPUT_PIN2                      A4
#define           ANALOG_OUTPUT_PIN                   9             //Pin used for high resolution PWM output

const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 15, d7 = 2;          // define pins used for LCD
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);// initialize LCD library by associating any needed LCD interface pin

//define analog input pin assignments
#define           UB_ANALOG_INPUT_PIN                 A2             /* Analog input for power supply.*/
#define           UR_ANALOG_INPUT_PIN                 A6             /* Analog input for sensor temperature.*/
#define           UA_ANALOG_INPUT_PIN                 A7             /* Analog input for lambda.*/

//define digital input pin assignments
#define           BUTTON_PIN                          A5

//Define adjustable parameters.                       
#define           UBAT_MIN_ADC                        330           /* about 7V  Minimum voltage (ADC value) on Ubat to operate */
#define           MAX_PWM                             163           //maximum pwm value for the heater
#define           HEATER_SETPOINT                     780           //Heating temprature goal


typedef enum{
  CONDENSATION,
  RAMP_UP,
  PID_CONTROL,
  OFF,
  HEATER_ERROR
} heaterState;


//Global variables.
int adcValue_UA = 0;                              /* ADC value read from the CJ125 UA output pin */ 
int adcValue_UR = 0;                              /* ADC value read from the CJ125 UR output pin */
int battADCvalue = 0;
//int adcValue_UB = 0;                              /* ADC value read from the voltage divider caluclating Ubat */
int adcValue_UA_Calibration = 0;                                    /* UA ADC value stored when CJ125 is in calibration mode, λ=1 */ 
int adcValue_UR_Calibration= 0;                                     /* UR ADC value stored when CJ125 is in calibration mode, optimal temperature 780C */
int UA_correction=0;
int UR_correction=0;
unsigned int heaterOutput = 0;                                       /* Current PWM output value (0-1023) of the heater output pin */
unsigned long previousMillis = 0;
unsigned long previousMillisHeater = 0;
int lambda=0;                        //measured lambda value*1000
int heaterTemperature=0;             //measures heater temperature value
uint16_t CJ125_Status;

//Lambda table. Calculated from table in the file:Data sheet_69034379_Lambda_Sensor_LSU_4.9.pdf
const unsigned int UaToLambda[2][22]={    //[row][column]
{39,108,135,167,220,268,284,299,307,310,328,349,361,378,452,509,555,606,674,738,770,792}, //Ua ADC value Vref5V, 10bit resolution
{750,800,822,850,900,950,970,990,1003,1010,1050,1100,1132,1179,1429,1701,1990,2434,3413,5391,7506,10119} //lambda*1000
};

//Temperature table. Compiled from LSU4.9 and CJ125 datasheets
const unsigned int  UrToTemperature[2][7]={
{0,630,780,900,990,1100,1200},  //Sensor temperature
{1023,562,211,135,110,95,89}       //ADC Ur (5V reference) 10bit resolution
};

//Define Variables we'll be connecting to PID
double PIDsetpoint, PIDinput, PIDoutput;

//Specify the links and initial tuning parameters
double Kp=1, Ki=0.4, Kd=0.4;
PID myPID(&PIDinput, &PIDoutput, &PIDsetpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  delay(500);
//  Serial.begin(9600);   //Set up serial communication.

  lcd.begin(16, 2);    // set up the LCD's number of columns and rows:
  lcd.clear();
    
  //Set up SPI.
  SPI.begin();  /* Note, SPI will disable the bult in LED. */
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);

   // Set up pin 9 for fastPWM
  pinMode(9,OUTPUT);
  TCCR1B=0b00001001; //set up fast PWM with 10bit resolution @about 15kHz
  TCCR1A=0b10000011; //and output to OC1A pin (D9)
  
  //Set up digital output pins.
  pinMode(CJ125_NSS_PIN, OUTPUT);  
  pinMode(LED_STATUS, OUTPUT);
  pinMode(HEATER_OUTPUT_PIN, OUTPUT);
  pinMode(A1, OUTPUT);
  //Set up input pins.
  pinMode(UB_ANALOG_INPUT_PIN, INPUT);
  pinMode(UR_ANALOG_INPUT_PIN, INPUT);
  pinMode(UA_ANALOG_INPUT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);

  //Set initial values.
  digitalWrite(CJ125_NSS_PIN, HIGH);
  digitalWrite(LED_STATUS, LOW);
  analogWrite(HEATER_OUTPUT_PIN, 0); /* PWM is initially off. */
  analogWrite(NB_OUTPUT_PIN, 0); /* PWM is initially off. */
    
  //Start of operation. (Test LED's).
//  Serial.print("Device reset.\n\r");
  digitalWrite(LED_STATUS, HIGH);
  delay(200);
  digitalWrite(LED_STATUS, LOW);
  
  // Print a message to the LCD.
//  lcd.setCursor(0, 0);
//  lcd.print("hello, world!");
  calibrateCJ125();  
}

void calibrateCJ125(){
  int i = 0;
  
  lcd.setCursor(0, 0);
  lcd.print("Batt ADC=");
  lcd.print(battADCvalue);
  lcd.print("    ");
  lcd.setCursor(0, 1);  //row2
  lcd.print("CJ125id=");
  CJ125_Status = COM_SPI(CJ125_IDENT_REG_REQUEST);
  lcd.print(CJ125_Status,HEX);                       //display CJ125 identification byte
  delay(3000);
  lcd.setCursor(0, 1);  //row2
  lcd.print("diag=");
  CJ125_Status = COM_SPI(CJ125_DIAG_REG_REQUEST);
  lcd.print(CJ125_Status,HEX);                      //display CJ125 diagnostics register
  lcd.print("     ");
  battADCvalue=analogRead(UB_ANALOG_INPUT_PIN);
  delay(1000);
  battADCvalue = analogRead(UB_ANALOG_INPUT_PIN);
  while (battADCvalue < UBAT_MIN_ADC || (CJ125_Status&0xFF) != (CJ125_DIAG_REG_STATUS_OK&0xFF)) {
    //Read battery voltage.
    battADCvalue = analogRead(UB_ANALOG_INPUT_PIN);
    lcd.setCursor(9, 0);    
    lcd.print(battADCvalue);
    //Read CJ125 diagnostic register from SPI.
    CJ125_Status = COM_SPI(CJ125_DIAG_REG_REQUEST);
    lcd.setCursor(5, 1);  //row2
    lcd.print(CJ125_Status,HEX);                      //display CJ125 diagnostics register    
    delay(200);
  }
  digitalWrite(LED_STATUS, HIGH);
    //Set CJ125 in calibration mode.
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibrating!");
  COM_SPI(CJ125_INIT_REG1_MODE_CALIBRATE);
    //Let values settle.
  delay(500);
  
  //Store optimal values before leaving calibration mode.
  adcValue_UA_Calibration=0;                                   //simple average filter
    for(i=0;i<8;i++){                              
  adcValue_UA_Calibration += analogRead(UA_ANALOG_INPUT_PIN);  //lambda
  }
  adcValue_UA_Calibration=adcValue_UA_Calibration/8;

  adcValue_UR_Calibration=0;                                   //simple average filter
    for(i=0;i<8;i++){                              
  adcValue_UR_Calibration += analogRead(UR_ANALOG_INPUT_PIN);  //temperature
  }
  adcValue_UR_Calibration=adcValue_UR_Calibration/8;

  UA_correction=304-adcValue_UA_Calibration;                   //calculate corrections
  UR_correction=211-adcValue_UR_Calibration;

  //Set CJ125 back in normal operation mode.
  COM_SPI(CJ125_INIT_REG1_MODE_NORMAL_V17);
  lcd.setCursor(0, 1);
  lcd.print("Calibrated!");
  delay(500);
  lcd.setCursor(0, 0);
  lcd.print("UA_cal(304):");
  lcd.print(adcValue_UA_Calibration);
  lcd.print("    ");
  lcd.setCursor(0, 1);
  lcd.print("UR_cal(211):");
  lcd.print(adcValue_UR_Calibration);
  lcd.print("    ");
  delay(5000);
}  //end of calibration


//Infinite loop.
void loop() {
  int i;
  float lambdaPoint;  //for lambda printing 
  
  //Update analog inputs.
  adcValue_UA=0;
  adcValue_UR=0;
  
  for(i=0;i<8;i++){                              //simple average filter
  adcValue_UA+=analogRead(UA_ANALOG_INPUT_PIN);  //lambda
  }
  adcValue_UA=adcValue_UA/8;
  adcValue_UA+=UA_correction; //apply correction
  
  for(i=0;i<8;i++){                              //simple average filter  
  adcValue_UR+=analogRead(UR_ANALOG_INPUT_PIN);  //heater resistance
  }
  adcValue_UR=adcValue_UR/8;
  adcValue_UR+=UR_correction; //apply correction

  lambda=getLambda(adcValue_UA);
  heaterTemperature=getTemperature(adcValue_UR);
  
   //update display once in every 1 sec.
   unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 1000) {
    // save the last time updated display
    previousMillis = currentMillis;
    battADCvalue=analogRead(UB_ANALOG_INPUT_PIN);  //battery
    lambdaPoint=lambda/1000;

    lcd.setCursor(0, 0);
    lcd.print("Lambda=");
    lcd.print(lambdaPoint);
    lcd.print("@"); //clear the rest
    lcd.print(adcValue_UA);

    lcd.setCursor(0, 1);
    lcd.print("Tmp=");
    lcd.print(heaterTemperature);
    lcd.print("(");
    lcd.print(adcValue_UR);
    lcd.print(")");
    lcd.print(heaterOutput);
    lcd.print(" ");
    
    }
  //Update CJ125 diagnostic register from SPI. And check for errors
  CJ125_Status = COM_SPI(CJ125_DIAG_REG_REQUEST);
  if((CJ125_Status&0xFF) == (CJ125_DIAG_REG_STATUS_NOPOWER&0xFF)){  //check errors (only account the LSB because MSB can vary actually, that is why &0xFF)
    lcd.setCursor(0, 0);
    lcd.print("Sensor pwr low!");
    delay(500);
  }
  if ((CJ125_Status&0xFF) ==(CJ125_DIAG_REG_STATUS_NOSENSOR&0xFF)){
    lcd.setCursor(0, 0);
    lcd.print("Sensor disconeected!");
    delay(500);
  }

  heaterControl();

  analogWrite10bitD9(map(lambda,750,1500,0,1023));  //output lambda to the analog PWM pin.
}

void heaterControl(){
  static heaterState state=CONDENSATION;   //initialise to condensation regime
  unsigned long currentMillis =millis();  
  
  switch(state){ 
     case CONDENSATION:
        if( battADCvalue > UBAT_MIN_ADC && (CJ125_Status&0xFF) == (CJ125_DIAG_REG_STATUS_OK&0xFF)){
          if (heaterTemperature < 781){
             heaterOutput=6;                           //fixed power of less than 1.5V                      
          }
          if(currentMillis -previousMillisHeater >= 60000){   //duration 60s form program start
             previousMillisHeater=currentMillis;
             state=RAMP_UP;
             heaterOutput=12;    //ramp initial value
          } 
        }
        else{
          heaterOutput=0;        
        }    
        break;
     case RAMP_UP:
        if( battADCvalue > UBAT_MIN_ADC && ((CJ125_Status&0xFF) == (CJ125_DIAG_REG_STATUS_OK&0xFF))){           
           if(currentMillis -previousMillisHeater >= 300){   //gradually increase pwm
              previousMillisHeater=currentMillis;
              if(heaterOutput<MAX_PWM) heaterOutput++;   //limit maximum to 12Veff @15V DC
              if (heaterTemperature >= HEATER_SETPOINT ){
                 state=PID_CONTROL;          //temperature reached move to PID control
                   //turn the PID on
                 PIDinput=heaterTemperature;
                 PIDsetpoint=HEATER_SETPOINT;
                 PIDoutput=heaterOutput;
                 myPID.SetOutputLimits(0, MAX_PWM);
                 myPID.SetMode(AUTOMATIC);
              }
           }
        }             
        else{
          heaterOutput=0;   //in case something wrong:shut down the heater power
        }
        break;
     case PID_CONTROL:
        if( battADCvalue > UBAT_MIN_ADC && ((CJ125_Status&0xFF) == (CJ125_DIAG_REG_STATUS_OK&0xFF))){
          PIDinput=heaterTemperature;
          myPID.Compute();
          heaterOutput=PIDoutput;
        }
        else{
          heaterOutput=0;       
        }       
        break;
  }
  analogWrite(HEATER_OUTPUT_PIN,heaterOutput);
}


//This function returns lambda value multiplied with 1000
int getLambda(int ADCvalue){       //(function tested on AVR with special separate program)
   int i;
   int lambda;
   i=0;
   while(i<22){
        if(adcValue_UR <= UaToLambda[0][i]){
          break;
          }
        i++;
   }
   if (i==0) {
    lambda=0;                    //min value(out of range)
    return lambda;
    }
   else if(i>=22){
    lambda=UaToLambda[1][21];    //max value
    return lambda;
    }
   else{                         //from table
    lambda=interpolate(adcValue_UR,UaToLambda[0][i-1],UaToLambda[1][i-1],UaToLambda[0][i],UaToLambda[1][i]);
    return lambda;
   }
}

//this function returns ceramic temperature value in Celsius degrees
int getTemperature(int adcValue_UR){        //(function tested on AVR with special separate program)
   int i;
   int temp;
   i=0;
   while(i<7){
        if(adcValue_UR >= UrToTemperature[1][i]){
          break;
          }
        i++;
   }
   if (i==0) {
    temp=0;                        //min value(out of range)
    return temp;
    }
   else if(i>=7){
      temp=UrToTemperature[0][6];  //max value
    return temp;
    }
   else{                           //from table
    temp=interpolate(adcValue_UR,UrToTemperature[1][i-1],UrToTemperature[0][i-1],UrToTemperature[1][i],UrToTemperature[0][i]);
    return temp;
   }
}

//function for simple linear interpolation... well actually this is same as Arduinu map() function but I leave it in
int interpolate(long x,long xa, long ya,long xb,long yb){ 
  int y;
  y=ya+((yb-ya)*(x-xa)/(xb-xa));
  return y;
}

//Function for transfering SPI data to the CJ125.
uint16_t COM_SPI(uint16_t TX_data) {

  //Set chip select pin low, chip in use.
  digitalWrite(CJ125_NSS_PIN, LOW);

  //Transmit and receive.
  byte highByte = SPI.transfer(TX_data >> 8);
  byte lowByte = SPI.transfer(TX_data & 0xff);

  //Set chip select pin high, chip not in use.
  digitalWrite(CJ125_NSS_PIN, HIGH);

  //Assemble response in to a 16bit integer and return the value.
  uint16_t Response = (highByte << 8) + lowByte;
  return Response;
  
}
void analogWrite10bitD9(unsigned int value10bit){
  OCR1A=value10bit;
}
