/******************************************************************
   PID Simple Example (Augmented with Processing.org Communication)
   Version 0.3
   by Brett Beauregard
   License: Creative-Commons Attribution Share-Alike
   April 2011
 ******************************************************************/
/* Alterado por Josenalde Oliveira em 18.04.2018 */
#include <PID_v1.h>

//Define Variables we'll be connecting to
const int VCC = 5;
double Setpoint, rawInput, Output;
double  voltageInput;
double KP, KI, KD;
int inputPin = 0; //A0 for sensor pin (pv sensor)
int outputPin = 3; //D3~ for pwm control signal
int pausePlay = 0;
// int setpointPin = 1; // for analogic set point A1 (potentiometer)


//Specify the links and initial tuning parameters
PID myPID(&rawInput, &Output, &Setpoint, 2.43, 4.86, 0.1, DIRECT);

unsigned long serialTime; //this will help us know when to talk with processing

void setup()
{
  //initialize the serial link with processing
  Serial.begin(9600);

  //initialize the variables we're linked to
  rawInput = 0;
  //voltageInput = (VCC/1024)*rawInput;
  Setpoint = 400;
  //Setpoint = analogRead(setpointPin);
  KP = 2.43; 
  KI = 4.86;
  KD = 0.1;
  //turn the PID on
  myPID.SetMode(MANUAL);
}

void loop()
{
  //pid-related code
  rawInput = analogRead(inputPin);
  //voltageInput = (VCC/1024)*rawInput;
  myPID.Compute();
  analogWrite(outputPin, Output);
  //send-receive with processing if it's time

  if (millis() > serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime += 100;
  }

}


/********************************************
   Serial Communication functions / helpers
 ********************************************/


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{

  // read the bytes sent from Processing
  int index = 0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  while (Serial.available() && index < 26)
  {
    if (index == 0) Auto_Man = Serial.read();
    else if (index == 1) Direct_Reverse = Serial.read();
    else foo.asBytes[index - 2] = Serial.read();
    index++;
  }

  // if the information we got was in the correct format,
  // read it into the system
  if (index == 26  && (Auto_Man == 0 || Auto_Man == 1) && (Direct_Reverse == 0 || Direct_Reverse == 1))
  {
    Setpoint = double(foo.asFloat[0]);
    //Input=double(foo.asFloat[1]);       // * the user has the ability to send the
    //   value of "Input"  in most cases (as
    //   in this one) this is not needed.
    if (Auto_Man == 0)                    // * only change the output if we are in
    { //   manual mode.  otherwise we'll get an
      Output = double(foo.asFloat[2]);    //   output blip, then the controller will
    }                                     //   overwrite.

                           // * read in and set the controller tunings
    KP = double(foo.asFloat[3]);           //
    KI = double(foo.asFloat[4]);           //
    KD = double(foo.asFloat[5]);           //
    myPID.SetTunings(KP, KI, KD);            //

    if (Auto_Man == 0) myPID.SetMode(MANUAL); // * set the controller mode
    else myPID.SetMode(AUTOMATIC);             //

    if (Direct_Reverse == 0) myPID.SetControllerDirection(DIRECT); // * set the controller Direction
    else myPID.SetControllerDirection(REVERSE);          //
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
  Serial.print("PID ");
  Serial.print(Setpoint);
  Serial.print(" ");
  Serial.print(rawInput);
  Serial.print(" ");
  Serial.print(Output);
  Serial.print(" ");
  Serial.print(myPID.GetKp());
  Serial.print(" ");
  Serial.print(myPID.GetKi());
  Serial.print(" ");
  Serial.print(myPID.GetKd());
  Serial.print(" ");
  if (myPID.GetMode() == AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");
  Serial.print(" ");
  if (myPID.GetDirection() == DIRECT) Serial.println("Direct");
  else Serial.println("Reverse");
}

