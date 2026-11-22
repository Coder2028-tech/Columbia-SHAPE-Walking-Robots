//Note: all of the angles shown are in the 270 degree world of the motor.
// mapping functions later on convert to the 180 degree world of the servo library (where 0 corresponds to 0 and 180 corresponds to 0 to 270)

#include <Servo.h>

// units in inches approximate
// note: switched to float for math that involves decimals.  The servo functions automatically convert to int internally
float L1 = 5.84; //length of upper leg in inches
float L2 = 6.89; //length of the lower leg in inches
float Height = 9; // this is a height in inches you choose, it should be less than the sum of the leg lengths

float HipEquil = 53; // stay in between 50-54 works best bc not leaning too far back/forward //hip angle offset from vertical for initial standing position (the equilibrium position)
float HipRange = 10; // Range over which hip moves.  It will go half of hip range on either side of equilibrium.
float HipMin = HipEquil - HipRange / 2;
float HipMax = HipEquil + HipRange / 2;

int numCycles = 5; // how many times you will loop through the code (i.e. 5 walking cycles)
int count = 1; // count will increment each pass through the loop.  Once it is > numCycles, will go to resting condition in loop
int PhaseDelay = 10; // time between each motor step.  Making this larger will move your motors slower.
float PhaseStep = .05; // redundent with the previous delay.  Leave this as is.

float KneeFlightAngle = 115; //angle of knee in flight phase, choose angle to ensure foot is off the ground for leg swing

//track if leg is in stance or flight.  If 1 it is in stance, if 0 in flight
int StanceFrontL = 1;
int StanceHindL = 1;
int StanceFrontR = 1;
int StanceHindR = 1;

Servo FrontHL; //create servo object to control your servo
int FrontHL_Pin = 7;
int FrontHL_Home = 70; //This value is where your motor is "centered".  For my robot, this is where the femur is vertical

Servo FrontKL; //create servo object to control your servo
int FrontKL_Pin = 6;
int FrontKL_Home = 180;

Servo HindHL; //create servo object to control your servo
int HindHL_Pin = 5;
int HindHL_Home = 70;

Servo HindKL; //create servo object to control your servo
int HindKL_Pin = 4;
int HindKL_Home = 180;

Servo FrontHR; //create servo object to control your servo
int FrontHR_Pin = 8;
int FrontHR_Home = 200;

Servo FrontKR; //create servo object to control your servo
int FrontKR_Pin = 9;
int FrontKR_Home = 90;

Servo HindHR; //create servo object to control your servo
int HindHR_Pin = 3;
int HindHR_Home = 200;

Servo HindKR; //create servo object to control your servo
int HindKR_Pin = 2;
int HindKR_Home = 90;

//Start the hip at its equilibrium position, inverse kinematics for knee angle to get height
float Angle_FrontHL = HipEquil * (PI / 180);
float Angle_FrontKL = Angle_FrontHL + acos((Height - L1*cos(Angle_FrontHL)) / L2);

float Angle_HindHL = HipEquil * (PI / 180);
float Angle_HindKL = Angle_HindHL + acos((Height - L1*cos(Angle_HindHL)) / L2);

float Angle_FrontHR = HipEquil * (PI / 180);
float Angle_FrontKR = Angle_FrontHR + acos((Height - L1*cos(Angle_FrontHR)) / L2);

float Angle_HindHR = HipEquil * (PI / 180); //convert to radians for acos
float Angle_HindKR = Angle_HindHR + acos((Height - L1*cos(Angle_HindHR)) / L2);

// store old value of knee angle in case you get a NaN result from the inverse kinematics
// we are assuming here that the first value you put makes sense
float Angle_FrontKL_OLD = Angle_FrontKL;
float Angle_HindKL_OLD = Angle_HindKL;
float Angle_FrontKR_OLD = Angle_FrontKR;
float Angle_HindKR_OLD = Angle_HindKR;

float Phase = 0; //variable that will go from 0 to 1

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200); //begin the serial monitor

  // Attach the servos
  FrontHL.attach(FrontHL_Pin); 
  FrontKL.attach(FrontKL_Pin);  
  HindHL.attach(HindHL_Pin);
  HindKL.attach(HindKL_Pin);
  FrontHR.attach(FrontHR_Pin); 
  FrontKR.attach(FrontKR_Pin);
  HindHR.attach(HindHR_Pin); 
  HindKR.attach(HindKR_Pin); 

  // convert back to degrees
  Angle_FrontHL = Angle_FrontHL * (180 / PI);
  Angle_FrontKL = Angle_FrontKL * (180 / PI);
  Angle_HindHL = Angle_HindHL * (180 / PI);
  Angle_HindKL = Angle_HindKL * (180 / PI);
  Angle_FrontHR = Angle_FrontHR * (180 / PI);
  Angle_FrontKR = Angle_FrontKR * (180 / PI);
  Angle_HindHR = Angle_HindHR * (180 / PI);
  Angle_HindKR = Angle_HindKR * (180 / PI);

  Serial.println("------- ENTERING HIP EQUILIBRIUM POSITION -------");

  //********* MAKE SURE INITIAL POSITION WORKS BEFORE UNCOMMENTING THE LOOP CODE *********//
  InitialPosition(); //function that sends motors to their first position
  
  Serial.println("------- COUNTDOWN TO MOTION -------");

  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
 
   if (count <= numCycles) // decide how many times the step will loop in the variables up above
   {
// )TargetAngFrontHL, TargetAngHindHL, TargetAngFrontHR, TargetAngHindHR
     Serial.println("Gait Part 1");
        StanceFrontL = 1; //change to 0 if you want a leg in "flight / swing"
        StanceHindL = 1;
        StanceFrontR = 1;
        StanceHindR = 1;
     UpdateServoPos(HipMax, HipMin, HipMin, HipMax);
 //    delay(2000); // introduced just to pause between each motion
 
     Serial.println("Gait Part 2");
        StanceFrontL = 0; //change to 0 if you want a leg in "flight / swing"
        StanceHindL = 1;
        StanceFrontR = 1;
        StanceHindR = 0;
     UpdateServoPos(HipMax, HipMin, HipMin, HipMax);
 //    delay(2000); // introduced just to pause between each motion
 
     Serial.println("Gait Part 3");
        StanceFrontL = 0; //change to 0 if you want a leg in "flight / swing"
        StanceHindL = 1;
        StanceFrontR = 1;
        StanceHindR = 0;
     UpdateServoPos(HipMin, HipMax, HipMax, HipMin);
 //    delay(2000); // introduced just to pause between each motion
 
     Serial.println("Gait Part 4");
        StanceFrontL = 1; //change to 0 if you want a leg in "flight / swing"
        StanceHindL = 1;
        StanceFrontR = 1;
        StanceHindR = 1;
     UpdateServoPos(HipMin, HipMax, HipMax, HipMin);
 //    delay(2000); // introduced just to pause between each motion
  Serial.println("Gait Part 5");
        StanceFrontL = 1; //change to 0 if you want a leg in "flight / swing"
        StanceHindL = 0;
        StanceFrontR = 0;
        StanceHindR = 1;
     UpdateServoPos(HipMin, HipMax, HipMax, HipMin);
  Serial.println("Gait Part 6");
        StanceFrontL = 1; //change to 0 if you want a leg in "flight / swing"
        StanceHindL = 0;
        StanceFrontR = 0;
        StanceHindR = 1;
     UpdateServoPos(HipMax, HipMin, HipMin, HipMax);
 
     count = count + 1; //increment number of cycles
   }
 
   else { //Once the Count is greater than numCycles (i.e. you have taken numCycles steps) go back to standing in equilibrium position
        StanceFrontL = 1; 
        StanceHindL = 1;
        StanceFrontR = 1;
        StanceHindR = 1;
     UpdateServoPos(HipEquil, HipEquil, HipEquil, HipEquil); //once loop cycles are done, return to equilibrium position
     }

}
void InitialPosition() {
  // careful with signs, will depend on how you oriented your motors
  // make sure legs are in an acceptable position when this function is called before starting motion

  // Repeat that process^ with each new additional leg.
  FrontHL.write(map(FrontHL_Home + Angle_FrontHL, 0, 270, 0, 180)); // make sure these signs make sense for each of your robot legs!
  FrontKL.write(map(FrontKL_Home - Angle_FrontKL, 0, 270, 0, 180)); // make sure these signs make sense for each of your robot legs!

  // Serial.print("KL: ");
  // Serial.print(FrontKL_Home - Angle_FrontKL);
  //   Serial.print("  KR: ");
  // Serial.print(FrontKR_Home + Angle_FrontKR);

  HindHL.write(map(HindHL_Home + Angle_HindHL, 0, 270, 0, 180)); // make sure these signs make sense for each of your robot legs!
  HindKL.write(map(HindKL_Home - Angle_HindKL, 0, 270, 0, 180)); // make sure these signs make sense for each of your robot legs!
    
  FrontHR.write(map(FrontHR_Home -  Angle_FrontHR, 0, 270, 0, 180)); // make sure these signs make sense for each of your robot legs!
  FrontKR.write(map(FrontKR_Home + Angle_FrontKR, 0, 270, 0, 180)); // make sure these signs make sense for each of your robot legs!


  HindHR.write(map(HindHR_Home - Angle_HindHR, 0, 270, 0, 180)); // make sure these signs make sense for each of your robot legs!
  HindKR.write(map(HindKR_Home + Angle_HindKR, 0, 270, 0, 180)); // make sure these signs make sense for each of your robot legs!
  // Serial.println(HindHR_Home - Angle_HindHR);
}
void UpdateServoPos(float TargetAngFrontHL, float TargetAngHindHL, float TargetAngFrontHR, float TargetAngHindHR) { //for one leg only using TargetAngHindHR
  // define start angles as the last values sent to the motors
  float FrontHL_StartAngle = Angle_FrontHL;
  float FrontKL_StartAngle = Angle_FrontKL;
  float HindHL_StartAngle = Angle_HindHL;
  float HindKL_StartAngle = Angle_HindKL;
  float FrontHR_StartAngle = Angle_FrontHR;
  float FrontKR_StartAngle = Angle_FrontKR;
  float HindHR_StartAngle = Angle_HindHR;
  float HindKR_StartAngle = Angle_HindKR;

  //NOTE: the phase loop updates all motors.  Any motor updates all should occur inside of this single for loop.
  // Do NOT create a different "phase" for loop for each leg
  for (Phase = 0; Phase <= 1; Phase = Phase + PhaseStep) //all motors start in StartAngle when phase = 0, and end in TargetAng when phase = 1
  {
    // Front Left
    Angle_FrontHL =  FrontHL_StartAngle + Phase * (TargetAngFrontHL - FrontHL_StartAngle); // linear interpolation from StartAngle to TargetAng
    Angle_FrontHL = Angle_FrontHL * (PI / 180); // put hip angle in radians
    if (StanceFrontL == 1) { // if in stance, do inverse kinematics to find knee angle
      Angle_FrontKL_OLD = Angle_FrontKL; //store previous value prior to doing math
      Angle_FrontKL = Angle_FrontHL + acos((Height - L1 * cos(Angle_FrontHL)) / L2);
      Angle_FrontKL = Angle_FrontKL * (180 / PI); // put knee angle in degrees
      if (isnan(Angle_FrontKL)) { // if impossible angle suggested for knee, send it back to its previous recorded angle
        Serial.println("NaN on Front Left knee - holding last angle");
        Angle_FrontKL = Angle_FrontKL_OLD;
      }
    }
    if (StanceFrontL == 0) { //if not in stance, put knee in the configuration for swing
      Angle_FrontKL = FrontKL_StartAngle + Phase * (KneeFlightAngle - FrontKL_StartAngle);
      Serial.println("Front Left Knee UP");
    }
    Angle_FrontHL = Angle_FrontHL * (180 / PI); //put hip angle back in degrees
    // Hind Left
    Angle_HindHL =  HindHL_StartAngle + Phase * (TargetAngHindHL - HindHL_StartAngle); // linear interpolation from StartAngle to TargetAng
    Angle_HindHL = Angle_HindHL * (PI / 180); // put hip angle in radians
    if (StanceHindL == 1) { // if in stance, do inverse kinematics to find knee angle
      Angle_HindKL_OLD = Angle_HindKL; //store previous value prior to doing math
      Angle_HindKL = Angle_HindHL + acos((Height - L1 * cos(Angle_HindHL)) / L2);
      Angle_HindKL = Angle_HindKL * (180 / PI); // put knee angle in degrees
      if (isnan(Angle_HindKL)) { // if impossible angle suggested for knee, send it back to its previous recorded angle
        Angle_HindKL = Angle_HindKL_OLD;
      }
    }
    if (StanceHindL == 0) { //if not in stance, put knee in the configuration for swing
      Angle_HindKL = HindKL_StartAngle + Phase * (KneeFlightAngle - HindKL_StartAngle);
    }
    Angle_HindHL = Angle_HindHL * (180 / PI); //put hip angle back in degrees
  // Front Right
    Angle_FrontHR =  FrontHR_StartAngle + Phase * (TargetAngFrontHR - FrontHR_StartAngle); // linear interpolation from StartAngle to TargetAng
    Angle_FrontHR = Angle_FrontHR * (PI / 180); // put hip angle in radians
    if (StanceFrontR == 1) { // if in stance, do inverse kinematics to find knee angle
      Angle_FrontKR_OLD = Angle_FrontKR; //store previous value prior to doing math
      Angle_FrontKR = Angle_FrontHR + acos((Height - L1 * cos(Angle_FrontHR)) / L2);
      Angle_FrontKR = Angle_FrontKR * (180 / PI); // put knee angle in degrees
      if (isnan(Angle_FrontKR)) { // if impossible angle suggested for knee, send it back to its previous recorded angle
        Angle_FrontKR = Angle_FrontKR_OLD;
      }
    }
    if (StanceFrontR == 0) { //if not in stance, put knee in the configuration for swing
      Angle_FrontKR = FrontKR_StartAngle + Phase * (KneeFlightAngle - FrontKR_StartAngle);
    }
    Angle_FrontHR = Angle_FrontHR * (180 / PI); //put hip angle back in degrees
  // Hind Right
    Angle_HindHR =  HindHR_StartAngle + Phase * (TargetAngHindHR - HindHR_StartAngle); // linear interpolation from StartAngle to TargetAng
    Angle_HindHR = Angle_HindHR * (PI / 180); // put hip angle in radians
    if (StanceHindR == 1) { // if in stance, do inverse kinematics to find knee angle
      Angle_HindKR_OLD = Angle_HindKR; //store previous value prior to doing math
      Angle_HindKR = Angle_HindHR + acos((Height - L1 * cos(Angle_HindHR)) / L2);
      Angle_HindKR = Angle_HindKR * (180 / PI); // put knee angle in degrees
      if (isnan(Angle_HindKR)) { // if impossible angle suggested for knee, send it back to its previous recorded angle
        Angle_HindKR = Angle_HindKR_OLD;
      }
    }
    if (StanceHindR == 0) { //if not in stance, put knee in the configuration for swing
      Angle_HindKR = HindKR_StartAngle + Phase * (KneeFlightAngle - HindKR_StartAngle);
    }
    Angle_HindHR = Angle_HindHR * (180 / PI); //put hip angle back in degrees
    // for additional leg motors, the phase code should go HERE
    // transform angles back to motor coordinates for write command
    FrontHL.write(map(FrontHL_Home + Angle_FrontHL, 0, 270, 0, 180));   //Make sure the signs here match the signs in InitialPosition
    FrontKL.write(map(FrontKL_Home - Angle_FrontKL, 0, 270, 0, 180));   //Make sure the signs here match the signs in InitialPosition
    HindHL.write(map(HindHL_Home + Angle_HindHL, 0, 270, 0, 180));   //Make sure the signs here match the signs in InitialPosition
    HindKL.write(map(HindKL_Home - Angle_HindKL, 0, 270, 0, 180));   //Make sure the signs here match the signs in InitialPosition
    FrontHR.write(map(FrontHR_Home - Angle_FrontHR, 0, 270, 0, 180));   //Make sure the signs here match the signs in InitialPosition
    FrontKR.write(map(FrontKR_Home + Angle_FrontKR, 0, 270, 0, 180));   //Make sure the signs here match the signs in InitialPosition
    HindHR.write(map(HindHR_Home - Angle_HindHR, 0, 270, 0, 180));   //Make sure the signs here match the signs in InitialPosition
    HindKR.write(map(HindKR_Home + Angle_HindKR, 0, 270, 0, 180));   //Make sure the signs here match the signs in InitialPosition
    delay(PhaseDelay);
  }
}
