#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHRCS.h>
#include <FEHMotor.h>
#include "FEHLCD.h"
#include <time.h>
#include <FEH.h>
#include <Arduino.h>

#define ENCODER_MAX 48
#define WHEEL_RADIUS 1.15
#define ANY_LIGHT_VALUE 0.45
#define RED_LIGHT_THRESHOLD 1.3
#define NINETYDEG_COUNTS 48
#define DEFAULT_SPEED 29
#define TURNING_SPEED 23
#define RCS_WAIT_TIME_IN_SEC 0.2

// Defines for pulsing the robot
#define PULSE_TIME 0.2
#define PULSE_POWER -13

// Define for the motor power
#define POWER -20

// Orientation of AruCo Code
#define FORWARDS 0
#define BACKWARDS 1

// heading keywords
#define N 90
#define NE 
#define NW 135
// need special code to handle 0
#define E 0
#define S 270
#define SE 
#define SW 
#define W 180

DigitalEncoder right_encoder(FEHIO::Pin8);
DigitalEncoder left_encoder(FEHIO::Pin10);
FEHMotor right_motor(FEHMotor::Motor1,9.0);
FEHMotor left_motor(FEHMotor::Motor2,9.0);
FEHServo armServo(FEHServo::Servo0);
AnalogInputPin lightSensor(FEHIO::Pin1);

void turn(int percent, int counts);
void moveForwardsNoEncoder(int speed, float time);
void moveForwardEncoder(int percent, int counts);
void detectStart();

// converts inches input into counts for shaft encoders
int getCounts(float dist){
    return (ENCODER_MAX * dist) / (2 * PI * WHEEL_RADIUS);
}

// smoothly move arm ( cannot have currentdegree = targetDegree)
void moveArm(int currentDegree, int targetDegree, int speedDiv){
    int direction = 1; //1 = up, -1 = down
    if(targetDegree < currentDegree){
        direction = -1;
    }
    for(int i = currentDegree + 1 * direction; abs(targetDegree-i) > 0; i+=(1*direction)){
        armServo.SetDegree(i);
        Sleep(0.025/speedDiv);
    }
    Sleep(0.3);

}

// pauses until the start light haas been detected by the CdS Cell
void detectStart(){
    Sleep(0.1);
        while((lightSensor.Value() > ANY_LIGHT_VALUE));
            LCD.Clear();
            moveForwardsNoEncoder(-35, 0.75);
            moveForwardEncoder(35, getCounts(1.6));
}



// gets the number of counts for the robot to move to the specified robots
// if x or y are sent in as 0, set them to the rcs pos
int getXYCountsRCS(float x, float y){

    Sleep(0.30);
    RCSPose* pos = RCS.RequestPosition();
    int count = 0;
    while(pos->x <= 0){
        Sleep(0.30);
        pos = RCS.RequestPosition();
        LCD.SetFontColor(RED);
        LCD.WriteRC("NEGATIVE RCS VALUES", 5, 5);
        LCD.SetFontColor(WHITE);
        count++;
        if(count > 7){
            LCD.SetFontColor(RED);
            LCD.WriteRC("TOO MANY INVALID RCS VALUES, RETURN 0", 5, 5);
            LCD.SetFontColor(WHITE);
            return 0;
        }
    }
    
    LCD.Clear();
    LCD.WriteLine("RCS X:");
    LCD.WriteLine(pos->x);
    LCD.WriteLine("RCS Y:");
    LCD.WriteLine(pos->y);
    LCD.Clear();
    if(x == 0){
        x = pos->x;
    }
    if(y == 0){
        y = pos->y;
    }
    float underSqrt = pow((x - pos->x), 2) + pow((y - pos->y), 2.0);
    float hyp = sqrt(underSqrt);
    LCD.WriteLine("DISTANCE:");
    LCD.WriteLine(hyp);
    LCD.WriteLine("COUNTS:");
    LCD.WriteLine(getCounts(hyp));
    return getCounts(hyp);
}

// gives the rotation counts for the most optimal rotation direction to the given heading from the current heading
// this seems to be off slightly
int getHeadingCounts(float heading){
    //90, given 135
    Sleep(0.1);
    RCSPose* pos = RCS.RequestPosition();
    int count = 0;
    while(pos->x <= 0){
        Sleep(0.30);
        pos = RCS.RequestPosition();
        LCD.SetFontColor(RED);
        LCD.WriteRC("NEGATIVE RCS VALUES", 5, 5);
        LCD.SetFontColor(WHITE);
        count++;
        if(count > 7){
            LCD.SetFontColor(RED);
            LCD.WriteRC("TOO MANY INVALID RCS VALUES, RETURN 0", 5, 5);
            LCD.SetFontColor(WHITE);
            return 0;
        }
    }
    float degrees = 0;
    //(90 - 135 = 45)
    if(abs(pos->heading - heading) > 180){  // passing 0
        if(pos->heading > heading){// turn counter-clockwise
            degrees = (360 - pos->heading) + heading;
        }else{ 
            degrees = (360 - heading) + pos->heading; 
        }
    } else{ // turn clockwise
        degrees = abs(pos->heading - heading); // 135 - 90 = 45
    }

    // 300 -> 40 = 100 degree turn
    int counts = (degrees/90.0) * NINETYDEG_COUNTS;
    LCD.WriteLine("HEADING CURRENT");
    LCD.WriteLine(pos->heading);
    LCD.WriteLine("HEADING FINAL");
    LCD.WriteLine(heading);
    LCD.WriteLine("COUNTS");
    LCD.WriteLine(counts);
    return counts;
}

// ONLY CALL WITH THE CARIDNAL DIRECTIONS OR JUST NOT SOMETHING CLOSE TO 0!
void headingCorrection(float heading){
    RCSPose* pos = RCS.RequestPosition();
    // account for exactly 0
    if(heading == 0){
        if(pos->heading < 358){
            turn(-TURNING_SPEED, getHeadingCounts(heading));
        } else if(pos->heading > 2){
            turn(TURNING_SPEED, getHeadingCounts(heading));
        }
    }
    if(pos->heading < heading - 2){
        turn(-TURNING_SPEED, getHeadingCounts(heading));
    } else if(pos->heading > heading + 2){
        turn(TURNING_SPEED, getHeadingCounts(heading));
    }
}
/*
// TODO: COMPLETE THIS FUNCTION
int getHeadingCounts(float x, float y){
    RCSPose* pos = RCS.RequestPosition();

    float radians = atan((pos->y - y)/ (pos->x - x));
    return radians;
}
    */ 


// writes the value that the CdS cell senses
void writeLight(){
        LCD.SetFontSize(5);
        int x, y;
        while(!LCD.Touch(&x,&y)){ //Wait for screen to be pressed
            
            LCD.Write(lightSensor.Value());
            Sleep(0.5);
            LCD.Clear();
        
        }
}
// moves until either encoder his the specified count
void moveForwardEncoder(int percent, int counts) {
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    right_motor.SetPercent(percent);
    left_motor.SetPercent(-1 * percent);

    // maybe take the larger? idk
    // have it stop when it stops spinning/ hits something
    while(right_encoder.Counts() <= counts || left_encoder.Counts() <= counts){
        // LCD.Write("Actual LE Counts: ");
        // LCD.WriteLine(left_encoder.Counts());
        // LCD.Write("Actual RE Counts: ");
        // LCD.WriteLine(right_encoder.Counts());
        // LCD.Clear();
    }
        // LCD.Write("Actual LE Counts: ");
        // LCD.WriteLine(left_encoder.Counts());
        // LCD.Write("Actual RE Counts: ");
        // LCD.WriteLine(right_encoder.Counts());

    right_motor.Stop();
    left_motor.Stop();
}

// positive percent = turn right, negative percent = turn left
// turning 90 is approx 3/4 of full rotation 
// so w/ 48, 90deg is 36 counts
// IS 45deg 3/8? if so then its 18 counts for 45deg
void turn(int percent, int counts) {


    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    while(right_encoder.Counts() <= counts || left_encoder.Counts() <= counts){
        right_motor.SetPercent(-1* percent);
        left_motor.SetPercent(-1* percent); 
    }
    
    right_motor.Stop();
    left_motor.Stop();

}

void moveForwardsNoEncoder(int speed, float time){
    right_motor.SetPercent(speed);
    left_motor.SetPercent(-1 * speed);
    Sleep(time);
    right_motor.Stop();
    left_motor.Stop();
}

// gets the number of counts for the given distance in inches
// waits for the given number of seconds
boolean displayLightColor(float seconds){
    boolean red = false;
    int starttime = time(NULL);
    while(time(NULL) - starttime <= seconds){
        if(lightSensor.Value() > RED_LIGHT_THRESHOLD){
            LCD.SetFontColor(RED);
            LCD.FillRectangle(0, 0, 319, 239);
            Sleep(1.5);
            red = true;
        } else{
            LCD.SetFontColor(BLUE);
            LCD.FillRectangle(0, 0, 319, 239);
            Sleep(1.5);
        }
    }
    return red;
}

void testServo(){
    armServo.TouchCalibrate();
}

void ERCMain(){

    // RCS STUFF
    RCS.InitializeTouchMenu("0150F4CPJ");
    //TODO: remove rate delimiter
    RCS.DisableRateLimit();
    //************************* */
    WaitForFinalAction();

    armServo.SetMin(500);
    armServo.SetMax(2500);

    //0 = left, 1 = mid, 2 = right
    //int lever = RCS.GetLever();

    int x, y;
    // encoder distance calc
    // wheel radius = 1.05 
    // 0.435 inches tread - or more likely 0.1 inches
    // total experimental radius is  1.15 inches
    // wheel radius of 1.15
    // 48 counts per full rotation
    // make a method to convert inches to counts
    // dist = (2pi*radius*CountsRecorded) / Counts per rev
    // dist = (2pi * 1.15 * n) / 48


    // QUALIFIERS
    detectStart();
    // Milestone 5 code to spin compost bin
    turn(-TURNING_SPEED, getHeadingCounts(202));
    //headingCorrection(193);
    // put arm in down position
    int lowerPos = 185, highPos = 102;
    // move to bin
    //moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(21.7, 4.0));
    moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(24, 0));
    //moveForwardEncoder(DEFAULT_SPEED, getCounts(7.0));
    // turn clockwise, starting from the high pos
    int armSpeedDiv = 18;
    float moveDist = 1.5;
    // make these into function calls to prevent copied code
    moveArm(highPos, lowerPos, armSpeedDiv);
    moveForwardEncoder(-DEFAULT_SPEED, getCounts(moveDist));
    moveArm(lowerPos, highPos, armSpeedDiv);
    moveForwardEncoder(DEFAULT_SPEED, getCounts(moveDist));
    moveArm(highPos, lowerPos, armSpeedDiv);
    moveForwardEncoder(-DEFAULT_SPEED, getCounts(moveDist));
    moveArm(lowerPos, highPos, armSpeedDiv);
    moveForwardEncoder(DEFAULT_SPEED, getCounts(moveDist));
    moveArm(highPos, lowerPos, armSpeedDiv);
    moveForwardEncoder(-DEFAULT_SPEED, getCounts(moveDist));
    moveArm(lowerPos, highPos, armSpeedDiv);
    moveForwardEncoder(DEFAULT_SPEED, getCounts(moveDist));
    moveArm(highPos, lowerPos, armSpeedDiv);
    moveArm(lowerPos, highPos, armSpeedDiv);
    // turn to apple bucket
    int appleLowPos = 157, appleHighPos = 105, appleSpeed = 2;
    turn(TURNING_SPEED, NINETYDEG_COUNTS/2);
    moveForwardEncoder(DEFAULT_SPEED, getCounts(0.5));
    // Call RCS to correct heading and position
    // offset turning slightly to not go on the ramp
    turn(TURNING_SPEED, getHeadingCounts(100));
    moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(0, 19.1));
    // turn to bucket
    turn(-TURNING_SPEED, (NINETYDEG_COUNTS*2)/3);
    turn(-TURNING_SPEED, getHeadingCounts(W));
    
    // back up to allow arm to come down
    moveForwardEncoder(-DEFAULT_SPEED, getCounts(1.0));
    moveArm(highPos, appleLowPos, appleSpeed);
    int toBucketCounts = getXYCountsRCS(13.75, 0);
    moveForwardEncoder(DEFAULT_SPEED, toBucketCounts);
    moveArm(appleLowPos, appleHighPos, appleSpeed);
    moveForwardEncoder(-DEFAULT_SPEED, toBucketCounts);
    turn(TURNING_SPEED, NINETYDEG_COUNTS/3);
    moveForwardEncoder(-DEFAULT_SPEED, getXYCountsRCS(28, 0));
    turn(TURNING_SPEED, getHeadingCounts(N));
    headingCorrection(N);
    // go up ramp
    moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(0, 43.75));
    turn(-TURNING_SPEED, NINETYDEG_COUNTS);
    moveForwardEncoder(DEFAULT_SPEED, getCounts(1.0));
    headingCorrection(W);
    moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(24.5, 0));
    // turn towards lower basket
    turn(TURNING_SPEED, NINETYDEG_COUNTS);
    headingCorrection(N);
    moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(0, 61));
    moveArm(appleHighPos, 160, 2);
    moveForwardEncoder(-DEFAULT_SPEED, getCounts(0.75));
    turn(TURNING_SPEED, 1);
    moveArm(160, 150, 2);
    moveForwardEncoder(-(DEFAULT_SPEED+10), getCounts(2.0));
    moveArm(150, appleHighPos, 5);
    // go to levers, any lever counts for primary points
    turn(-TURNING_SPEED, NINETYDEG_COUNTS);
    headingCorrection(W);
    turn(TURNING_SPEED, NINETYDEG_COUNTS/8 + 1);
    moveForwardEncoder(DEFAULT_SPEED, getCounts(4));
    // flick lever down
    moveArm(appleHighPos, lowerPos, armSpeedDiv);
    moveForwardEncoder(-DEFAULT_SPEED, getCounts(2.0));
    moveForwardEncoder(DEFAULT_SPEED, getCounts(2.0));
    // flick lever up
    moveArm(lowerPos, appleHighPos, armSpeedDiv);
    moveForwardEncoder(-DEFAULT_SPEED, getCounts(1.5));
    // go to open window with arm
    turn(-TURNING_SPEED, NINETYDEG_COUNTS);
    headingCorrection(S);
    moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(0, 50));
    moveArm(appleHighPos, appleLowPos, armSpeedDiv);
    turn(TURNING_SPEED + 10, NINETYDEG_COUNTS/2 + 10);
    // back up to humidifier interface
    moveForwardEncoder(-DEFAULT_SPEED, getXYCountsRCS(0, 45));
    // turn to light and move to it
    headingCorrection(N);
    moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(14.3, 0));
    // detect the light color and move turn in the appropriate direction
    int directionMult = -1; 
    if(displayLightColor(3.0)){
        directionMult = 1;
    }
    moveForwardEncoder(-DEFAULT_SPEED, getCounts(2.0));
    turn(DEFAULT_SPEED * directionMult, NINETYDEG_COUNTS/9);
    // move arm down to press button
    moveArm(80, 170, 3);
    moveForwardEncoder(DEFAULT_SPEED, getCounts(2.5));
    moveForwardEncoder(-DEFAULT_SPEED, getCounts(2.5));
    moveArm(170, 80, 3);
    // turn back in the same direction
    turn(-DEFAULT_SPEED * directionMult, NINETYDEG_COUNTS/9);
    // back up to the ramp
    moveForwardEncoder(-DEFAULT_SPEED, 28.2 - 14.34);
    turn(-DEFAULT_SPEED, NINETYDEG_COUNTS);
    // go down ramp
    moveForwardEncoder(DEFAULT_SPEED, getCounts(25.0));
    LCD.Clear();
    LCD.WriteLine(RCS.RequestsRemaining());



    






    
    // MILESTONE 5  -  26/20
    // detectStart();
    // // turn(-DEFAULT_SPEED, NINETYDEG_COUNTS/2);
    // // // turn a little more to face the bin
    // // turn(-DEFAULT_SPEED, NINETYDEG_COUNTS/8);
    // //turn(-DEFAULT_SPEED, getHeadingCounts(40.0));
    // turn(-DEFAULT_SPEED, NINETYDEG_COUNTS/2);
    // turn(-DEFAULT_SPEED, NINETYDEG_COUNTS/5);
    // // put arm in down position
    // int lowerPos = 180, highPos = 105;
    // // move to bin
    // //moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(21.7, 4.0));
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(5.3));
    // //moveForwardEncoder(DEFAULT_SPEED, getCounts(7.0));
    // // turn clockwise, starting from the high pos
    // int armSpeedDiv = 8;
    // float moveDist = 1.5;
    // // make these into function calls to prevent copied code
    // moveArm(highPos, lowerPos, armSpeedDiv);
    // moveForwardEncoder(-DEFAULT_SPEED, getCounts(moveDist));
    // moveArm(lowerPos, highPos, armSpeedDiv);
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(moveDist));
    // moveArm(highPos, lowerPos, armSpeedDiv);
    // moveForwardEncoder(-DEFAULT_SPEED, getCounts(moveDist));
    // moveArm(lowerPos, highPos, armSpeedDiv);
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(moveDist));
    // moveArm(highPos, lowerPos, armSpeedDiv);
    // moveForwardEncoder(-DEFAULT_SPEED, getCounts(moveDist));
    // moveArm(lowerPos, highPos, armSpeedDiv);
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(moveDist));
    // moveArm(highPos, lowerPos, armSpeedDiv);
    // //do this all in reverse to turn counterclockwise
    // moveArm(lowerPos, highPos, armSpeedDiv);
    // moveForwardEncoder(-DEFAULT_SPEED, getCounts(moveDist));
    // moveArm(highPos, lowerPos, armSpeedDiv);
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(moveDist));
    // moveArm(lowerPos, highPos, armSpeedDiv);
    // moveForwardEncoder(-DEFAULT_SPEED, getCounts(moveDist));
    // moveArm(highPos, lowerPos, armSpeedDiv - 1);
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(moveDist));
    // moveArm(lowerPos, highPos, armSpeedDiv);
    // moveForwardEncoder(-DEFAULT_SPEED, getCounts(moveDist));
    // moveArm(highPos, lowerPos, armSpeedDiv - 1);
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(moveDist));
    // moveArm(lowerPos, highPos, armSpeedDiv);
    // moveForwardEncoder(-DEFAULT_SPEED, getCounts(moveDist));
    // moveArm(highPos, lowerPos, armSpeedDiv - 1);
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(moveDist));
    // moveArm(lowerPos, highPos, armSpeedDiv);
    // // turn to rotate bin
    // // turn(DEFAULT_SPEED, getHeadingCounts(18));
    // // lower arm to finish rotating
    // // moveArm(highPos, lowerPos);
    // // turn(-DEFAULT_SPEED, getHeadingCounts(22.0));
    // // // do previous in reverse
    // // turn(DEFAULT_SPEED, getHeadingCounts(18));
    // // moveArm(lowerPos, highPos);
    // // turn(-DEFAULT_SPEED, getHeadingCounts(22.0));
    // // moveArm(highPos, lowerPos);
    // // go back to start
    // moveForwardEncoder(-DEFAULT_SPEED, getCounts(5.3));
    // //turn(DEFAULT_SPEED, getHeadingCounts(300));
    // turn(DEFAULT_SPEED, NINETYDEG_COUNTS/2);
    // // turn(DEFAULT_SPEED, NINETYDEG_COUNTS/8);
    // moveForwardEncoder(-DEFAULT_SPEED, getCounts(2.2));

    // // // Milestone 4 - 4/20, picked
    // // we need arm movement code, more precise movement code (RCS) to account for slipping, and 
    //detectStart();  
    //reset arm
    //Put a while here or a switchcase here to make the detect start a major requirement
    // moveArm(82, 80);
    // //turn(DEFAULT_SPEED, NINETYDEG_COUNTS/2);
    // // move to y coord for apple lineup
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(17.0));
    // check_y(22.0, FORWARDS);
    // //turn to face apple
    // turn(-DEFAULT_SPEED, NINETYDEG_COUNTS/2);
    // //move to apple
    // moveArm(80, 165);
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(2.0));
    // //check_x(10.0, FORWARDS);
    // // Arm movement code
    // // FIND ARM MIN AND MAX and the range of the the arm mechanism
    // moveArm(165, 135);
    // moveForwardEncoder(-DEFAULT_SPEED, getCounts(2.0));
    // //turn slightly away from ramp
    // turn(DEFAULT_SPEED, 25);
    // // back up
    // moveForwardEncoder(-DEFAULT_SPEED, getCounts(3.0));
    // // turn back to align
    // turn(-DEFAULT_SPEED, 25);
    // // back up to ramp
    // moveForwardEncoder(-DEFAULT_SPEED, getCounts(10.0));
    // check_x(30.5, BACKWARDS); // make sure this x aligns well with the 
    // // turn to ramp
    // turn(DEFAULT_SPEED, NINETYDEG_COUNTS + 20);
    // // go up ramp 
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(15.0));
    // // get to line
    // check_y(48.0, FORWARDS);
    // turn(-DEFAULT_SPEED, NINETYDEG_COUNTS/2);
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(3.0));
    // turn(DEFAULT_SPEED, NINETYDEG_COUNTS/2);
    // check_heading(N);
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(5.0));
    // moveArm(140, 170);
    // moveForwardEncoder(-DEFAULT_SPEED, getCounts(5.0));
    // turn(-DEFAULT_SPEED, NINETYDEG_COUNTS/2);
    // moveArm(170, 100);
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(5.0));
    // moveArm(100, 165);

    // 3/23/26 movement test - GOOD
    // moveForwardsNoEncoder(speed, 3.0);
    // turnRightNoEncoder(25, 5.0);
    // moveForwardsNoEncoder(-1 * speed, 3.0);

    // 3/24/26 encoder movement test - Alright, Encoders slightly inconsistent
    // writeLight();
    // moveForwardEncoder(speed, getCounts(3.0));
    // turn(25, 36);
    // turn(-25, 36);
    // moveForwardEncoder(-1 * speed, getCounts(3.0));
    // writeLight();

    // 3/25/26 Milestone 3 test - it worked, needed to get rid of hot glue on treads to prevent slipping
    // detectStart();
    // turn(speed, NINETYDEG_COUNTS/2);
    // moveForwardEncoder(speed, getCounts(12));
    // Sleep(0.2);
    // // RCS to the good position
    // moveForwardEncoder(speed, getCounts(19));
    // Sleep(0.2);
    // turn(-speed, NINETYDEG_COUNTS + 1);
    // moveForwardEncoder(35, getCounts(17));
    // moveForwardEncoder(-35, getCounts(12));
    //moveForwardEncoder(-speed, getCounts(13));
    // while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
    // while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed
    // moveForwardEncoder(-speed, getCounts(10));
    // while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
    // while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed
    // writeLight();


    // 3/24/26 CdS cell testing
    //writeLight();

    // ***** *ENCODER MOVEMENT TEST* ******//
    // // //Test move forwards 5 inches
    // turn(speed, 36);
    // turn(-1 * speed, 18);
    // moveForwardEncoder(speed, getCounts(5.0));

    // ********* Encoder output test *********//
    // float inches = 3;
    // encoderExploration(inches);
    // while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
    // while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed
    // turn(speed, 72);
    // Sleep(0.1);
    // turn(-speed, 36);
    // Sleep(0.1);

    // 3/24/26  FIND THE THRESHOLD FOR LIGHT VALUES USING FILTERS - YOU FOUND IT
    // writeLight();
    // *** 0.44 for on, 1.9 for off ***
    
    // MILESTONE 2
    //15 inches from light to ramp
    // 12 ramp is 12 inches long and 3 inches high, 12.36 inches hypotenuse
    // 11.5 inches from top of ramp to middle of line
    // 8? inches up to the light,
    // 8.5 ish inches to the lights, make sure the robot actually pushes it
    // MILESTONE 2 CODE
    // (cds cell detected light)
    // moveForwardsNoEncoder(-speed, 1.0);
    // Sleep(0.2);
    // moveForwardsNoEncoder(speed, 1.0);
    // Sleep(0.2);
    // //turn(speed - 5, 27);
    // turnRightNoEncoder(speed, 2.0);
    // Sleep(0.2);
    // moveForwardsNoEncoder(speed, 20);
    // Sleep(0.2);
    // // turn(-1 * (speed - 5), 27);
    // Sleep(0.2);
    // moveForwardEncoder(speed, getCounts(12));
    // Sleep(0.2);
    // displayLightColor(5);
    // moveForwardsEncoder(-1  * speed, getCounts(1.5));
    // moveForwardsEncoder(speed, getCounts(1.5));
    // turn(speed, 18); // 45 deg turn right
    // to ramp
    // moveForwardsEncoder(speed, getCounts(12));
    // sleep(0.2);
    // up ramp
    // moveForwardsEncoder(speed, getCounts(12.36));
    // moveForwardsEncoder(speed, getCounts(8));
    // turn(-1 * speed, 38);
    // move to light, probably write new code for moving until light is sensed?
    // moveForwardsEncoder(speed, getCounts(8));
    // (cds cell detected light), (getCorrectLight from cdscell)
    // turn approc 12.5 degrees either direction
    // (based on light from cds cell), turn(speed, 5)

    // //TODO REDO THIS
    // // Milestone 2 REDO
    // detectStart();
    // turn(DEFAULT_SPEED, getHeadingCounts(N));
    // check_heading(N);
    // moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(0, 47));
    // // make sure it understeers so that we turn the correct direction
    // // maybe change how getHeadingCounts works so that we get the direction back as well********
    // // maybe change getheading counts to instead take a coordinate instead of an angle 
    // // OVERLOAD GETHEADINGCOUNTS FOR COORDINATES
    // turn(-DEFAULT_SPEED, getHeadingCounts(W));
    // check_heading(W);
    // moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(14.3,0));
    // int directionMult = -1; 
    // if(displayLightColor(3.0)){
    //     directionMult = 1;
    // }
    // moveForwardEncoder(-DEFAULT_SPEED, getCounts(2.0));
    // turn(DEFAULT_SPEED * directionMult, NINETYDEG_COUNTS/9);
    // moveArm(80, 170, 3);
    // moveForwardEncoder(35, getCounts(2.5));
    // moveForwardEncoder(-35, getCounts(2.5));
    // moveArm(170, 80, 3);
    // turn(DEFAULT_SPEED, getHeadingCounts(E));
    // check_heading(E);
    // moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(28.2, 0));
    // turn(DEFAULT_SPEED, getHeadingCounts(S));
    // turn(DEFAULT_SPEED, getHeadingCounts(S));
    // turn(DEFAULT_SPEED, getHeadingCounts(S));
    // moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(30, 3));

    // // MILESTONE 2 NO-RCS BACKUP
    // // make sure to disable RCS initialization for this one
    // detectStart();
    // turn(DEFAULT_SPEED, NINETYDEG_COUNTS/2);
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(25.0));
    // turn(-DEFAULT_SPEED, NINETYDEG_COUNTS);
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(28.2 - 14.34));
    // int directionMult = -1; 
    // if(displayLightColor(3.0)){
    //     directionMult = 1;
    // }
    // moveForwardEncoder(-DEFAULT_SPEED, getCounts(2.0));
    // turn(DEFAULT_SPEED * directionMult, NINETYDEG_COUNTS/9);
    // moveArm(80, 170, 3);
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(2.5));
    // moveForwardEncoder(-DEFAULT_SPEED, getCounts(2.5));
    // moveArm(170, 80, 3);
    // turn(-DEFAULT_SPEED * directionMult, NINETYDEG_COUNTS/9);
    // moveForwardEncoder(-DEFAULT_SPEED, 28.2 - 14.34);
    // turn(-DEFAULT_SPEED, NINETYDEG_COUNTS);
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(25.0));





    // moveForwardsNoEncoder(speed, 7.0);
    // Sleep(0.5);
    // moveForwardsNoEncoder(-1 * speed, 7.0);
    // Sleep(0.5);
    // moveForwardsNoEncoder(speed, 1);
    // Sleep(0.5);
    // turnRightNoEncoder(speed, 3.0);
    // Sleep(0.5);
    // moveForwardsNoEncoder(speed, 20.0);

    //moveForwardsNoEncoder(speed, 7.4);
    //Sleep(0.5);
    //moveForwardsNoEncoder(-1 * speed, 7.4);

    
    //while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
    //while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed

    //moveForwardsNoEncoder(speed, 8.0);
    //Sleep(0.5);
    //moveForwardsNoEncoder(-1 * speed, 8.0);

    //encoder movement
    // moveForwards(speed, 472);
    // Sleep(1.0);
    // turnLeft(speed, 239);
    // Sleep(1.0);
    // moveForwards(speed, 337);
    // Sleep(1.0);
    // turnRight(speed, 239);
    // Sleep(1.0);
    // moveForwards(speed, 135);

    
        // moveArm(down to pick up basket)
        // moveForwards(speed, countToApples);
        // moveArm(up to pick up basket);
        // move


        
    
    
    
    //472.37 for 14 inches
    //337 for 10 inches
    //135 for 4 inches
}

