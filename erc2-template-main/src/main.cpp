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
#define DEFAULT_SPEED 37
#define FAST_SPEED 45
#define TURNING_SPEED 25

// heading keywords
#define N 90
#define E 0
#define S 270
#define W 180

DigitalEncoder right_encoder(FEHIO::Pin8);
DigitalEncoder left_encoder(FEHIO::Pin14);
FEHMotor right_motor(FEHMotor::Motor1,9.0);
FEHMotor left_motor(FEHMotor::Motor2,9.0);
FEHServo armServo(FEHServo::Servo0);
AnalogInputPin lightSensor(FEHIO::Pin1);

// converts inches input into counts for shaft encoders
int getCounts(float dist){
    return (ENCODER_MAX * dist) / (2 * PI * WHEEL_RADIUS);
}
// smoothly move arm
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

void moveForwardEncoder(int percent, int counts) {
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    right_motor.SetPercent(percent);
    left_motor.SetPercent(-1 * percent);
    while(left_encoder.Counts() <= counts || right_encoder.Counts() <= counts);
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

// pauses until the start light haas been detected by the CdS Cell
void detectStart(){
    Sleep(0.1);
        while((lightSensor.Value() > ANY_LIGHT_VALUE));
        LCD.Clear();
        moveForwardsNoEncoder(-35, 0.75);
        moveForwardEncoder(35, getCounts(2.25));
}

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
// gets the number of counts for the robot to move to the specified robots
// if x or y are sent in as 0, set them to the rcs pos
void RCSRecover(RCSPose* pos){
    int count = 0;
    while(pos->x <= 0 || pos->x >100){
        Sleep(0.30);
        pos = RCS.RequestPosition();
        LCD.SetFontColor(RED);
        LCD.SetFontSize(5);
        LCD.WriteRC("NEGATIVE RCS VALUES", 5, 5);
        LCD.WriteRC(pos->x, 6, 5);
        LCD.SetFontColor(WHITE);
        if(count >= 4){
            Sleep(0.7);
        } 
        //panic move
        if(count == 6){
            moveForwardEncoder(DEFAULT_SPEED, getCounts(1));
        }
        count++;
        if(count > 15){
            LCD.SetFontColor(RED);
            LCD.WriteRC("TOO MANY INVALID RCS VALUES, RETURN 0", 5, 5);
            LCD.SetFontColor(WHITE);
            return;
        }
    }

}
int getXYCountsRCS(float x, float y){
    float hyp = 100;
    while(hyp > 50){
        Sleep(0.30);
        RCSPose* pos = RCS.RequestPosition();
        RCSRecover(pos);
        if(x == 0){
            x = pos->x;
        }
        if(y == 0){
            y = pos->y;
        }
        hyp = sqrt(pow((x - pos->x), 2) + pow((y - pos->y), 2.0));
    }


    return getCounts(hyp);
}

// gives the rotation counts for the optimal rotation direction to the given heading from the current heading
int getHeadingCounts(float heading){
    Sleep(0.30);
    RCSPose* pos = RCS.RequestPosition();
    RCSRecover(pos);
    float degrees = 0;
    // 300 -> 10
    if(abs(pos->heading - heading) > 180){  // passing 0
        if(pos->heading > heading){// turn counter-clockwise
            degrees = (360 - pos->heading) + heading;
        }else{ 
            degrees = (360 - heading) + pos->heading; 
        }
    } else{ // turn clockwise
        degrees = abs(pos->heading - heading); 
    }


    int counts = (degrees/90.0) * NINETYDEG_COUNTS;
    return counts;
}

// Only call with headings > 10 or == 0 to account for RCS inconsistency
void headingCorrection(float heading){
    RCSPose* pos = RCS.RequestPosition();
    RCSRecover(pos);
    // account for exactly 0
    if(heading == 0){
        if(180 < pos->heading && pos->heading < 357){
            turn(-TURNING_SPEED, getHeadingCounts(heading));
        } else if(180 > pos->heading && pos->heading > 3){
            turn(TURNING_SPEED, getHeadingCounts(heading));
        }
    } else{
        if(pos->heading < heading - 4){
            turn(-TURNING_SPEED, getHeadingCounts(heading));
        } else if(pos->heading > heading + 4){
            turn(TURNING_SPEED, getHeadingCounts(heading));
        }
    }
}
void xCorrection(int x){
    RCSPose* pos = RCS.RequestPosition();
    RCSRecover(pos);
    // account for exactly 0
    if(x != pos->x){
        if(pos->heading > 90 && pos->heading < 270){
            if(x > pos->x){
                moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(x, 0));
            } else{
                moveForwardEncoder(-DEFAULT_SPEED, getXYCountsRCS(x, 0));
            }
        } else{
            if(x > pos->x){
                moveForwardEncoder(-DEFAULT_SPEED, getXYCountsRCS(x, 0));
            } else{
                moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(x, 0));
            }
        }
    }   
}

void yCorrection(int y){
    RCSPose* pos = RCS.RequestPosition();
    RCSRecover(pos);
    // account for exactly 0
    if(y != pos->y){
        if(pos->heading > 0 && pos->heading < 180){
            if(y > pos->y){
                moveForwardEncoder(-DEFAULT_SPEED, getXYCountsRCS(0, y));
            } else{
                moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(0, y));
            }
        } else{
            if(y > pos->y){
                moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(0, y));
            } else{
                moveForwardEncoder(-DEFAULT_SPEED, getXYCountsRCS(0, y));
            }
        }
    }   
}


// method for hammering the arm on the compost bin mechanism to perform one 90 degree rotation.     
void compostBinTurn(int high, int low, int speed, float moveDist){
    moveArm(high, low, speed);
    moveForwardEncoder(-DEFAULT_SPEED, getCounts(moveDist));
    moveArm(low, high, speed);
    moveForwardEncoder(DEFAULT_SPEED, getCounts(moveDist));
}

void reverseCompostBin(int high, int low, int speed, float moveDist){
    moveArm(low, high, speed);
    moveForwardEncoder(-DEFAULT_SPEED, getCounts(moveDist));
    moveArm(high, low, speed);
    moveForwardEncoder(DEFAULT_SPEED, getCounts(moveDist));
}

void writeLight(){
        LCD.SetFontSize(5);
        int x, y;
        while(!LCD.Touch(&x,&y)){ //Wait for screen to be pressed
            
            LCD.Write(lightSensor.Value());
            Sleep(0.5);
            LCD.Clear();
        
        }
}

void ERCMain(){

    // armServo.SetMin(500);
    // armServo.SetMax(2500);
    // //TESTING REMOVE THIS BEFORE FINAL UPLOAD
    // writeLight();
    // moveForwardEncoder(DEFAULT_SPEED, getCounts(7.5));
    // moveForwardEncoder(-DEFAULT_SPEED, getCounts(7.5));
    // turn(TURNING_SPEED, NINETYDEG_COUNTS);
    // turn(-TURNING_SPEED, NINETYDEG_COUNTS);
    // moveArm(90, 180, 2);
    // moveArm(180, 90, 2);


    RCS.InitializeTouchMenu("0150F4CPJ");
    WaitForFinalAction();

    armServo.SetMin(500);
    armServo.SetMax(2500);

    detectStart();
    turn(-TURNING_SPEED, getHeadingCounts(180));
    turn(-TURNING_SPEED, getHeadingCounts(194));
    int lowerPos = 185, highPos = 105;
    // move to bin
    moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(23.9, 0));
    // turn clockwise, starting from the high pos
    int armSpeedDiv = 30;
    float moveDist = 1.5;
    for(int i = 0; i < 2; i++){
        compostBinTurn(highPos, lowerPos, armSpeedDiv, moveDist);
    }
    turn(-TURNING_SPEED, 1);
    compostBinTurn(highPos, lowerPos, armSpeedDiv, moveDist);
    turn(TURNING_SPEED, 1);
    // correcting turn to account for initial oversteer
    compostBinTurn(highPos, lowerPos, armSpeedDiv, moveDist);
    moveArm(highPos, lowerPos, armSpeedDiv);
    moveArm(lowerPos, highPos+25, armSpeedDiv);
    // turn to apple bucket
    int appleLowPos = 155, appleHighPos = 100, appleSpeed = 2;
    turn(TURNING_SPEED, NINETYDEG_COUNTS/2);
    moveForwardEncoder(DEFAULT_SPEED, getCounts(0.5));
    // *CHANGE go to lower window first, then apple basket
    // offset turning slightly to not go on the ramp
    turn(TURNING_SPEED, getHeadingCounts(113));
    moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(0, 20.75));
    turn(TURNING_SPEED + 22, NINETYDEG_COUNTS/4 + 5);
    moveForwardEncoder(-DEFAULT_SPEED, getCounts(0.5));
    turn(TURNING_SPEED + 22, NINETYDEG_COUNTS/4);
    moveArm(highPos+33, highPos, armSpeedDiv);
    turn(-(TURNING_SPEED + 18), NINETYDEG_COUNTS/2 + 10);
    turn(TURNING_SPEED + 18, 1);
    moveArm(highPos, highPos +33, armSpeedDiv);
    moveForwardEncoder(-DEFAULT_SPEED, getXYCountsRCS(0, 18));
    // turn(TURNING_SPEED, getHeadingCounts(100));
    // moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(0, 18.95));
    // turn to bucket
    turn(-TURNING_SPEED, (NINETYDEG_COUNTS)/3);
    turn(-TURNING_SPEED, getHeadingCounts(W-2));
    //headingCorrection(W-5);
    // back up to allow arm to come down to pick up basket
    moveForwardEncoder(-DEFAULT_SPEED, getCounts(2.0));
    moveArm(highPos, appleLowPos, appleSpeed);
    int toBucketCounts = getXYCountsRCS(13.2, 0);
    moveForwardEncoder(DEFAULT_SPEED, toBucketCounts);
    moveArm(appleLowPos, appleHighPos, appleSpeed);
    moveArm(appleHighPos, appleHighPos + 2, appleSpeed);
    moveForwardEncoder(-DEFAULT_SPEED, toBucketCounts);
    turn(TURNING_SPEED, NINETYDEG_COUNTS/3 - 3);
    moveForwardEncoder(-DEFAULT_SPEED, getXYCountsRCS(28.0, 0));
    turn(TURNING_SPEED, getHeadingCounts(N-5));
    headingCorrection(N-2);
    // go up ramp
    moveForwardEncoder(FAST_SPEED, getXYCountsRCS(0, 43.75));
    // manual turn and movement to get out of deadzone
    turn(-TURNING_SPEED, NINETYDEG_COUNTS);
    moveForwardEncoder(DEFAULT_SPEED, getCounts(2.0));
    headingCorrection(W);
    moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(24.5, 0));
    // turn towards lower basket
    turn(TURNING_SPEED, NINETYDEG_COUNTS);
    headingCorrection(N);
    moveForwardEncoder(FAST_SPEED, getXYCountsRCS(0, 61));
    moveArm(appleHighPos, 160, 2);
    moveForwardEncoder(-DEFAULT_SPEED, getCounts(0.75));
    turn(TURNING_SPEED, 1);
    moveArm(160, 150, 2);
    moveForwardEncoder(-FAST_SPEED, getCounts(0.75));
    moveArm(150, appleHighPos, 5);
    moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(0, 61.3));
    // go to lever
    turn(-TURNING_SPEED, NINETYDEG_COUNTS);
    headingCorrection(174);
    //turn slightly to the right if off still

    moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(16.75, 0));
    // slight correction
    //turn(TURNING_SPEED, 6);
    // flick lever down
    moveArm(appleHighPos, lowerPos + 30, armSpeedDiv);
    moveForwardEncoder(-DEFAULT_SPEED, getCounts(2.5));
    Sleep(4.4);
    turn(-TURNING_SPEED, 1);
    moveForwardEncoder(DEFAULT_SPEED, getCounts(3.5));
    turn(TURNING_SPEED, 4);
    moveForwardEncoder(-DEFAULT_SPEED, getCounts(1));
    // flick lever up
    moveArm(lowerPos + 30, appleHighPos, armSpeedDiv);
    moveForwardEncoder(-DEFAULT_SPEED, getCounts(1));
    // go to open window with arm
    turn(-TURNING_SPEED, NINETYDEG_COUNTS);
    headingCorrection(S);
    moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(0, 50));
    moveArm(appleHighPos, appleLowPos, armSpeedDiv);
    turn(TURNING_SPEED + 20, NINETYDEG_COUNTS/2 + 10);
    turn(-TURNING_SPEED, 1);
    // back up to humidifier interface
    turn(-TURNING_SPEED, 2);
    moveForwardEncoder(-DEFAULT_SPEED, getXYCountsRCS(0, 45.8));
    moveArm(appleLowPos, highPos, 8);
    // turn to light and move to it
    headingCorrection(W);
    moveForwardEncoder(DEFAULT_SPEED, getXYCountsRCS(6.4, 0));   
    // back to the ramp
    moveForwardEncoder(-FAST_SPEED, getXYCountsRCS(28.8, 0));
    turn(-DEFAULT_SPEED, NINETYDEG_COUNTS);
    // go down ramp
    moveForwardEncoder(FAST_SPEED, getXYCountsRCS(0, 8.0)/2);
    headingCorrection(S);
    moveForwardEncoder(FAST_SPEED, getXYCountsRCS(0, 9.0));
    turn(-DEFAULT_SPEED, NINETYDEG_COUNTS/2);
    moveForwardEncoder(FAST_SPEED, getCounts(9.0));



    
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
// gets the number of counts for the given distance in inches
// waits for the given number of seconds
boolean displayLightColor(float seconds){
    boolean red = false;
    int starttime = time(NULL);
    while(time(NULL) - starttime <= seconds){
        if(lightSensor.Value() > RED_LIGHT_THRESHOLD){
            LCD.SetFontColor(RED);
            // LCD.FillRectangle(0, 0, 319, 239);
            // Sleep(1.5);
            red = true;
        } else{
            LCD.SetFontColor(BLUE);
            // LCD.FillRectangle(0, 0, 319, 239);
            // Sleep(1.5);
        }
    }
    return red;
}
