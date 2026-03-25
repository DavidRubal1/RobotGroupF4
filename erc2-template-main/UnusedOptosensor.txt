// Declarations for analog optosensors
AnalogInputPin right_opto(FEHIO::Pin0);
AnalogInputPin middle_opto(FEHIO::Pin1);
AnalogInputPin left_opto(FEHIO::Pin2);

void optosensorTest(){
    int x, y; //for touch screen

    //Initialize the screen
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    LCD.WriteLine("Analog Optosensor Testing");
    LCD.WriteLine("Touch the screen");
    while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
    while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed

    // Record values for optosensors on and off of the straight line
    // Left Optosensor on straight line
    LCD.Clear(BLACK);
    LCD.WriteLine("Place left optosensor on straight line");
    Sleep(0.25); // Wait to avoid double input
    LCD.WriteLine("Touch screen to record value (1/12)");
    while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
    while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed
    // Write the value returned by the optosensor to the screen
    float leftOptosensorValue = left_opto.Value();
    LCD.Write("Left Optosensor Value:");
    LCD.WriteLine(leftOptosensorValue);

    // Write the value returned by the optosensor to the screen
    float midOptosensorValue = middle_opto.Value();
    LCD.Write("Mid Optosensor Value:");
    LCD.WriteLine(midOptosensorValue);


    // Write the value returned by the optosensor to the screen
    float rightOptosensorValue = right_opto.Value();
    LCD.Write("Right Optosensor Value:");
    LCD.WriteLine(rightOptosensorValue);

    while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
    while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed


    // Left Optosensor off straight line
    LCD.Clear(BLACK);
    LCD.WriteLine("Place left optosensor off straight line");
    Sleep(0.25); // Wait to avoid double input
    LCD.WriteLine("Touch screen to record value (2/12)");
    while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
    while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed
    // Write the value returned by the optosensor to the screen
    leftOptosensorValue = left_opto.Value();
    LCD.Write("Left Optosenor Value: ");
    LCD.WriteLine(leftOptosensorValue);

    midOptosensorValue = middle_opto.Value();
    LCD.Write("Mid Optosensor Value:");
    LCD.WriteLine(midOptosensorValue);

    rightOptosensorValue = right_opto.Value();
    LCD.Write("Right Optosensor Value:");
    LCD.WriteLine(rightOptosensorValue);

    while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
    while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed

    // Print end message to screen
    LCD.Clear(BLACK);
    LCD.WriteLine("Test Finished");


}

enum LineStates {
    MIDDLE,
    RIGHT,
    LEFT
};

void lineFollow(){
    int x, y; //for touch screen

    //Initialize the screen
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    LCD.WriteLine("Analog Optosensor Testing");
    LCD.WriteLine("Touch the screen");
    while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
    while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed
    

    int speed = 25;
    int state = MIDDLE; // Set the initial state
    while (true) { // I will follow this line forever!
        switch(state) {
            
            case MIDDLE:
                /* Drive straight*/
                right_motor.SetPercent(speed);
                left_motor.SetPercent(-1 * speed);

                if (right_opto.Value() > 1.5 && middle_opto.Value() < 3.0) {
                state = RIGHT; 
                }
                /* Code for if left sensor is on the line */
                if (left_opto.Value() > 1.5 && middle_opto.Value() < 3.0) {
                state = LEFT; 
                }
                break;
            // If the right sensor is on the line...
            case RIGHT:
                // Set motor powers for right turn
                /* Drive */
                right_motor.SetPercent(0);
                left_motor.SetPercent(-1 * speed);
                if( middle_opto.Value() > 3.0 ) {
                state = MIDDLE;
                }
                break;
            // If the left sensor is on the line...
            case LEFT:
                /* Mirror operation of RIGHT state */
                right_motor.SetPercent(speed);
                left_motor.SetPercent(0);
                if( middle_opto.Value() > 3.0 ) {
                    state = MIDDLE;
                }
                break;
            default: // Error. Something is very wrong.
                break;
        }

        
        // // Write the value returned by the optosensor to the screen
        // float leftOptosensorValue = left_opto.Value();
        // LCD.Write("Left Optosensor Value:");
        // LCD.WriteLine(leftOptosensorValue);

        // // Write the value returned by the optosensor to the screen
        // float midOptosensorValue = middle_opto.Value();
        // LCD.Write("Mid Optosensor Value:");
        // LCD.WriteLine(midOptosensorValue);


        // // Write the value returned by the optosensor to the screen
        // float rightOptosensorValue = right_opto.Value();
        // LCD.Write("Right Optosensor Value:");
        // LCD.WriteLine(rightOptosensorValue);
        // Sleep(0.25);
        // LCD.Clear();
    }
}