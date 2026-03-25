#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHRCS.h>
#include <FEHSD.h>

void ERCMain()
{
    // Declare variables.
    // For touchscreen functionality.
    int touch_x, touch_y;
    // Loop n through points to record necessary positions.
    int n;
    char points[4] = {'A', 'B', 'C', 'D'};

    // Disable RCS position rate limiting to allow for more
    // frequent position updates (testing only, won't work during competition)
    RCS.DisableRateLimit();

    // Call this function to initialize the RCS to a course.
    // For Exploration 3, please use the key provided below.
    //     This matches the AruCo code that is provided for this exploration.
    // If your team wishes to use RCS's positioning system going forward,
    //     please use the 8 digit code on your team's page on the store website.
    RCS.InitializeTouchMenu("Z1TESTING");

    // Open SD file for writing
    FEHFile *fptr = SD.FOpen("test.txt", "w");

    // Wait for touchscreen to be pressed and released
    LCD.WriteLine("Press Screen to Start");
    while (!LCD.Touch(&touch_x, &touch_y));
    while (LCD.Touch(&touch_x, &touch_y));

    // Clear screen
    LCD.Clear();

    // Step through each path point (A,B,C,D) to record position and heading
    for (n = 0; n <= 3; n++)
    {
        LCD.Clear();
        LCD.WriteRC("Touch to set point ", 0, 0);
        LCD.WriteRC(points[n], 0, 20);

        RCSPose *pose;

        // Loop until touchscreen is pressed, continuously requesting 
        // RCS position data and writing it to the LCD screen
        while (!LCD.Touch(&touch_x, &touch_y))
        {
            // Request position data from RCS
            pose = RCS.RequestPosition();

            // Clear previous position data from LCD
            LCD.SetFontColor(BLACK);
            LCD.FillRectangle(130, 20, 100, 60);

            LCD.SetFontColor(WHITE);
            LCD.WriteRC("X Position:", 2, 0);
            LCD.WriteRC("Y Position:", 3, 0);
            LCD.WriteRC("Heading:", 4, 0);

            // Write the RCS data to the LCD screen
            LCD.WriteRC(pose->x, 2, 12);
            LCD.WriteRC(pose->y, 3, 12);
            LCD.WriteRC(pose->heading, 4, 12);
        }

        // Print RCS data for this path point to file
        SD.FPrintf(fptr, "%f %f\n", pose->x, pose->y);

        // Wait for touchscreen to be released
        while (!LCD.Touch(&touch_x, &touch_y));
    }

    // Close SD file
    SD.FClose(fptr);
}