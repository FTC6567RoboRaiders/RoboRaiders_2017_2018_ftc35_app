package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboraiders.Robot.Robot;

/**
 * Created by Jason Sember on 9/23/2017.
 */

@TeleOp

public class TeleOpNewMecanum extends OpMode {

    public Robot robot = new Robot();

    /* Define variables */
    float LeftBack;   // Power for left back motor
    float RightBack;  // Power for right back motor
    float LeftFront;  // Power for left front motor
    float RightFront; // Power for right front motor
    boolean nudging = false;
    int nudgeCount = 0;

    float maxpwr;     // Maximum power if the four motors

    // The following variables are used to control how often telemetry data is written to the log
    //
    //  - currentTimeStamp - is the current time stamp, this is updated every time the loop() method is
    //                     called
    //
    //  - pastTimeStamp    - is the time stamp that the log was last updated, initially it is set to 0,
    //                     and is only updated when the log is updated
    //
    //  - LOG_INTERVAL     - the amount of time per each log updated, initially set to 1/4 of a
    //                     second, this value is in milliseconds (1/4 of sec = 250 milliseconds)

    private long currentTimeStamp;
    private long pastTimeStamp;
    private static final long LOG_INTERVAL = 250;

    @Override
    public void init() {

        robot.initialize(hardwareMap);
        pastTimeStamp = 0;

        // Write message to log indicating that teleop program is initialized
        Log.d("Initialized","Teleop Initialization Complete");

        telemetry.addData("Initialized", true);
        telemetry.update();
    }

    @Override
    public void loop() {

        boolean logIt;

        currentTimeStamp = System.currentTimeMillis();   //* get the current time stamp

        /*
        are the right_stick powers right?
         */
        LeftBack = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
        RightBack = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        LeftFront = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        RightFront = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;

        maxpwr = findMaxPower(LeftBack, LeftFront, RightBack, RightFront);

        // The method shouldLog() will determine if the recording interval has expired from the
        // last time
        logIt = shouldLog();

        // Log the "raw" stick values and max power calculation
        if( logIt ) {
            Log.d("RWS","Start of RAW Stick Values");
            Log.d("RWS",String.format("left stick y: %s", gamepad1.left_stick_y));
            Log.d("RWS",String.format("left stick x: %s", gamepad1.left_stick_x));
            Log.d("RWS",String.format("right stick x: %s", gamepad1.right_stick_x));
            Log.d("RWS","End of RAW Stick Values");
            Log.d("MAX",String.format("maxpower: %s",maxpwr));
            Log.d("RWP","Start of RAW Powers");
            Log.d("RWP",String.format("LeftBack: %s", LeftBack));
            Log.d("RWP",String.format("RightBack: %s", RightBack));
            Log.d("RWP",String.format("LeftFront: %s", LeftFront));
            Log.d("RWP",String.format("RightFront: %s", RightFront));
            Log.d("RWP","End of RAW Powers");

        }

        LeftBack = LeftBack / maxpwr;
        RightBack = RightBack / maxpwr;
        LeftFront = LeftFront / maxpwr;
        RightFront = RightFront / maxpwr;

        // Log normalized powers
        if ( logIt ) {
            Log.d("NML","Start of Normalized Powers");
            Log.d("NML",String.format("LeftBack: %s", LeftBack));
            Log.d("NML",String.format("RightBack: %s", RightBack));
            Log.d("NML",String.format("LeftFront: %s", LeftFront));
            Log.d("NML",String.format("RightFront: %s", RightFront));
            Log.d("NML","End of Normalized Powers");
        }

        LeftBack = (float) scaleInput(LeftBack);
        RightBack = (float) scaleInput(RightBack);
        LeftFront = (float) scaleInput(LeftFront);
        RightFront = (float) scaleInput(RightFront);

        // Log scaled powers
        if ( logIt ) {
            Log.d("SCL","Start of Scaled Powers");
            Log.d("SCL",String.format("LeftBack: %s", LeftBack));
            Log.d("SCL",String.format("RightBack: %s", RightBack));
            Log.d("SCL",String.format("LeftFront: %s", LeftFront));
            Log.d("SCL",String.format("RightFront: %s", RightFront));
            Log.d("SCL","End of Scaled Powers");
        }

        robot.setDriveMotorPower(LeftFront/2, RightFront/2, LeftBack/2, RightBack/2);

        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) { // "If any
            // of the dpad buttons are pressed on the
            // first controller...

            if (!nudging) { // ...if nudging is false (the robot is still being nudged)...

                if (gamepad1.dpad_up) robot.setDriveMotorPower(0.5, 0.5, 0.5 ,0.5); // ...if 'up' was pressed, the robot moves forward...

                else if (gamepad1.dpad_down) robot.setDriveMotorPower(-0.5, -0.5, -0.5, -0.5); // ...else if 'down' was pressed, the robot
                    // moves backward...

                else if (gamepad1.dpad_left) robot.setDriveMotorPower(-0.6, 0.6, 0.6, -0.6); // ...else if 'left' was pressed, the robot
                    // strafes left...

                else if (gamepad1.dpad_right) robot.setDriveMotorPower(0.6, -0.6, -0.6, 0.6); // ...else if 'right' was pressed, the robot
                    // strafes right...
            }

            nudgeCount++; // ...after this one loop cycle, the number of the nudgeCount goes up by one...

            if (nudgeCount > 5) { // ...if the number of the nudgeCount goes above 5...

                nudging = true; // ...nudging is true, so the robot cannot nudge anymore. This allows for 5 loop cycles
                // of movement...
            }
        }
        else { // ...else if nudging is true (the robot is no longer being nudged)...

            nudging = false; // ...nudging is returned to false, which allows nudging again if any of the dpad buttons
            // are pressed on the first controller...

            nudgeCount = 0; // ...and nudgeCount is reset to 0."
        }
    }

    @Override
    public void stop() {

    }

    /** scaleInput will attempt to smooth or scale joystick input when driving the
     * robot in teleop mode.  By smoothing the joystick input more controlled movement
     * of the robot will occur, especially at lower speeds.
     * <br><br>
     * To scale the input, 16 values are used that increase in magnitude, the algorithm
     * will determine where the input value roughly falls in the array by multiplying it
     * by 16, then will use the corresponding array entry from the scaleArray variable to
     * return a scaled value.
     * <br><br>
     * <b>Example 1:</b> dVal (the input value or value passed to this method) is set to 0.76
     * <br>
     * Stepping through the algorithm
     * <ol>
     *     <li> 0.76*16 = 12.16, but because we cast the calculations as an integer (int)
     *     we lose the .16 so the value just is 12, variable index now contains 12.  <b>Note:</b>
     *     the index variable will tell us which of the array entries in the scaleArray array to
     *     use.</li>
     *     <li> Check if the index is negative (less than zero), in this example the
     *     variable index contains a positive 12</li>
     *     <li> Check if the variable index is greater than 16, this is done so the
     *     algorithm does not exceed the number of entries in the scaleArray array</li>
     *     <li> Initialize the variable dScale to 0.0 (not really needed but we are
     *     just being safe)</li>
     *     <li> If dVal (value passed to this method) was initially negative, then
     *     set the variable dScale to the negative of the scaleArray(index), in this example
     *     dVal was initially 0.76 so not negative</li>
     *     <li> If dVal (value passed to this method) was initially positive, then
     *     set the variable dScale to the scaleArray(index), since index is 12, then
     *     scaleArray(12) = 0.60.  <b>Remember, in java the first array index is 0,
     *     this is why scaleArray(12) is not 0.50</b></li>
     *     <li> Return the dScale value (0.60)</li>
     * </ol>
     *
     * <br><br>
     * <b>Example 2</b> dVal (the input value or value passed to this method) is set to -0.43
     * <br>
     * Stepping through the algorithm
     * <ol>
     *     <li> -0.43*16 = -6.88, but because we cast the calculations as an integer (int)
     *     we lose the .88 so the value just is 12, variable index now contains -6.  <b>Note:</b>
     *     the index variable will tell us which of the array entries in the scaleArray array to
     *     use.</li>
     *     <li> Check if the index is negative (less than zero), in this example the
     *     variable index is negative, so make the negative a negative (essentially
     *     multiplying the variable index by -1, the variable index now contains 6</li>
     *     <li> Check if the variable index is greater than 16, this is done so the
     *     algorithm does not exceed the number of entries in the scaleArray array</li>
     *     <li> Initialize the variable dScale to 0.0 (not really needed but we are
     *     just being safe)</li>
     *     <li> If dVal (value passed to this method) was initially negative, then
     *     set the variable dScale to the negative of the scaleArray(index), in this example
     *     dVal was initially -0.43, so make sure to return a negative value of scaleArray(6).
     *     scaleArray(6) is equal to 0.18 and the negative of that is -0.18 <b>Remember,
     *     in java the first array index is 0, this is why scaleArray(6) is not 0.15</b></li>
     *     <li> Return the dScale value (-0.18)</li>
     * </ol>
     *
     * @param dVal the value to be scaled -between -1.0 and 1.0
     * @return the scaled value
     * <B>Author(s)</B> Unknown - copied from internet
     */
    double scaleInput(double dVal) {
        // in the floats.

        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    /**
     * findMaxPower - finds the maximum power of four power values
     * @param pwr1 first power
     * @param pwr2 second power
     * @param pwr3 third power
     * @param pwr4 fourth power
     *
     * @return maximum power of the four values
     * <B>Author(s):</B> Jason Sember and Steeeve
     */
    float findMaxPower(float pwr1, float pwr2, float pwr3, float pwr4) {

        float maxpwrA = Math.max(Math.abs(pwr1), Math.abs(pwr2));
        float maxpwrB = Math.max (Math.abs(pwr3), Math.abs(pwr4));
        float maxpwr = Math.max(Math.abs(maxpwrA), Math.abs(maxpwrB));

         if (maxpwr > 1.0) {

             return maxpwr;
         }
         else {

             return 1;
         }
    }

    /**
     * Will determine when the log should be updated with new data.  The previous time is subtracted
     * from the current time with a result of a time change or delta.  The time delta is then compared
     * to the log interval (LOG_INTERVAL) which represents the number of seconds (or fractions of a
     * second) that should expire before updating the log.  If the delta time is greater than the
     * log interval, this method will return a true.  If the delta time is less than the log interval,
     * this method will return a false.
     *
     * Under the covers, this method will set the variable pastTimeStamp, when the log interval time
     * has expired.
     *
     * @return boolean - TRUE, interval has expired and caller should write to log
     *                 - FALSE, interval has not expired and caller should not write to log
     */

    private boolean shouldLog() {

        if ( (currentTimeStamp-pastTimeStamp) > LOG_INTERVAL ) {
            pastTimeStamp = currentTimeStamp;
            return true;
        }
        else {
            return false;
        }

    }

}