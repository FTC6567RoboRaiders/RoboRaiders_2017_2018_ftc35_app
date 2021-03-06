package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboraiders.Robot.Robot;

/**
 * Created by Jason Sember on 9/23/2017.
 */

@TeleOp
@Disabled

public class TeleOpNewMecanumTank extends OpMode {

    public Robot robot = new Robot();

    /* Define variables */
    float LeftBack;   // Power for left back motor
    float RightBack;  // Power for right back motor
    float LeftFront;  // Power for left front motor
    float RightFront; // Power for right front motor

    float maxpwr;     // Maximum power of the four motors

    @Override
    public void init() {

        robot.initialize(hardwareMap);

        telemetry.addData("Initialized", true);
        telemetry.update() ;
    }

    @Override
    public void loop() {

        /*----------------------------------------------------------------------*/
        /* For tank drive mecanum, the left and right joystick in the y-axis    */
        /* direction on gamepad 1 control the left and right side motors        */
        /* respectively.  The left joystick in the x-axis controls the strafing */
        /* motion of the robot                                                  */
        /*                                                                      */
        /* GAMEPAD1                                                             */
        /* Left Joystick  - y-axis forward/backward motion of the left wheels   */
        /* Left Joystick  - x-axis strafing left or right                       */
        /* Right Joystick - y-axis forward/backward motion of the right wheels  */
        /*----------------------------------------------------------------------*/
        LeftBack = gamepad1.left_stick_y + gamepad1.left_stick_x;
        RightBack = gamepad1.right_stick_y - gamepad1.left_stick_x;
        LeftFront = gamepad1.left_stick_y - gamepad1.left_stick_x;
        RightFront = gamepad1.right_stick_y + gamepad1.left_stick_x;

        maxpwr = findMaxPower(LeftBack, LeftFront, RightBack, RightFront);

        LeftBack = LeftBack / maxpwr;
        RightBack = RightBack / maxpwr;
        LeftFront = LeftFront / maxpwr;
        RightFront = RightFront / maxpwr;

        LeftBack = (float) scaleInput(LeftBack);
        RightBack = (float) scaleInput(RightBack);
        LeftFront = (float) scaleInput(LeftFront);
        RightFront = (float) scaleInput(RightFront);

        robot.setDriveMotorPower(LeftFront/2, RightFront/2, LeftBack/2, RightBack/2);
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
        return Math.max(Math.abs(maxpwrA), Math.abs(maxpwrB));
    }
}