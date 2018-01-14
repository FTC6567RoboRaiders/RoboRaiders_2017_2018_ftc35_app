package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.roboraiders.Robot.Robot;

/**
 * Created by Steve Kocik on 1/14/2018.
 */

@TeleOp(name = "FrankenBot", group = "Test Bots")

public class FrankenBot extends OpMode {

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    Servo lefty = null;
    Servo righty = null;

    public boolean currStateA = false;   // A is open
    public boolean prevStateA = false;
    public boolean currStateB = false;   // B is close
    public boolean prevStateB = false;

    @Override
    public void init() {

        leftMotor = hardwareMap.get(DcMotor.class, "motorLeft");
        rightMotor = hardwareMap.get(DcMotor.class, "motorRight");

        righty = hardwareMap.get(Servo.class, "righty");
        lefty = hardwareMap.get(Servo.class, "lefty");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        currStateA = false;
        prevStateA = false;
        currStateB = false;
        prevStateB = false;


        float left = gamepad1.left_stick_y;
        float right = gamepad1.right_stick_y;

        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);

        left = (float) scaleInput(left);
        right = (float) scaleInput(right);

        leftMotor.setPower(left);
        rightMotor.setPower(right);

        //  setMotorPower(left, right);

        // open
        currStateA = gamepad1.a;
        if (currStateA && currStateA != prevStateA) {

            righty.setPosition(0.75);
            lefty.setPosition(0.25);
            prevStateA = currStateA;
        } else if (!currStateB && currStateB != prevStateB) {

            prevStateB = currStateB;
        }

        // close the lefty and righty
        currStateB = gamepad1.b;
        if (currStateB && currStateB != prevStateB) {

            righty.setPosition(0.4);
            lefty.setPosition(0.6);

            prevStateB = currStateB;
        } else if (!currStateB && currStateB != prevStateB) {

            prevStateB = currStateB;
        }


    }

    public void setMotorPower(float left, float right) {

        leftMotor.setPower(left);
        rightMotor.setPower(right);
    }


    /**
     * scaleInput will attempt to smooth or scale joystick input when driving the
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
     * <li> 0.76*16 = 12.16, but because we cast the calculations as an integer (int)
     * we lose the .16 so the value just is 12, variable index now contains 12.  <b>Note:</b>
     * the index variable will tell us which of the array entries in the scaleArray array to
     * use.</li>
     * <li> Check if the index is negative (less than zero), in this example the
     * variable index contains a positive 12</li>
     * <li> Check if the variable index is greater than 16, this is done so the
     * algorithm does not exceed the number of entries in the scaleArray array</li>
     * <li> Initialize the variable dScale to 0.0 (not really needed but we are
     * just being safe)</li>
     * <li> If dVal (value passed to this method) was initially negative, then
     * set the variable dScale to the negative of the scaleArray(index), in this example
     * dVal was initially 0.76 so not negative</li>
     * <li> If dVal (value passed to this method) was initially positive, then
     * set the variable dScale to the scaleArray(index), since index is 12, then
     * scaleArray(12) = 0.60.  <b>Remember, in java the first array index is 0,
     * this is why scaleArray(12) is not 0.50</b></li>
     * <li> Return the dScale value (0.60)</li>
     * </ol>
     * <p>
     * <br><br>
     * <b>Example 2</b> dVal (the input value or value passed to this method) is set to -0.43
     * <br>
     * Stepping through the algorithm
     * <ol>
     * <li> -0.43*16 = -6.88, but because we cast the calculations as an integer (int)
     * we lose the .88 so the value just is 12, variable index now contains -6.  <b>Note:</b>
     * the index variable will tell us which of the array entries in the scaleArray array to
     * use.</li>
     * <li> Check if the index is negative (less than zero), in this example the
     * variable index is negative, so make the negative a negative (essentially
     * multiplying the variable index by -1, the variable index now contains 6</li>
     * <li> Check if the variable index is greater than 16, this is done so the
     * algorithm does not exceed the number of entries in the scaleArray array</li>
     * <li> Initialize the variable dScale to 0.0 (not really needed but we are
     * just being safe)</li>
     * <li> If dVal (value passed to this method) was initially negative, then
     * set the variable dScale to the negative of the scaleArray(index), in this example
     * dVal was initially -0.43, so make sure to return a negative value of scaleArray(6).
     * scaleArray(6) is equal to 0.18 and the negative of that is -0.18 <b>Remember,
     * in java the first array index is 0, this is why scaleArray(6) is not 0.15</b></li>
     * <li> Return the dScale value (-0.18)</li>
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
     *
     * @param pwr1 first power
     * @param pwr2 second power
     * @param pwr3 third power
     * @param pwr4 fourth power
     * @return maximum power of the four values
     * <B>Author(s):</B> Jason Sember and Steeeve
     */
    float findMaxPower(float pwr1, float pwr2, float pwr3, float pwr4) {

        float maxpwrA = Math.max(Math.abs(pwr1), Math.abs(pwr2));
        float maxpwrB = Math.max(Math.abs(pwr3), Math.abs(pwr4));
        float maxpwr = Math.max(Math.abs(maxpwrA), Math.abs(maxpwrB));

        if (maxpwr > 1.0) {

            return maxpwr;
        } else {

            return 1;
        }
    }
}