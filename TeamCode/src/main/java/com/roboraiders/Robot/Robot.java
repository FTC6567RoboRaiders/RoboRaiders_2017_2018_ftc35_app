package com.roboraiders.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an Op Mode.
 *
 *
 * This class will be used to define all "basic" functions for our 2017 - 2018 robot.
 * <br>
 * Basic functions would include things like:
 * <ul>
 *     <li>Setting power to motors</li>
 *     <li>Obtaining color sensor information</li>
 *     <li>Obtaining distance sensor information</li>
 *     <li>Obtaining encoder counts</li>
 *     <li>Setting servo position(s)</li>
 * </ul>
 * <br>
 * Any advanced functionality, lets say like <u>following a white line</u> or <u>moving the
 * robot until a given distance from a barrier</u> should be handled in a different class
 * (e.g. a Driver class).
 * <br>
 * <b>Author(s):</b> Jason Sember, Alex Snyder, Katelin Zichittella, add your name here ...
 *
 */

public class Robot {

    /* Robot Motors, Servos, CR Servos and Sensors */
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;
    /*public DcMotor motorRelicAcross = null;
    public DcMotor motorGlyphIn = null;
    public DcMotor motorGlyphUpLeft = null;
    public DcMotor motorGlyphUpRight = null;*/

    public Servo servoJewel = null;
    public Servo servoArmLeft = null;
    public Servo servoArmRight = null;
    //public Servo servoRelicGripper = null;

    public CRServo servoWheelLeft = null;
    public CRServo servoWheelRight = null;
    //public CRServo servoRelicDown = null;

    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;
    public BNO055IMU imu;

    /* Local OpMode Members */
    public HardwareMap hwMap =  null;

    /* Public Variables */
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public Orientation angles;

    /** Constructor for Robot class, current does nothing but is needed since every class needs a constructor
     *
     */
    public Robot(){

    }

    /**
     * This method will initialize the robot
     *
     * @param ahwMap hardware map for the robot
     */
    public void initialize(HardwareMap ahwMap) {

        // Save reference to hardware map
        hwMap = ahwMap;

        // Define and initialize motors
        motorFrontLeft = hwMap.get(DcMotor.class, "left_Front");
        motorFrontRight = hwMap.get(DcMotor.class, "right_Front");
        motorBackLeft = hwMap.get(DcMotor.class, "left_Back");
        motorBackRight = hwMap.get(DcMotor.class, "right_Back");
        /*motorRelicAcross = hwMap.get(DcMotor.class, "relic_Across");
        motorGlyphIn = hwMap.get(DcMotor.class, "glyph_In");
        motorGlyphUpLeft = hwMap.get(DcMotor.class, "glyph_Up_Left");
        motorGlyphUpRight = hwMap.get(DcMotor.class, "glyph_Up_Right");*/

        // Defines the directions the motors will spin
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        /*motorRelicAcross.setDirection(DcMotor.Direction.FORWARD);
        motorGlyphIn.setDirection(DcMotor.Direction.FORWARD);
        motorGlyphUpLeft.setDirection(DcMotor.Direction.REVERSE);
        motorGlyphUpRight.setDirection(DcMotor.Direction.FORWARD);*/

        // Set all motors to zero power
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        /*motorRelicAcross.setPower(0);
        motorGlyphIn.setPower(0);
        motorGlyphUpLeft.setPower(0);
        motorGlyphUpRight.setPower(0);*/

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODER if encoders are installed, and we wouldn't use encoders for teleop, even if we
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //will use them in teleop.
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*motorRelicAcross.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlyphIn.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlyphUpLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlyphUpRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

        // Define and initialize servos
        servoJewel = hwMap.get(Servo.class, "servo_Jewel");
        servoArmLeft = hwMap.get(Servo.class, "servo_Arm_Left");
        servoArmRight = hwMap.get(Servo.class, "servo_Arm_Right");
        //servoRelicGripper = hwMap.get(Servo.class, "servo_Relic_Gripper");

        // Define and initialize CR servos
        servoWheelLeft = hwMap.get(CRServo.class, "servo_Wheel_Left");
        servoWheelRight = hwMap.get(CRServo.class, "servo_Wheel_Right");
        //servoRelicDown = hwMap.get(CRServo.class, "servo_Relic_Down");

        // Set all CR servos to zero power
        servoWheelLeft.setPower(0);
        servoWheelRight.setPower(0);
        //servoRelicDown.setPower(0);

        // Define and initialize sensors
        colorSensor = hwMap.get(ColorSensor.class, "sensor_color");
        distanceSensor = hwMap.get(DistanceSensor.class, "sensor_distance");
        imu = hwMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);
    }

    /**
     * This method will initialize all of the servos
     */
    public void initializeServos() {

        servoJewel.setPosition(0.1);
        servoArmLeft.setPosition(0.1);
        servoArmRight.setPosition(0.9);
        //servoRelicGripper.setPosition(0.3);
    }

    /**
     * This method will set the power for the drive motors
     *
     * @param leftFront power setting for the left front motor
     * @param rightFront power setting for the right front motor
     * @param leftBack power setting for the left back motor
     * @param rightBack power setting for the right back motor
     */
    public void setDriveMotorPower(double leftFront, double rightFront, double leftBack, double rightBack){

        motorFrontLeft.setPower(leftFront);
        motorFrontRight.setPower(rightFront);
        motorBackLeft.setPower(leftBack);
        motorBackRight.setPower(rightBack);
    }

    /**
     * This method will set the power for the relic across motor and the relic down CR servo
     *
     * @param across power setting for the relic across motor
     * @param down power setting for the relic down CR servo
     */
    public void setRelicAttachmentsPower(double across, double down) {

        //motorRelicAcross.setPower(across);
        //servoRelicDown.setPower(down);
    }

    public void armsClose() {

        servoArmLeft.setPosition(0.3);
        servoArmRight.setPosition(0.7);
    }

    public void armsRest() {

        servoArmLeft.setPosition(0.8);
        servoArmRight.setPosition(0.2);
    }

    public void wheelsIn() {

        servoWheelLeft.setPower(0.5);
        servoWheelRight.setPower(-0.5);
    }

    public void wheelsOut() {

        servoWheelLeft.setPower(-0.5);
        servoWheelRight.setPower(0.5);
    }

    public void wheelsRest() {

        servoWheelLeft.setPower(0.0);
        servoWheelRight.setPower(0.0);
    }

    /**
     * This method will pull a glyph in using the glyph intake assembly
     */
    /*public void glyphIn() {

        servoWheelLeft.setPower(0.5);
        servoWheelRight.setPower(-0.5);
        servoArmLeft.setPosition(0.3);
        servoArmRight.setPosition(0.7);
        //motorGlyphIn.setPower(0.5);
    }*/

    /**
     * This method will push a glyph out from the omni wheel assembly
     */
    /*public void glyphOut() {

        servoWheelLeft.setPower(-0.5);
        servoWheelRight.setPower(0.5);
        servoArmLeft.setPosition(0.3);
        servoArmRight.setPosition(0.7);
        //motorGlyphIn.setPower(-0.5);
    }*/

    /**
     * This method will set the glyph intake assembly to its resting phase
     */
    /*public void glyphRest() {

        servoWheelLeft.setPower(0.0);
        servoWheelRight.setPower(0.0);
        servoArmLeft.setPosition(0.8);
        servoArmRight.setPosition(0.2);
        //motorGlyphIn.setPower(0.0);
    }*/

    /**
     * This method will raise a glyph in the omni wheel assembly
     */
    /*public void glyphUp() {

        resetEncoders();
        runWithEncoders();

        double COUNTS = calculateCOUNTS(10);

        motorGlyphUpLeft.setPower(0.5);
        motorGlyphUpRight.setPower(0.5);

        while (getGlyphUpEncoderCount() < COUNTS) {


        }

        motorGlyphUpLeft.setPower(0.0);
        motorGlyphUpRight.setPower(0.0);

        runWithoutEncoders();
    }*/

    /**
     * This method will lower a glyph in the omni wheel assembly
     */
    /*public void glyphDown() {

        resetEncoders();
        runWithEncoders();

        double COUNTS = calculateCOUNTS(10);

        motorGlyphUpLeft.setPower(-0.5);
        motorGlyphUpRight.setPower(-0.5);

        while (getGlyphUpEncoderCount() > COUNTS) {


        }

        motorGlyphUpLeft.setPower(0.0);
        motorGlyphUpRight.setPower(0.0);

        runWithoutEncoders();
    }*/

    /**
     * This method will open the gripper servo
     */
    /*public void gripperOpen() {

        servoRelicGripper.setPosition(0.7);
    }*/

    /**
     * This method will close the gripper servo
     */
    /*public void gripperClose() {

        servoRelicGripper.setPosition(0.3);
    }*/

    /**
     * This method will reset the IMU
     */
    public void resetIMU() {

        imu.initialize(parameters);
    }

    /**
     * This method will return the current heading of the IMU
     *
     * @return getHeading() - the current heading of the IMU
     */
    public float getHeading() {

        float heading;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //this sets up the how we want the IMU to report data
        heading = Math.abs(angles.firstAngle); //heading is equal to the absolute value of the first angle

        return heading;
    }

    /**
     * This method will reset the encoder count of each motor to 0. It should be used before runWithEncoders
     * and getEncoderCount when strafing.
     */
    public void resetEncoders() {

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /*motorRelicAcross.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlyphIn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlyphUpLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlyphUpRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
    }

    /**
     * This method will set the mode of all of the drive train motors to run using encoder
     */
    public void runWithEncoders() {

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*motorRelicAcross.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorGlyphIn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorGlyphUpLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorGlyphUpRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
    }

    /**
     * This method will set the mode of all of the drive train motors to run without encoder
     */
    public void runWithoutEncoders() {

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * When strafing and using this method, the caller must call resetEncoders to reset the
     * encoder count to 0 (see above).
     *
     * @return averageCount - average encoder count (throws out high and low values and calculates
     * using middle two)
     */
    public int getSortedEncoderCount() {

        int[] encoderArray = new int[4];

        encoderArray[0] = Math.abs(motorFrontLeft.getCurrentPosition());
        encoderArray[1] = Math.abs(motorFrontRight.getCurrentPosition());
        encoderArray[2] = Math.abs(motorBackLeft.getCurrentPosition());
        encoderArray[3] = Math.abs(motorBackRight.getCurrentPosition());

        int I;
        int J;
        int Temp;

        for (I = 0; I < 3; I++) {

            for (J = I + 1; J < 4; J++) {

                if (encoderArray[I] < encoderArray[J]) {

                }
                else {

                    Temp = encoderArray[I];
                    encoderArray[I] = encoderArray[J];
                    encoderArray[J] = Temp;
                }
            }
        }
        int averageCount = (encoderArray[1] + encoderArray[2]) / 2;

        return averageCount;
    }

    /**
     * This method will return the absolute value of the current encoder count of the front left motor
     *
     * @return Math.abs(motorFrontLeft.getCurrentPosition()) - the absolute value of the current
     * encoder count of the front left motor
     */
    public int getEncoderCount() {

        return Math.abs(motorFrontLeft.getCurrentPosition());
    }

    /**
     * This method will return the average encoder count of the two "glyph up" motors
     *
     * @return averageCount - the average encoder count of the two "glyph up" motors
     */
    /*public int getGlyphUpEncoderCount() {

        int averageCount = (Math.abs(motorGlyphUpLeft.getCurrentPosition()) +
                Math.abs(motorGlyphUpRight.getCurrentPosition())) / 2;
        return averageCount;
    }*/

    /**
     * This method will return COUNTS after it is calculated from distance
     *
     * @param distance the desired distance in inches the robot will travel
     * @return COUNTS - the number of encoder counts the robot will travel that is equal
     * to the number of inches
     */
    public double calculateCOUNTS(int distance) {

        double COUNTS;

        int DIAMETER = 4; //diameter of wheel
        double GEAR_RATIO = (2.0/3.0); //gear ratio
        int PULSES = 1120; //encoder counts in one revolution
        double CIRCUMFERENCE = Math.PI * DIAMETER; //gives you circumference
        double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO; //gives the rotations
        COUNTS = PULSES * ROTATIONS; //gives the counts

        return COUNTS;
    }

    /**
     * This method will return the current distance of the distance sensor from an object
     * in inches
     *
     * @return distanceSensor.getDistance(DistanceUnit.CM) - the current distance of the
     * distance sensor from an object in inches
     */
    public double getDistance() {

        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * This method will return the color sensor reading of the selected color
     *
     * @param color our alliance color
     * @return colorIntensity - the color sensor reading of the selected color
     */
    public int getColorIntensity(String color) {

        int colorIntensity = 0; //the color sensor reading of the selected color

        if (color.equals("red")) { //if the selected color is red

            colorIntensity = colorSensor.red(); //colorIntensity will be the red reading
        }

        else if (color.equals("blue")) { //if the selected color is blue

            colorIntensity = colorSensor.blue(); //colorIntensity will be the blue reading
        }

        return colorIntensity; //the value will be returned so that it can be used
    }

    /**
     * This method sets the jewel servo position to the desired position.
     *
     * @param servoPosition the desired position of the jewel servo
     */
    public void setJewelServoPosition(double servoPosition) {

        servoJewel.setPosition(servoPosition);
    }

    /**
     * This method returns the current position of the jewel servo
     *
     * @return servoJewel.getPosition() - the current position of the jewel servo
     */
    public double getJewelServoPosition() {

        return servoJewel.getPosition();
    }
}