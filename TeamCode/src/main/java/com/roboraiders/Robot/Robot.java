package com.roboraiders.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Alex Snyder on 1/17/18
 */

public class Robot {

    /* Robot Motors, Servos, CR Servos and Sensors */
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;
    public DcMotor motorRelic = null;
    public DcMotor motorGlyphLift = null;

    public Servo servoJewel = null;
    public Servo servoElbow = null;
    public Servo servoRelicWrist = null;
    public Servo servoRelicGripper = null;
    public Servo servoGlyphUpper = null;
    public Servo servoGlyphLower = null;
    public Servo servoGlyphFlipper = null;

    public ColorSensor colorSensor;
    public ModernRoboticsI2cRangeSensor mrRangeSide;     // Back side range sensor
    public ModernRoboticsI2cRangeSensor mrRangeBack;     // Back back range sensor
    public ModernRoboticsI2cRangeSensor mrRangeFront;    // Front forward range sensor
    public BNO055IMU imu;

    /* Local OpMode Members */
    public HardwareMap hwMap =  null;

    /* Public Variables */
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public Orientation angles;
    public double integratedZAxis;
    public double iza_lastHeading = 0.0;
    public double iza_deltaHeading;
    public float iza_newHeading;
    public Orientation iza_angles;

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
        motorRelic = hwMap.get(DcMotor.class, "relic");
        motorGlyphLift = hwMap.get(DcMotor.class, "glyph_Lift");

        // Defines the directions the motors will spin
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorRelic.setDirection(DcMotor.Direction.FORWARD);
        motorGlyphLift.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorRelic.setPower(0);
        motorGlyphLift.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODER if encoders are installed, and we wouldn't use encoders for teleop, even if we
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //will use them in teleop.
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRelic.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlyphLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize servos
        servoJewel = hwMap.get(Servo.class, "servo_Jewel");
        servoElbow = hwMap.get(Servo.class, "servo_Elbow");
        servoRelicWrist = hwMap.get(Servo.class, "servo_Relic_Wrist");
        servoRelicGripper = hwMap.get(Servo.class, "servo_Relic_Gripper");
        servoGlyphUpper = hwMap.get(Servo.class, "servo_Glyph_Upper");
        servoGlyphLower = hwMap.get(Servo.class, "servo_Glyph_Lower");
        servoGlyphFlipper = hwMap.get(Servo.class, "servo_Glyph_Flipper");

        // Define and initialize sensors
        colorSensor = hwMap.get(ColorSensor.class, "sensor_color");
        mrRangeSide = hwMap.get(ModernRoboticsI2cRangeSensor.class, "mr_range_side");
        mrRangeBack = hwMap.get(ModernRoboticsI2cRangeSensor.class, "mr_range_back");
        mrRangeFront = hwMap.get(ModernRoboticsI2cRangeSensor.class, "mr_range_front");
        imu = hwMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);
    }

    /**
     * This method will initialize all of the servos
     */
    public void initializeServos() {

        servoJewel.setPosition(0.4);
        servoElbow.setPosition(0.0);
        servoGlyphUpper.setPosition(1.0);
        servoGlyphLower.setPosition(1.0);
        servoGlyphFlipper.setPosition(0.0);
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
     * This method sets the power for the glyph lift
     *
     * @param glyphLift the power to be set for the glyph lift
     */
    public void setGlyphLiftMotorPower(double glyphLift) {

        motorGlyphLift.setPower(glyphLift);
    }

    /**
     * This method will set the position of both glyph grabber servos to an open position large enough to grab the glyph
     */
    public void glyphGrabberBothOpen() {

        servoGlyphUpper.setPosition(1.0);
        servoGlyphLower.setPosition(1.0);
    }

    /**
     * This method will set the position of the upper glyph grabber servo to an open position large enough to grab the glyph
     */
    public void glyphGrabberUpperOpen() {

        servoGlyphUpper.setPosition(1.0);
    }

    /**
     * This method will set the position of the lower glyph grabber servo to an open position large enough to grab the glyph
     */
    public void glyphGrabberLowerOpen() {

        servoGlyphLower.setPosition(1.0);
    }

    /**
     * This method will set the position of the upper glyph grabber servo to a closed position, grabbing the glyph
     */
    public void glyphGrabberUpperClose() {

        servoGlyphUpper.setPosition(0.0);
    }

    /**
     * This method will set the position of the lower glyph grabber servo to a closed position, grabbing the glyph
     */
    public void glyphGrabberLowerClose() {

        servoGlyphLower.setPosition(0.0);
    }

    /**
     * This method will flip the glyph flipper servo
     */
    public void glyphFlip() {

        servoGlyphFlipper.setPosition(1.0);
    }

    /**
     * This method will return the glyph flipper servo to its original position
     */
    public void glyphFlipBack() {

        servoGlyphFlipper.setPosition(0.0);
    }

    /**
     * This method will set the power for the relic motor
     *
     * @param relic power setting for the relic motor
     */
    public void setRelicMotorPower(double relic) {

        motorRelic.setPower(relic);
    }

    /**
     * This method will raise the wrist servo
     */
    public void wristUp() {

        servoRelicWrist.setPosition(0.6);
    }

    /**
     * This method will lower the wrist servo
     */
    public void wristDown() {

       servoRelicWrist.setPosition(0.1);
    }

    /**
     * This method will open the gripper servo
     */
    public void gripperOpen() {

        servoRelicGripper.setPosition(1.0);
    }

    /**
     * This method will close the gripper servo
     */
    public void gripperClose() {

        servoRelicGripper.setPosition(0.0);
    }

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
    }

    /**
     * This method will set the mode of all of the motors to run using encoder
     */
    public void runWithEncoders() {

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
     * This method will return COUNTS after it is calculated from distance
     *
     * @param distance the desired distance in inches the robot will travel
     * @return COUNTS - the number of encoder counts the robot will travel that is equal
     * to the number of inches
     */
    public double calculateCOUNTS(double distance) {

        double COUNTS;

        int DIAMETER = 4; //diameter of wheel
        double GEAR_RATIO = (2.0 / 3.0); //gear ratio
        int PULSES = 1120; //encoder counts in one revolution
        double CIRCUMFERENCE = Math.PI * DIAMETER; //gives you circumference
        double ROTATIONS = (distance / CIRCUMFERENCE) * GEAR_RATIO; //gives the rotations
        COUNTS = PULSES * ROTATIONS; //gives the counts

        return COUNTS;
    }

    /**
     * This method will return the current distance of the side distance sensor from an object
     * in inches
     *
     * @return mrRangeSide.getDistance(DistanceUnit.INCH) - the current distance of the
     * side distance sensor from an object in inches
     */
    public double getSideDistance() {

        return mrRangeSide.getDistance(DistanceUnit.INCH);

    }

    /**
     * This method will return the current distance of the back distance sensor from an object
     * in inches
     *
     * @return mrRangeBack.getDistance(DistanceUnit.INCH) - the current distance of the
     * back distance sensor from an object in inches
     */
    public double getBackDistance() {

        return mrRangeBack.getDistance(DistanceUnit.INCH);
    }

    /**
     * This method will return the current distance of the front distance sensor from an object
     * in inches
     *
     * @return mrRangeFront.getDistance(DistanceUnit.INCH) - the current distance of the
     * front distance sensor from an object in inches
     */
    public double getFrontDistance() {

        return mrRangeFront.getDistance(DistanceUnit.INCH);
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
     * This method sets the jewel servo position to the desired position
     *
     * @param servoPosition the desired position of the jewel servo
     */
    public void setJewelServoPosition(double servoPosition) {

        servoJewel.setPosition(servoPosition);
    }

    /**
     * This method sets the elbow servo position to the desired position
     *
     * @param servoPosition the desired position of the elbow servo
     */
    public void setElbowServoPosition(double servoPosition) {

        servoElbow.setPosition(servoPosition);
    }

    /**
     * This method returns the current position of the jewel servo
     *
     * @return servoJewel.getPosition() - the current position of the jewel servo
     */
    public double getJewelServoPosition() {

        return servoJewel.getPosition();
    }

    /**
     * This method returns the current position of the elbow servo
     *
     * @return servoElbow.getPosition() - the current position of the elbow servo
     */
    public double getElbowServoPosition() {

        return servoElbow.getPosition();
    }

    /**
     * This method will lift the glyph up slightly at the beginning of autonomous
     */
    public void liftGlyph() throws InterruptedException {

        glyphGrabberLowerClose();
        Thread.sleep(500);

        motorGlyphLift.setPower(0.5);
        Thread.sleep(750);

        motorGlyphLift.setPower(0.0);
        Thread.sleep(250);
    }

    /**
     * This method will lower the glyph down before placing in the cryptobox in autonomous
     */
    public void lowerGlyph() throws InterruptedException {

        motorGlyphLift.setPower(-0.5);
        Thread.sleep(500);

        motorGlyphLift.setPower(0.0);
        Thread.sleep(250);

        glyphGrabberLowerOpen();
        Thread.sleep(500);
    }

    /**
     * This will mimic the Modern Robotics getIntegratedZAxis method
     *
     * @return integratedZAxis - the integrated z axis
     */
    public double getIntegratedZAxis() {

        // This sets up the how we want the IMU to report data
        iza_angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Obtain the heading (Z-Axis)
        iza_newHeading = iza_angles.firstAngle;

        // Calculate the change in the heading from the previous heading
        iza_deltaHeading = iza_newHeading - iza_lastHeading;

        // Bosch IMU wraps at 180, so compensate for this
        if (iza_deltaHeading <= -180.0) {

            iza_deltaHeading += 360.0;
        }
        else if (iza_deltaHeading >= 180.0) {

            iza_deltaHeading -= 360;
        }

        // Calculate the integratedZAxis
        integratedZAxis += iza_deltaHeading;

        // Save the current heading for the next call to this method
        iza_lastHeading = iza_newHeading;

        return integratedZAxis;
    }
}