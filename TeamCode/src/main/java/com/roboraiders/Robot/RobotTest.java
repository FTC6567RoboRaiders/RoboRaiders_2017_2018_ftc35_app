package com.roboraiders.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 *
 * This is a class that is a DIRECT copy (as of 1/17/18) of the Robot class.
 * This class was created to test new prototypes on the robot without changing the current Robot class code.
 *
 * Created by Alex Snyder on 1/17/18
 */

public class RobotTest {


    /* Robot Motors, Servos, CR Servos and Sensors */
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;
    public DcMotor motorRelic = null;
    public DcMotor motorGlyphInLeft = null;
    public DcMotor motorGlyphInRight = null;
    public DcMotor motorGlyphLift = null;

    public Servo servoJewel = null;
    public Servo servoElbow = null;
    public Servo servoRelicWrist = null;
    public Servo servoRelicGripper = null;
    public Servo servoGlyphRight = null;
    public Servo servoGlyphLeft = null;

    public ColorSensor colorSensor;
    //public DistanceSensor distanceSensor;
    public BNO055IMU imu;

    /* Local OpMode Members */
    public HardwareMap hwMap =  null;

    /* Public Variables */
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public Orientation angles;

    /** Constructor for RobotTest class, current does nothing but is needed since every class needs a constructor
     *
     */
    public RobotTest(){

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
        motorGlyphInLeft = hwMap.get(DcMotor.class, "glyph_In_Left");
        motorGlyphInRight = hwMap.get(DcMotor.class, "glyph_In_Right");
        motorGlyphLift = hwMap.get(DcMotor.class, "glyph_Lift");

        // Defines the directions the motors will spin
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorRelic.setDirection(DcMotor.Direction.FORWARD);
        motorGlyphInLeft.setDirection(DcMotor.Direction.REVERSE);
        motorGlyphInRight.setDirection(DcMotor.Direction.FORWARD);
        motorGlyphLift.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorRelic.setPower(0);
        motorGlyphInLeft.setPower(0);
        motorGlyphInRight.setPower(0);
        motorGlyphLift.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODER if encoders are installed, and we wouldn't use encoders for teleop, even if we
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //will use them in teleop.
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRelic.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlyphInLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlyphInRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlyphLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize servos
        servoJewel = hwMap.get(Servo.class, "servo_Jewel");
        servoElbow = hwMap.get(Servo.class, "servo_Elbow");
        servoRelicWrist = hwMap.get(Servo.class, "servo_Relic_Wrist");
        servoRelicGripper = hwMap.get(Servo.class, "servo_Relic_Gripper");
        servoGlyphRight = hwMap.get(Servo.class, "servo_glyph_right");
        servoGlyphLeft = hwMap.get(Servo.class, "servo_glyph_left");

        // Define and initialize sensors
        colorSensor = hwMap.get(ColorSensor.class, "sensor_color");
        //distanceSensor = hwMap.get(DistanceSensor.class, "sensor_distance");
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
        servoElbow.setPosition(0.55);
        servoRelicWrist.setPosition(0.0);       //VERIFY THIS THROUGH TESTING
        servoRelicGripper.setPosition(0.0);     //VERIFY THIS THROUGH TESTING
        servoGlyphRight.setPosition(0.4);
        servoGlyphLeft.setPosition(0.6);
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
     * This method will set the power for the two glyph in motors
     *
     * @param glyph power setting for the two glyph in motors
     */
    public void setGlyphInMotorPower(double glyph) {

        motorGlyphInLeft.setPower(glyph);
        motorGlyphInRight.setPower(glyph);
    }

    /**
     * This method will set the positions of the glyph grabber servos to an open positions large enough to grab the glyph.
     *
     */
    public void GlyphGrabberOpen() {

        servoGlyphLeft.setPosition(0.3);
        servoGlyphRight.setPosition(0.6);
    }

    /**
     * This method will set the positions of the glyph grabber to a closed position grabbing the glyph.
     *
     */
    public void GlyphGrabberClose() {

        servoGlyphLeft.setPosition(0.45);
        servoGlyphRight.setPosition(0.45);
    }

    /**
     * This method sets the power for the glyph lift.
     *
     * @param glyph the power to be set for the glyph lift.
     */

    public void setGlyphLiftPower(double glyph) {

        motorGlyphLift.setPower(glyph);
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

        servoRelicWrist.setPosition(1.0);
    }

    /**
     * This method will lower the wrist servo
     */
    public void wristDown() {

        double wristPosition = servoRelicWrist.getPosition();

        wristPosition = wristPosition - 0.01;
        servoRelicWrist.setPosition(wristPosition);
    }

    /**
     * This method will open the gripper servo
     */
    public void gripperOpen() {

        servoRelicGripper.setPosition(0.0);
    }

    /**
     * This method will close the gripper servo
     */
    public void gripperClose() {

        servoRelicGripper.setPosition(1.0);
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
     * This method will set the mode of all of the motors to run to position
     */
    /*public void runToPosition() {

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }*/

    /**
     * This method will set the target position of all of the motors to COUNTS
     *
     * @param COUNTS - the number of encoder counts the robot will travel that is equal
     * to the number of inches
     */
    /*public void setTargetPosition(double COUNTS) {

        motorFrontLeft.setTargetPosition((int) COUNTS);
        motorFrontRight.setTargetPosition((int) COUNTS);
        motorBackLeft.setTargetPosition((int) COUNTS);
        motorBackRight.setTargetPosition((int) COUNTS);
    }*/

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

        return 1; //distanceSensor.getDistance(DistanceUnit.CM);
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
     * This method expels a glyph in autonomous
     *
     * @throws InterruptedException
     */
    public void expelGlyph(Robot bot) throws InterruptedException {

        bot.motorGlyphInLeft.setPower(-0.75);
        bot.motorGlyphInRight.setPower(-0.75);

        Thread.sleep(1000);

        bot.motorGlyphInLeft.setPower(0.0);
        bot.motorGlyphInRight.setPower(0.0);
    }
}