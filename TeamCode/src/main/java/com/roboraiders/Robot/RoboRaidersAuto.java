package com.roboraiders.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Alex Snyder on 10/8/17.
 */

public abstract class RoboRaidersAuto extends LinearOpMode {

    public VuforiaLocalizer vuforia;
    public VuforiaTrackable relicTemplate;
    public boolean currStateDistance = false;
    public boolean prevStateDistance = false;
    public String pictograph = "UNKNOWN";

    /**
     * This method will initialize Vuforia in autonomous op modes
     *
     * @param hwMap the hardware map we will be using
     */
    public void vuforiaInitialization(HardwareMap hwMap) {

        // Vuforia initialization
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AedUDNP/////AAAAGXH2ZpUID0KanSX9ZSR37LKFSFokxIqmy/g0BNepdA9EepixxnO00qygLnMJq3Fg9gZxnkUJaKgk14/UjhxPWVQIs90ZXJLc21NvQvOeZ3dOogagVP8yFnFQs2xCijGmC/CE30ojlAnbhAhqz1y4tZPW2QkK5Qt0xCakTTSAw3KPQX2mZxX+qMxI2ljrN0eaxaKVnKnAUl8x3naF1mez7f9c8Xdi1O5auL0ePdG6bJhWjEO1YwpSd8WkSzNDEkmw20zpQ7zaOOPw5MeUQUr9vAS0fef0GnLjlS1gb67ajUDlEcbbbIeSrLW/oyRGTil8ueQC2SWafdspSWL3SJNaQKWydies23BxJxM/FoLuYYjx";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();
    }

    /**
     * This method will to push the jewel off the platform that is not the current alliance color
     *
     * @param bot           the bot currently being worked on
     * @param allianceColor the color of your alliance
     */
    public void selectJewel(Robot bot, String allianceColor) throws InterruptedException {

        //Does the robot need to move forward at all? Or no? Discuss with programming team. This program assumes no.
        //assuming color sensor is mounted facing right

        //assuming red alliance

        if (allianceColor.equals("red")) { //red alliance

            if (bot.getColorIntensity("red") > bot.getColorIntensity("blue"))  { //if the ball on the right is red

                telemetry.addLine().addData("Red", bot.getColorIntensity("red"));
                telemetry.addLine().addData("Blue", bot.getColorIntensity("blue"));
                telemetry.update();

                imuTurn(bot, 25, 0.25, "left"); //pivot left
                Thread.sleep(500);

                bot.setJewelServoPosition(0.1); //move arm back to initialization position
                Thread.sleep(1000);

                imuTurn(bot, 25, 0.25, "right"); //pivot right to original position
                Thread.sleep(500);
            }
            else { //the ball on the right is blue

                telemetry.addLine().addData("Red", bot.getColorIntensity("red"));
                telemetry.addLine().addData("Blue", bot.getColorIntensity("blue"));
                telemetry.update();

                imuTurn(bot, 25, 0.25, "right"); //pivot right
                Thread.sleep(500);

                bot.setJewelServoPosition(0.1); //move arm back to initialization position
                Thread.sleep(1000);

                imuTurn(bot, 25, 0.25, "left"); //pivot left to original position
                Thread.sleep(500);
            }
        }

        //assuming blue alliance

        else if (allianceColor.equals("blue")) { //not red alliance (blue alliance)

            if (bot.getColorIntensity("blue") > bot.getColorIntensity("red")) { //if the ball on the right is blue

                telemetry.addLine().addData("Red", bot.getColorIntensity("red"));
                telemetry.addLine().addData("Blue", bot.getColorIntensity("blue"));
                telemetry.update();

                imuTurn(bot, 25, 0.25, "left"); //pivot left
                Thread.sleep(500);

                bot.setJewelServoPosition(0.1); //move arm back to initialization position
                Thread.sleep(1000);

                imuTurn(bot, 25, 0.25, "right"); //pivot right to original position
                Thread.sleep(500);
            }
            else { //the ball on the right is blue

                telemetry.addLine().addData("Red", bot.getColorIntensity("red"));
                telemetry.addLine().addData("Blue", bot.getColorIntensity("blue"));
                telemetry.update();

                imuTurn(bot, 25, 0.25, "right"); //pivot right
                Thread.sleep(500);

                bot.setJewelServoPosition(0.1); //move arm back to initialization position
                Thread.sleep(1000);

                imuTurn(bot, 25, 0.25, "left"); //pivot left to original position
                Thread.sleep(500);
            }
        }
    }

    /**
     * This method directs the robot to place the glyph in the key column
     *
     * @param bot           the bot currently being worked on
     * @param allianceColor the color of your alliance
     * @param pictograph    the name of the pictograph as determined by getRelicRecoveryVuMark()
     * @throws InterruptedException
     */
    public void selectColumn(Robot bot, String allianceColor, String pictograph) throws InterruptedException {

        if (allianceColor.equals("red")) { //if we are on the red side

            if (pictograph.equals("LEFT")) { //if the pictograph says that the key column is the left column

                encodersMove(bot, 15, 0.5, "backward"); //move backward 15 inches until in front of the left column
                Thread.sleep(250);
            }
            else if (pictograph.equals("CENTER")) { //else if the pictograph says that the key column is the center column

                encodersMove(bot, 10, 0.5, "backward"); //move backward 10 inches until in front of the center column
                Thread.sleep(250);
            }
            else if (pictograph.equals("RIGHT")) { //else if the pictograph says that the key column is the right column

                encodersMove(bot, 5, 0.5, "backward"); //move backward 5 inches until in front of the right column
                Thread.sleep(250);
            }
            else if (pictograph.equals("UNKNOWN")) { //else if the pictograph cannot determine which column is the key column

                encodersMove(bot, 10, 0.5, "backward"); //move backward 10 inches until in front of the center column (default)
                Thread.sleep(250);
            }
        }
        else if (allianceColor.equals("blue")) { //else if we are on the blue side

            if (pictograph.equals("LEFT")) { //if the pictograph says that the key column is the left column

                encodersMove(bot, 5, 0.5, "forward"); //move forward 5 inches until in front of the left column
                Thread.sleep(250);
            }
            else if (pictograph.equals("CENTER")) { //else if the pictograph says that the key column is the center column

                encodersMove(bot, 10, 0.5, "forward"); //move forward 10 inches until in front of the center column
                Thread.sleep(250);
            }
            else if (pictograph.equals("RIGHT")) { //else if the pictograph says that the key column is the right column

                encodersMove(bot, 15, 0.5, "forward"); //move forward 15 inches until in front of the right column
                Thread.sleep(250);
            }
            else if (pictograph.equals("UNKNOWN")) { //else if the pictograph cannot determine which column is the key column

                encodersMove(bot, 10, 0.5, "forward"); //move forward 10 inches until in front of the center column (default)
                Thread.sleep(250);
            }
        }

        imuTurn(bot, 90, 0.5, "left"); //turn left 90 degrees
        Thread.sleep(250);

        placeGlyph(bot); //run the method placeGlyph
        Thread.sleep(250);
    }

    /**
     * This method will turn the robot right or left a certain angle measure using the IMU
     *
     * @param bot       the bot currently being worked on
     * @param degrees   the desired number of degrees to turn
     * @param power     the desired power the wheel motors will run at
     * @param direction the direction the robot is turning; either right or left
     */
    public void imuTurn(Robot bot, float degrees, double power, String direction) { //gets hardware from
        //Robot and defines degrees as a
        //float, power as a double, and direction as a string

        bot.resetIMU(); //resets IMU angle to zero

        bot.getHeading(); //returns the current heading of the IMU

        if (direction.equals("right")) { //if the desired direction is right

            bot.setDriveMotorPower(power, -power, power, -power); //the robot will turn right
        }
        else if (direction.equals("left")) { //if the desired direction is left

            bot.setDriveMotorPower(-power, power, -power, power); //the robot will turn left
        }

        while (bot.getHeading() < (degrees - 20) && opModeIsActive()) { //while the value of getHeading is
            //less then the degree value
            //and while opMode is active continue the while loop

            telemetry.addData("Heading", bot.getHeading()); //feedback of getHeading value
            telemetry.update(); //continuous update
        }

        bot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stops robot
    }

    /**
     * This program will make the robot move forwards, backwards, right, or left with encoders
     *
     * @param bot       the robot currently being worked on
     * @param distance  the distance the robot should travel in inches
     * @param power     the speed the robot will travel at
     * @param direction the direction the robot will travel: either forward, backward, right, or left
     */
    public void encodersMove(Robot bot, int distance, double power, String direction) { //sets the parameters

        bot.resetEncoders(); //resets encoders
        bot.runWithEncoders(); //sets the mode back to run with encoder

        double COUNTS = bot.calculateCOUNTS(distance); //COUNTS is now equal to the value calculated

        if (direction.equals("forward")) { //if the desired direction is forward

            bot.setDriveMotorPower(power, power, power, power); //start driving forward

            while (bot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still less than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", bot.getSortedEncoderCount());
                telemetry.update();
            }

            bot.setDriveMotorPower(0, 0, 0, 0); //stop the robot
        }
        else if (direction.equals("backward")) { //if the desired direction is backward

            bot.setDriveMotorPower(-power, -power, -power, -power); //start driving backward

            while (bot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still greater than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", bot.getSortedEncoderCount());
                telemetry.update();
            }

            bot.setDriveMotorPower(0, 0, 0, 0); //stop the robot
        }
        else if (direction.equals("right")) { //if the desired direction is right

            bot.setDriveMotorPower(power, -power, -power, power); //start strafing right

            while (bot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still less than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", bot.getSortedEncoderCount());
                telemetry.update();
            }

            bot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
        }
        else if (direction.equals("left")) { //if the desired direction is left

            bot.setDriveMotorPower(-power, power, power, -power); //start strafing left

            while (bot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still greater than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", bot.getSortedEncoderCount());
                telemetry.update();
            }

            bot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
        }

        bot.runWithoutEncoders(); //sets the mode back to run without encoder
    }

    /**
     * This method will strafe the robot right until the distance sensor has detected the robot has
     * passed a certain number of dividers
     *
     * @param bot             the bot currently being worked on
     * @param dividersTarget  the desired number of dividers to pass
     * @param power           the desired power the wheel motors will run at
     * @param desiredDistance the desired distance from the target
     */
    public void distanceSensorCount(Robot bot, int dividersTarget, double power, int desiredDistance) { //establishes
        //parameters for method

        /*double dividersDistance = 0; //counts the number of times that the robot hits the divider with the distance sensor

        bot.setDriveMotorPower(-power, power, power, -power); //robot is moving left at whatever power is specified

        while (dividersDistance < dividersTarget && opModeIsActive()) { //while the robot has not yet hit the specified number of dividers
            //and the opMode has not been stopped

            telemetry.addData("Distance", bot.getDistance());
            telemetry.update();

            if (bot.getDistance() <= desiredDistance) { //if the distance of the
                //sensor is less than the
                //pre-specified value, aka the robot is passing
                //close to the divider

                currStateDistance = true; //the robot is currently passing a divider
                telemetry.addData("Distance Sensor", "Is In Front of a Divider");
                telemetry.update();
            }
            else { //if the distance of the sensor is greater than the
                //pre-specified value, aka the robot is between dividers

                currStateDistance = false; //the robot is not currently passing a divider
                telemetry.addData("Digital Sensor", "Is Not In Front of a Divider");
                telemetry.update();
            }

            if (currStateDistance && currStateDistance != prevStateDistance) { //if the robot sees the
                //divider and it didn't see the divider before
                //basically, if the robot sees the divider

                dividersDistance++; //add 1 to the current "dividersDistance" variable
                prevStateDistance = currStateDistance; //now the previous state is the same as the current state
            }
            else if (!currStateDistance && currStateDistance != prevStateDistance) { //if the touch sensor
                //is just starting to not be pressed:

                prevStateDistance = currStateDistance; //now the previous state equals the current state,
                //don't change anything to the "dividersDistance" variable
            }
        }

        bot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot*/
    }

    /**
     * This method will determine the name of the pictograph the robot sees
     */
    public void getRelicRecoveryVuMark() {

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate); //the variable vuMark is now the name of
        //the pictograph the robot currently sees

        if (vuMark.equals(RelicRecoveryVuMark.LEFT)) { //if vuMark is left

            pictograph = "LEFT"; //pictograph is set equal to left
        }
        else if (vuMark.equals(RelicRecoveryVuMark.CENTER)) { //else if vuMark is center

            pictograph = "CENTER"; //pictograph is set equal to center
        }
        else if (vuMark.equals(RelicRecoveryVuMark.RIGHT)) { //else if vuMark is right

            pictograph = "RIGHT"; //pictograph is set equal to right
        }
        else { //else if the robot cannot determine the name of the pictograph

            pictograph = "UNKNOWN"; //pictograph is set equal to unknown
        }
    }

    /**
     * This program lowers the servoJewel arm
     *
     * @param bot the robot currently being worked on
     * @param finalServoPosition the final position that the servo arm will stop at
     * @throws InterruptedException
     */
    public void lowerArm(Robot bot, double finalServoPosition) throws InterruptedException {

        double servoPosition = bot.getJewelServoPosition(); //sets getPosition() to servoPosition

        while (servoPosition < finalServoPosition && opModeIsActive()) { //while the op mode is active and while the servo position variable is less
            //than 0.99

            servoPosition = servoPosition + 0.05;          //add 0.05 to the current servoPosition variable
            bot.setJewelServoPosition(servoPosition);
            Thread.sleep(75);                              //wait 0.02 seconds (20 milliseconds)
        }

        Thread.sleep(250);
    }

    /**
     * This method opens the arms, backs up 4 inches, closes arms, moves forward 5 inches to push glyph in column,
     * and moves backwards 1 inch to stay in the safe zone but not be touching the glyph. This is also steps 3-7
     * of the pseudocode for Version 2 of autonomous we developed on Dec. 6th, 2017.
     *
     *
     * @param bot - the bot currently being worked on
     * @throws InterruptedException
     */
    public void placeGlyph(Robot bot) throws InterruptedException {

        bot.armsOpen();
        Thread.sleep(250);

        encodersMove(bot, 4, 0.5, "backwards");
        Thread.sleep(250);

        bot.armsClose();
        Thread.sleep(250);

        encodersMove(bot, 5, 0.5, "forward");
        Thread.sleep(250);

        encodersMove(bot, 1, 0.5, "backward");
        Thread.sleep(250);
    }
}