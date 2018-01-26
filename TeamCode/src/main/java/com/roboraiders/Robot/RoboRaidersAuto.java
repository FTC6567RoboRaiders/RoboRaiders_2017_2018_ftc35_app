package com.roboraiders.Robot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    public String pictograph = "UNKNOWN";
    public boolean cur_Y_ButtonState = false;
    public boolean prev_Y_ButtonState = false;
    public double distanceFromWall = 0;

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
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
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

                bot.setElbowServoPosition(1.0);
                Thread.sleep(500);

                bot.setElbowServoPosition(0.54);
                Thread.sleep(500);

                bot.setJewelServoPosition(0.1); //move arm back to initialization position
                Thread.sleep(1000);

            }
            else { //the ball on the right is blue

                telemetry.addLine().addData("Red", bot.getColorIntensity("red"));
                telemetry.addLine().addData("Blue", bot.getColorIntensity("blue"));
                telemetry.update();

                bot.setElbowServoPosition(0.0);
                Thread.sleep(500);

                bot.setElbowServoPosition(0.54);
                Thread.sleep(500);

                bot.setJewelServoPosition(0.1); //move arm back to initialization position
                Thread.sleep(1000);

            }
        }

        //assuming blue alliance

        else if (allianceColor.equals("blue")) { //not red alliance (blue alliance)

            if (bot.getColorIntensity("blue") > bot.getColorIntensity("red")) { //if the ball on the right is blue

                telemetry.addLine().addData("Red", bot.getColorIntensity("red"));
                telemetry.addLine().addData("Blue", bot.getColorIntensity("blue"));
                telemetry.update();

                bot.setElbowServoPosition(1.0);
                Thread.sleep(500);

                bot.setElbowServoPosition(0.54);
                Thread.sleep(500);

                bot.setJewelServoPosition(0.1); //move arm back to initialization position
                Thread.sleep(1000);

            }
            else { //the ball on the left is blue

                telemetry.addLine().addData("Red", bot.getColorIntensity("red"));
                telemetry.addLine().addData("Blue", bot.getColorIntensity("blue"));
                telemetry.update();

                bot.setElbowServoPosition(0.0);
                Thread.sleep(500);

                bot.setElbowServoPosition(0.54);
                Thread.sleep(500);

                bot.setJewelServoPosition(0.1); //move arm back to initialization position
                Thread.sleep(1000);
            }
        }

        Thread.sleep(1000);
    }

    /**
     * This method directs the robot to place the glyph in the key column
     *
     * @param bot           the bot currently being worked on
     * @param allianceColor the color of your alliance
     * @param alliancePlacement the placement of the alliance: either close or far
     * @param pictograph    the name of the pictograph as determined by getRelicRecoveryVuMark()
     * @throws InterruptedException
     */
    public void selectColumn(Robot bot, String allianceColor, String alliancePlacement, String pictograph) throws InterruptedException {

        if (allianceColor.equals("red")) { //if we are on the red side

            if (alliancePlacement.equals("close")) { //if we are close to the audience

                if (pictograph.equals("LEFT")) { //if the pictograph says that the key column is the left column

                    encodersMove(bot, 36, 0.5, "backward"); //move backward 36 inches until in front of the left column
                    Thread.sleep(250);
                }
                else if (pictograph.equals("CENTER")) { //else if the pictograph says that the key column is the center column

                    encodersMove(bot, 28, 0.5, "backward"); //move backward 28 inches until in front of the center column
                    Thread.sleep(250);
                }
                else if (pictograph.equals("RIGHT")) { //else if the pictograph says that the key column is the right column

                    encodersMove(bot, 22, 0.5, "backward"); //move backward 22 inches until in front of the right column
                    Thread.sleep(250);
                }
                else if (pictograph.equals("UNKNOWN")) { //else if the pictograph cannot determine which column is the key column

                    encodersMove(bot, 28, 0.5, "backward"); //move backward 28 inches until in front of the center column (default)
                    Thread.sleep(250);
                }

                imuTurn(bot, 90, 0.5, "left"); //turn left 90 degrees
                Thread.sleep(250);
            }

            else if (alliancePlacement.equals("far")) { //if we are far from the audience

                encodersMove(bot, 20, 0.5, "backward"); //drive backward
                Thread.sleep(250);

                imuTurn(bot, 90, 0.5, "right"); //turn right 90 degrees
                Thread.sleep(250);

                if (pictograph.equals("LEFT")) { //if the pictograph says that the key column is the left column

                    encodersMove(bot, 13, 0.5, "forward"); //move forward 13 inches until in front of the left column
                    Thread.sleep(250);
                }
                else if (pictograph.equals("CENTER")) { //else if the pictograph says that the key column is the center column

                    encodersMove(bot, 8, 0.5, "forward"); //move forward 8 inches until in front of the center column
                    Thread.sleep(250);
                }
                else if (pictograph.equals("RIGHT")) { //else if the pictograph says that the key column is the right column

                    encodersMove(bot, 2, 0.5, "forward"); //move forward 2 inches until in front of the right column
                    Thread.sleep(250);
                }
                else if (pictograph.equals("UNKNOWN")) { //else if the pictograph cannot determine which column is the key column

                    encodersMove(bot, 8, 0.5, "forward"); //move forward 8 inches until in front of the center column (default)
                    Thread.sleep(250);
                }

                imuTurn(bot, 90, 0.5, "right"); //turn right 90 degrees
                Thread.sleep(250);
            }
        }
        else if (allianceColor.equals("blue")) { //else if we are on the blue side

            if (alliancePlacement.equals("close")) {  //if we are close to the audience

                if (pictograph.equals("LEFT")) { //if the pictograph says that the key column is the left column

                    encodersMove(bot, 22, 0.5, "forward"); //move forward 22 inches until in front of the left column
                    Thread.sleep(250);
                }
                else if (pictograph.equals("CENTER")) { //else if the pictograph 1says that the key column is the center column

                    encodersMove(bot, 28, 0.5, "forward"); //move forward 28 inches until in front of the center column
                    Thread.sleep(250);
                }
                else if (pictograph.equals("RIGHT")) { //else if the pictograph says that the key column is the right column

                    encodersMove(bot, 36, 0.5, "forward"); //move forward 36 inches until in front of the right column
                    Thread.sleep(250);
                }
                else if (pictograph.equals("UNKNOWN")) { //else if the pictograph cannot determine which column is the key column

                    encodersMove(bot, 28, 0.5, "forward"); //move forward 28 inches until in front of the center column (default)
                    Thread.sleep(250);
                }

                imuTurn(bot, 90, 0.5, "left"); //turn left 90 degrees
                Thread.sleep(250);
            }

            else if (alliancePlacement.equals("far")) { //if we are far from the audience

                encodersMove(bot, 20, 0.5, "forward"); //drive forward
                Thread.sleep(250);

                imuTurn(bot, 90, 0.5, "right"); //turn right 90 degrees
                Thread.sleep(250);

                if (pictograph.equals("LEFT")) { //if the pictograph says that the key column is the left column

                    encodersMove(bot, 1, 0.5, "forward"); //move forward 1 inch until in front of the left column
                    Thread.sleep(250);
                }
                else if (pictograph.equals("CENTER")) { //else if the pictograph says that the key column is the center column

                    encodersMove(bot, 5, 0.5, "forward"); //move forward 5 inches until in front of the center column
                    Thread.sleep(250);
                }
                else if (pictograph.equals("RIGHT")) { //else if the pictograph says that the key column is the right column

                    encodersMove(bot, 10, 0.5, "forward"); //move forward 10 inches until in front of the right column
                    Thread.sleep(250);
                }
                else if (pictograph.equals("UNKNOWN")) { //else if the pictograph cannot determine which column is the key column

                    encodersMove(bot, 5, 0.5, "forward"); //move forward 5 inches until in front of the center column (default)
                    Thread.sleep(250);
                }

                imuTurn(bot, 90, 0.5, "left"); //turn left 90 degrees
                Thread.sleep(250);
            }
        }

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

        while (bot.getHeading() < (degrees - 15) && opModeIsActive()) { //while the value of getHeading is
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

        telemetry.addData("Pictograph", pictograph);
        telemetry.update();
    }

    /**
     * This program lowers the servoJewel arm
     *
     * @param bot the robot currently being worked on
     * @throws InterruptedException
     */
    public void lowerArm(Robot bot) throws InterruptedException {

        double servoPosition = bot.getJewelServoPosition(); //sets getPosition() to servoPosition

        while (servoPosition < 0.90 && opModeIsActive()) { //while the op mode is active and while the servo position variable is less
            //than 0.98

            servoPosition = servoPosition + 0.05;          //add 0.05 to the current servoPosition variable
            bot.setJewelServoPosition(servoPosition);
            Thread.sleep(75);                              //wait 0.075 seconds (75 milliseconds)
        }

        Thread.sleep(250);
    }

    /**
     * This method places a glyph in the cryptobox. This is also steps 3-7
     * of the pseudocode for Version 2 of autonomous we developed on Dec. 6th, 2017.
     *
     * @param bot - the bot currently being worked on
     * @throws InterruptedException
     */
    public void placeGlyph(Robot bot) throws InterruptedException {

        encodersMove(bot, 1, 0.5, "forward"); //moves one inch forward
        Thread.sleep(1000);

        bot.glyphGrabberOpen(); //releases the glyph
        Thread.sleep(500);

        encodersMove(bot, 4, 0.5, "backward"); //moves four inches backward
        Thread.sleep(500);

        imuTurn(bot, 180, 0.5, "right"); //turns 180 degrees right
        Thread.sleep(500);

        encodersMove(bot, 7, 0.5, "backward"); //moves seven inches backward
        Thread.sleep(500);

        encodersMove(bot, 1, 0.5, "forward"); //moves one inch forward
        Thread.sleep(500);
    }

    /**
     * This method will help the drive team to align the robot prior to autonomous using the range sensor
     *
     * @param bot the bot currently being worked on
     */
    public void alignRobot(Robot bot) {

        gamepad1.reset();

        while (!prev_Y_ButtonState) {

            distanceFromWall = bot.getDistance();

            telemetry.addData("mr_Range ", "%.2f inches", distanceFromWall);

            if (distanceFromWall < 13.4) {

                telemetry.addLine("Move the robot farther away from the wall.");
            }
            else if (distanceFromWall >= 13.4 && distanceFromWall <= 14.5) {

                telemetry.addLine("The robot is good.");
            }
            else if (distanceFromWall > 14.5) {

                telemetry.addLine("Move the robot closer to the wall.");
            }
            else {

                telemetry.addLine("Please place the robot in front of the wall.");
            }

            cur_Y_ButtonState = gamepad1.y;                           // get the current state of button "y"

            if (cur_Y_ButtonState) {                                  // when the "y" button on the gamepad is pushed

                if (!prev_Y_ButtonState) {                            // when the previous "y" button was NOT pushed

                    prev_Y_ButtonState = true;                        // indicate that the previous y button state is PUSHED
                }
            }

            telemetry.update();
        }
    }

    /**
     * This method will run the movement code in JewelCloseBlue for use in IndieAutonomousOptions
     *
     * @param bot the bot currently being worked on
     * @throws InterruptedException
     */
    public void justParkCloseBlue(Robot bot) throws InterruptedException {

        encodersMove(bot, 32, 0.5, "forward");
        Thread.sleep(500);

        imuTurn(bot, 90, 0.5, "left");
        Thread.sleep(500);

        encodersMove(bot, 2, 0.5, "forward");
        Thread.sleep(500);
    }

    /**
     * This method will run the movement code in JewelFarBlue for use in IndieAutonomousOptions
     *
     * @param bot the bot currently being worked on
     * @throws InterruptedException
     */
    public void justParkFarBlue(Robot bot) throws InterruptedException {

        encodersMove(bot, 22, 0.5, "forward");
        Thread.sleep(500);

        encodersMove(bot, 18, 0.5, "right");
        Thread.sleep(500);
    }

    /**
     * This method will run the movement code in JewelCloseRed for use in IndieAutonomousOptions
     *
     * @param bot the bot currently being worked on
     * @throws InterruptedException
     */
    public void justParkCloseRed(Robot bot) throws InterruptedException {

        encodersMove(bot, 32, 0.5, "backward");
        Thread.sleep(500);

        imuTurn(bot, 90, 0.5, "left");
        Thread.sleep(500);

        encodersMove(bot, 2, 0.5, "forward");
        Thread.sleep(500);
    }

    /**
     * This method will run the movement code in JewelFarRed for use in IndieAutonomousOptions
     *
     * @param bot the bot currently being worked on
     * @throws InterruptedException
     */
    public void justParkFarRed(Robot bot) throws InterruptedException {

        encodersMove(bot, 22, 0.5, "backward");
        Thread.sleep(500);

        encodersMove(bot, 18, 0.5, "right");
        Thread.sleep(500);
    }
}