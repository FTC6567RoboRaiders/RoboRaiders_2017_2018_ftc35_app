package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

/**
 * Created by Alex (and a little bit J-Dawg) on 11/8/17.
 */

@Autonomous


public class JewelArmTest2 extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
//        robot.initializeServos();
        robot.servoJewel.setPosition(0.4);
        robot.servoElbow.setPosition(0.0);

        telemetry.addData("Initialized", true);
        telemetry.update();

        waitForStart();

        double servoJewelPosition = robot.getJewelServoPosition();

        while (servoJewelPosition < 0.85 && opModeIsActive()) {
            telemetry.addData("Jewel Arm Pos: ", servoJewelPosition);

            if (servoJewelPosition == 0.65) {
                robot.servoElbow.setPosition(0.87);
                Thread.sleep(500);
            }

            servoJewelPosition = servoJewelPosition + 0.05;
            robot.setJewelServoPosition(servoJewelPosition);
            telemetry.update();
            Thread.sleep(75);
        }

        robot.servoJewel.setPosition(0.87);
        telemetry.addData("Jewel Arm Pos: ", servoJewelPosition);


        Thread.sleep(1000);

    }
}