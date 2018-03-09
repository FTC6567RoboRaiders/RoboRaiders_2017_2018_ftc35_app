package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

/**
 * Created by Alex (and a little bit J-Dawg) on 11/8/17.
 */

@Autonomous
@Disabled

public class JewelArmTest extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
//        robot.initializeServos();
        robot.servoJewel.setPosition(0.4);
//        robot.servoElbow.setPosition(0.0);

        telemetry.addData("Initialized", true);
        telemetry.update();

        waitForStart();
        //  robot.liftGlyph();
        double servoJewelPosition = robot.getJewelServoPosition();
        telemetry.addData("Jewel Arm Pos: ", servoJewelPosition);
        telemetry.update();

        while (servoJewelPosition < 0.85 && opModeIsActive()) {
            servoJewelPosition = servoJewelPosition + 0.05;
            robot.setJewelServoPosition(servoJewelPosition);
            telemetry.addData("Jewel Arm Pos: ", servoJewelPosition);
            telemetry.update();
            Thread.sleep(75);
        }
        robot.servoJewel.setPosition(0.87);
        telemetry.addData("Jewel Arm Pos: ", servoJewelPosition);

        telemetry.update();
        Thread.sleep(1000);

    }
}