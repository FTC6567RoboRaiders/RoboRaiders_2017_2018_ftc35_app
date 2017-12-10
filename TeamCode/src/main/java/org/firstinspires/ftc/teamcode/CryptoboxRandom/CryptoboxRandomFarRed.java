package org.firstinspires.ftc.teamcode.CryptoboxRandom;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

/**
 * Created by Alex Snyder on 12/6/17.
 */

@Autonomous
//@Disabled

public class CryptoboxRandomFarRed extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        robot.initializeServos();
        vuforiaInitialization(hardwareMap);
        telemetry.addData("Initialized", true);
        telemetry.update();

        waitForStart();

        lowerArm(robot, 0.99);
        selectJewel(robot, "red");

        encodersMove(robot, 16, 0.5, "backward");
        Thread.sleep(500);

        encodersMove(robot, 12, 0.5, "right");
        Thread.sleep(500);

        imuTurn(robot, 180, 0.5, "right");
        Thread.sleep(500);
    }
}