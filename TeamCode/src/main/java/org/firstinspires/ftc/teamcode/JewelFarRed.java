package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

/**
 * Created by Alex (and a little bit J-Dawg) on 11/8/17.
 */

@Autonomous

public class JewelFarRed extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        vuforiaInitialization(hardwareMap);

        robot.setServoPosition(0.1);

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