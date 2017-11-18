package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

/**
 * Created by Katelin Zichittella on 11/15/17.
 */

@Autonomous

public class CryptoboxCloseBlue extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        vuforiaInitialization(hardwareMap);

        robot.setServoPosition(0.1);

        waitForStart();

        lowerArm(robot, 0.99);
        selectJewel(robot, "blue");

        encodersMove(robot, 24, 0.5, "forward");
        Thread.sleep(500);

        imuTurn(robot, 90, 0.5, "left");
        Thread.sleep(500);

        encodersMove(robot, 3, 0.5, "forward");
        Thread.sleep(500);

        distanceSensorCount(robot, 2, 0.3, 18);
        Thread.sleep(500);

        encodersMove(robot, 1, 0.3, "right");
        Thread.sleep(500);
    }
}