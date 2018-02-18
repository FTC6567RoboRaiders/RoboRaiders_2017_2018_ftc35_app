package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

/**
 * Created by Alex on 2/18/2018
 */

@Autonomous


public class JewelArmTest extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        robot.initializeServos();
        alignRobot(robot, "blue", "close");
        telemetry.addData("Initialized", true);
        telemetry.update();

        waitForStart();
        robot.liftGlyph();

        lowerArm(robot);
        selectJewel(robot, "blue");

        encodersMove(robot, 32, 0.5, "forward");
        Thread.sleep(500);

        imuTurn(robot, 90, 0.5, "left");
        Thread.sleep(500);

        encodersMove(robot, 2, 0.5, "forward");
        Thread.sleep(500);
    }
}
