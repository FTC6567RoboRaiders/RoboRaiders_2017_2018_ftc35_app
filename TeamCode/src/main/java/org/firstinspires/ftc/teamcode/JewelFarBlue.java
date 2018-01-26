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

public class JewelFarBlue extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        robot.initializeServos();
        alignRobot(robot);
        telemetry.addData("Initialized", true);
        telemetry.update();

        waitForStart();
        robot.liftGlyph();

        lowerArm(robot);
        selectJewel(robot, "blue");

        encodersMove(robot, 22, 0.5, "forward");
        Thread.sleep(500);

        encodersMove(robot, 18, 0.5, "right");
        Thread.sleep(500);
    }
}