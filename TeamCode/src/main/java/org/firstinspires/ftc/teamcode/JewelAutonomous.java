package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

/**
 * Created by (mostly) Nick Urbin and (a little bit) Kevin McCrudden on 10/1/17.
 */

@Autonomous
@Disabled

public class JewelAutonomous extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        robot.initializeServos();
        vuforiaInitialization(hardwareMap);

        waitForStart();

        lowerArm(robot, 0.99);
        selectJewel(robot, "blue");
    }
}


