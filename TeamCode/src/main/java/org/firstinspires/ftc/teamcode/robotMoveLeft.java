package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;
/**
 * Created by Alex on 11/15/17.
 */

@Autonomous

public class robotMoveLeft extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);

        while(opModeIsActive()) {

            robot.setDriveMotorPower(1.0, -1.0, 1.0, -1.0);
        }
    }

}
