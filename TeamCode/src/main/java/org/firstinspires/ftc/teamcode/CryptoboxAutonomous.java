package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Alex Snyder and Fiona Beer on 10/1/2017.
 */

@Autonomous
//@Disabled

public class CryptoboxAutonomous extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap); //sets parameters for initialize sequence
        vuforiaInitialization(hardwareMap);

        waitForStart();

        /*robot.setDriveMotorPower(1, 1, 1, 1); //move forward for two seconds
        Thread.sleep(2000);
        robot.setDriveMotorPower(0, 0, 0, 0);
        Thread.sleep(500);

        robot.setDriveMotorPower(1, -1, -1, 1); //robot strafes right
        Thread.sleep(1000);
        robot.setDriveMotorPower(0, 0, 0, 0);
        Thread.sleep(500);*/

        //touchSensorCount(robot, 2, 0.2);
        distanceSensorCount(robot, 2, 0.4, 16);
        Thread.sleep(2000);

        encodersMove(robot, 2, 0.4, "right");
        Thread.sleep(500);
    }
}
