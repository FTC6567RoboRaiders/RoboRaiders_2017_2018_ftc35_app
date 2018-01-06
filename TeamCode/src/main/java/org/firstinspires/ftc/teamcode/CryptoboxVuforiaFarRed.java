package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

/**
 * Created by Katelin Zichittella on 12/6/17.
 */

@Autonomous
@Disabled

public class CryptoboxVuforiaFarRed extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        vuforiaInitialization(hardwareMap);
        robot.initializeServos();
        telemetry.addData("Initialized", true);
        telemetry.update();

        waitForStart();

        lowerArm(robot, 0.99);
        selectJewel(robot, "red");

        getRelicRecoveryVuMark();
        Thread.sleep(250);

        encodersMove(robot, 20, 0.5, "backward");
        Thread.sleep(250);

        imuTurn(robot, 90, 0.5, "left");
        Thread.sleep(250);

        encodersMove(robot, 5, 0.5, "backward");
        Thread.sleep(250);

        selectColumnDistanceSensor(robot, "red", pictograph);
    }
}