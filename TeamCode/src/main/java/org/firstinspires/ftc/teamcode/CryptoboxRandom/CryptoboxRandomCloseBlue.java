package org.firstinspires.ftc.teamcode.CryptoboxRandom;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

/**
 * Created by Alex Snyder on 11/15/17.
 */

@Autonomous

public class CryptoboxRandomCloseBlue extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        robot.initializeServos();
        vuforiaInitialization(hardwareMap);
        telemetry.addData("Initialized", true);
        telemetry.update();

        waitForStart();

        lowerArm(robot, 0.99); //jewel is selected
        selectJewel(robot, "blue");

        encodersMove(robot, 32, 0.5, "forward");
        Thread.sleep(500);

        imuTurn(robot, 90, 0.5, "left"); //robot turns so glyoh collector is facing cryptobox
        Thread.sleep(500);

        placeGlyph(robot); //robot places glyph
        Thread.sleep(500);

        encodersMove(robot, 1, 0.3, "right");
        Thread.sleep(500);

    }
}