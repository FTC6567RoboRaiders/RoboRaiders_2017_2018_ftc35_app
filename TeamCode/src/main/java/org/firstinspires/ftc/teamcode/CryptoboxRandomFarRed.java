package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

/**
 * Created by Alex Snyder on 12/6/17.
 */

@Autonomous

public class CryptoboxRandomFarRed extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        robot.initializeServos();
        telemetry.addData("Initialized", true);
        telemetry.update();

        waitForStart();

        lowerArm(robot); //jewel is selected
        selectJewel(robot, "red");

        encodersMove(robot, 20, 0.5, "backward"); //do not drive as far backward
        Thread.sleep(500);

        encodersMove(robot, 16, 0.5, "right"); //robot strafes right until in front of cryptobox
        Thread.sleep(500);

        imuTurn(robot, 184, 0.5, "left"); //robot turns 180 degrees left
        Thread.sleep(500);

        placeGlyph(robot); //robot places glyph
        Thread.sleep(500);
    }
}