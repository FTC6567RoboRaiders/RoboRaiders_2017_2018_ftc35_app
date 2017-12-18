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

        lowerArm(robot, 0.99); //jewel is selected
        selectJewel(robot, "red");

        encodersMove(robot, 19, 0.5, "backward");
        Thread.sleep(500);

        encodersMove(robot, 20, 0.5, "right");
        Thread.sleep(500);

        imuTurn(robot, 183, 0.5, "right"); //robot turns so glyph collector faces cryptobox
        Thread.sleep(500);

        //placeGlyph(robot); //robot places glyph
        Thread.sleep(500);
    }
}