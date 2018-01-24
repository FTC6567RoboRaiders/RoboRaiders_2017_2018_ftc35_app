package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

/**
 * Created by Alex Snyder on 12/6/17.
 */

@Autonomous

public class CryptoboxRandomCloseRed extends RoboRaidersAuto {

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

        encodersMove(robot, 28, 0.5, "backward"); //drive backward
        Thread.sleep(500);

        imuTurn(robot, 90, 0.5, "left"); //robot turns so glyph collector faces cryptobox
        Thread.sleep(500);

        placeGlyph(robot); //robot places glyph`
        Thread.sleep(500);
    }
}