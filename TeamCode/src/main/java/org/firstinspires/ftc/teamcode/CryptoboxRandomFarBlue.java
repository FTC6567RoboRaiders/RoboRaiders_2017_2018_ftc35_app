package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

/**
 * Created by Alex Snyder on 12/6/17.
 */

@Autonomous

public class CryptoboxRandomFarBlue extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        robot.initializeServos();
        telemetry.addData("Initialized", true);
        telemetry.update();

        waitForStart();

        lowerArm(robot, 0.99); //jewel is selected
        selectJewel(robot, "blue");

        if (blue) { //if ball on the right is blue

            encodersMove(robot, 23, 0.5, "forward"); //drive forward
            Thread.sleep(500);
        }
        else {

            encodersMove(robot, 22, 0.5, "forward"); //do not drive as far forward
            Thread.sleep(500);
        }

        encodersMove(robot, 18, 0.5, "right"); //robot strafes right until in front of cryptobox
        Thread.sleep(500);

        placeGlyph(robot); //robot places glyph
        Thread.sleep(500);
    }
}