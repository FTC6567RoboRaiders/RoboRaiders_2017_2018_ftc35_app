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

        encodersMove(robot, 2, 0.5, "backward");
        Thread.sleep(250);

        getRelicRecoveryVuMark();
        Thread.sleep(250);

        telemetry.addData("Pictograph", pictograph);
        telemetry.update();

        if (red) { //if ball on the right is red

            encodersMove(robot, 22, 0.5, "backward"); //do not drive as far backward
            Thread.sleep(500);
        }
        else {

            encodersMove(robot, 23, 0.5, "backward"); //drive backward
            Thread.sleep(500);
        }

        imuTurn(robot, 90, 0.5, "left");
        Thread.sleep(250);

        selectColumn(robot, "red", pictograph); //select column based on encoder readouts

        imuTurn(robot, 182, 0.5, "left"); //robot turns 180 degrees left
        Thread.sleep(500);

        placeGlyph(robot); //robot places glyph in selected column
        Thread.sleep(500);

        //selectColumnDistanceSensor(robot, "red", pictograph); //this selects the key column using the distance sensor
    }
}