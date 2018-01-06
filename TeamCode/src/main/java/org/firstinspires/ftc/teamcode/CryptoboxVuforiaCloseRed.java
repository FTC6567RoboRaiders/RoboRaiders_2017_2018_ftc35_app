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

public class CryptoboxVuforiaCloseRed extends RoboRaidersAuto {

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

        //selectColumn(robot, "red", "close", pictograph);  //move to key column using encoders

        selectColumnDistanceSensor(robot, "red", pictograph); //this selects the column using the distance sensor

        imuTurn(robot, 95, 0.5, "left"); //robot turns so glyph collector faces cryptobox
        Thread.sleep(500);

        placeGlyph(robot); //robot places glyph in the key column
        Thread.sleep(500);
    }
}