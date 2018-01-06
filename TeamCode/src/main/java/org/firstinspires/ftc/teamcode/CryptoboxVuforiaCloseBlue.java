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

public class CryptoboxVuforiaCloseBlue extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        vuforiaInitialization(hardwareMap);
        robot.initializeServos();
        telemetry.addData("Initialized", true);
        telemetry.update();

        waitForStart();

        lowerArm(robot, 0.99); //jewel is selected
        selectJewel(robot, "blue");

        getRelicRecoveryVuMark(); //read the pictograph
        Thread.sleep(250);

        telemetry.addData("Pictograph", pictograph); //store the pictograph in memory
        telemetry.update();

        //selectColumn(robot, "blue", "close", pictograph);  //move to key column using encoders

        selectColumnDistanceSensor(robot, "blue", pictograph); //this selects the key column using a distance sensor

        imuTurn(robot, 95, 0.5, "left"); //robot turns so glyph collector faces cryptobox
        Thread.sleep(500);

        placeGlyph(robot); //robot places glyph in key column
        Thread.sleep(500);
    }
}