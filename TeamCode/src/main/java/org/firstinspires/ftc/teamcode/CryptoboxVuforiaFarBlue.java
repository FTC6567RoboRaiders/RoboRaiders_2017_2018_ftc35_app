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

public class CryptoboxVuforiaFarBlue extends RoboRaidersAuto {

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
        selectJewel(robot, "blue");

        encodersMove(robot, 2, 0.5, "forward");
        Thread.sleep(250);

        getRelicRecoveryVuMark();
        Thread.sleep(250);

        telemetry.addData("Pictograph", pictograph);
        telemetry.update();

        if (blue) { //if ball on the right is blue

            encodersMove(robot, 23, 0.5, "forward"); //drive forward
            Thread.sleep(500);
        }
        else {

            encodersMove(robot, 22, 0.5, "forward"); //do not drive as far forward
            Thread.sleep(500);
        }

        imuTurn(robot, 90, 0.5, "right");
        Thread.sleep(250);

        selectColumn(robot, "blue", "far", pictograph);

        placeGlyph(robot);
        Thread.sleep(500);

        //selectColumnDistanceSensor(robot, "blue", pictograph); //select column using the distance sensor
    }
}