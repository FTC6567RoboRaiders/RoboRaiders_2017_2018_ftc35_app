package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

/**
 * Created by Katelin Zichittella on 12/6/17.
 */

@Autonomous
//@Disabled

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

        lowerArm(robot);
        selectJewel(robot, "blue");

        getRelicRecoveryVuMark();
        Thread.sleep(250);

        //selectColumn(robot, "blue", "close", pictograph);

        /*encodersMove(robot, 15, 0.5, "forward");
        Thread.sleep(250);

        selectColumnDistanceSensor(robot, "blue", pictograph);*/
    }
}