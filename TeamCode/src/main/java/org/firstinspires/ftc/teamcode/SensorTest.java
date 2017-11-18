package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Nick on 10/8/17.
 */

@Autonomous

public class SensorTest extends RoboRaidersAuto {

    public Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        vuforiaInitialization(hardwareMap);
        //robot.servoJewel.setPosition(1.0); //lower arm with color sensor
        telemetry.addData("Initialized", true);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            /*robot.getColorIntensity("red");
            robot.getColorIntensity("blue");
            robot.distanceSensor.getDistance(DistanceUnit.CM);
            robot.digitalTouch.getState();
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            getRelicRecoveryVuMark()*/

            telemetry.addData("Red", robot.getColorIntensity("red"));
            telemetry.addData("Blue", robot.getColorIntensity("blue"));
            telemetry.addData("Red Direct", robot.colorSensor.red());
            telemetry.addData("Blue Direct", robot.colorSensor.blue());
            telemetry.addData("Distance", robot.distanceSensor.getDistance(DistanceUnit.CM));
            /*telemetry.addData("Pictograph", pictograph);
            telemetry.addData("IMU Angle", robot.angles.firstAngle);
            telemetry.addData("Touch", robot.digitalTouch.getState());*/
            telemetry.update();

            /*imuTurnLeft(robot, 90, 0.5);
            Thread.sleep(1000);
            imuTurnRight(robot, 90, 0.5);
            Thread.sleep(1000);

            encodersStrafe(robot, 30, 0.25, "left");
            Thread.sleep(1000);
            encodersStrafe(robot, 30, 0.25, "right");
            Thread.sleep(1000);*/

            //touchSensorCount(robot, 2, 0.2);
            //distanceSensorCount(robot, 3, 0.3);
        }
    }
}