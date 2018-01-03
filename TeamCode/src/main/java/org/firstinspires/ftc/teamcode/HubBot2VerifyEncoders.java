package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

@Autonomous(name = "HubBot2 Verify the Motor Encoders", group = "Verify")
@Disabled


/**
 * This class will verify that the drive motor encoders are working properly by running with encoders
 * and then outputting the encoder counts to the phone and log.
 *
 * made some updates so Alex can verify her android studio installation is right....
 *
 * Created by SteveKocik on 11/20/2017.
 */

public class HubBot2VerifyEncoders extends LinearOpMode {

    // The following variables are used to control how often telemetry data is written to the log
    //
    //  - currentTimeStamp - is the current time stamp, this is updated every time within the
    //                     while(opModeIsActive()) loop
    //
    //  - pastTimeStamp    - is the time stamp that the log was last updated, initially it is set to 0,
    //                     and is only updated when the log is updated
    //
    //  - LOG_INTERVAL     - the amount of time per each log updated, initially set to 1/2 of a
    //                     second, this value is in milliseconds (1/2 of sec = 500 milliseconds)

    public DcMotor motorBackRight = null;

    private long currentTimeStamp;
    private long pastTimeStamp;
    private static final long LOG_INTERVAL = 500;
    private boolean itsTimeToLog;
    int encoderCount;



    @Override
    public void runOpMode() throws InterruptedException {

        motorBackRight = hardwareMap.get(DcMotor.class, "right_Back");

        telemetry.addLine().addData("Initialized:", false);
        telemetry.addLine().addData("motorBackRight:  ", encoderCount);
        telemetry.addLine().addData("EncMode: ", motorBackRight.getMode());
        telemetry.addLine().addData("isPID: ",motorBackRight.getMode().isPIDMode());
        telemetry.addLine().addData("PortNumber: ",motorBackRight.getPortNumber());
        telemetry.addLine().addData("Power: ",motorBackRight.getPower());

        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setPower(0.0);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pastTimeStamp = 0;

        telemetry.addLine().addData("Initialized", true);
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            currentTimeStamp = System.currentTimeMillis();   // get the current time stamp

            motorBackRight.setPower(0.5);
            // store the current encoder counts (positions) for the drive motors
            encoderCount = motorBackRight.getCurrentPosition();

            // The method timeToLog() will determine if the logging interval has expired from the
            // last time
            itsTimeToLog = timeToLog();

            // Log the encoder count for each of the motors
            if( itsTimeToLog ) {

                // Update the driver station display with the same information as has been
                // captured to the log file.

                telemetry.addLine().addData("motorBackRight:  ", encoderCount);
                telemetry.addLine().addData("EncMode: ", motorBackRight.getMode());
                telemetry.addLine().addData("isPID: ",motorBackRight.getMode().isPIDMode());
                telemetry.addLine().addData("PortNumber: ",motorBackRight.getPortNumber());
                telemetry.addLine().addData("Power: ",motorBackRight.getPower());
                telemetry.update();

            } // if( itsTimeToLog )

        } // while(opModeIsActive())

        motorBackRight.setPower(0);                 // stop the robot
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);      // stop and reset the encoders

    } // public void runOpMode() throws InterruptedException

    /**
     * Will determine when the log should be updated with new data.  The previous time is subtracted
     * from the current time with a result of a time change or delta.  The time delta is then compared
     * to the log interval (LOG_INTERVAL) which represents the number of seconds (or fractions of a
     * second) that should expire before updating the log.  If the delta time is greater than the
     * log interval, this method will return a true.  If the delta time is less than the log interval,
     * this method will return a false.
     *
     * Under the covers, this method will set the variable pastTimeStamp, when the log interval time
     * has expired.
     *
     * @return boolean - TRUE, interval has expired and caller should write to log
     *                 - FALSE, interval has not expired and caller should not write to log
     */

    private boolean timeToLog() {

        if ( (currentTimeStamp-pastTimeStamp) > LOG_INTERVAL ) {
            pastTimeStamp = currentTimeStamp;
            return true;
        }
        else {
            return false;
        }

    } // private boolean timeToLog()


}

