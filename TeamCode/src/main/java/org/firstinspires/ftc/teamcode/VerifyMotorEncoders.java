package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

@Autonomous(name = "Verify the Motor Encoders", group = "Verify")

/**
 * This class will verify that the encoders are working properly by running with encoders
 * and then output the encoder counts to the phone and log.
 *
 * Created by SteveKocik on 11/20/2017.
 */

public class VerifyMotorEncoders extends RoboRaidersAuto {

    // The following variables are used to control how often telemetry data is written to the log
    //
    //  - currentTimeStamp - is the current time stamp, this is updated every time the loop() method is
    //                     called
    //
    //  - pastTimeStamp    - is the time stamp that the log was last updated, initially it is set to 0,
    //                     and is only updated when the log is updated
    //
    //  - LOG_INTERVAL     - the amount of time per each log updated, initially set to 1/4 of a
    //                     second, this value is in milliseconds (1/2 of sec = 500 milliseconds)

    private long currentTimeStamp;
    private long pastTimeStamp;
    private static final long LOG_INTERVAL = 500;
    private boolean itsTimeToLog;
    private int[] encoderArray = new int[4];


    public Robot robot = new Robot();


    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);

        // Write message to log indicating that teleop program is initialized
        Log.d("INIT","VerifyMotorEncoders Initialization Complete");

        telemetry.addData("Initialized", true);
        telemetry.update();

        robot.resetEncoders();                                // resets encoders
        robot.runWithEncoders();                              // set motors to RUN_WITH_ENCODERS

        waitForStart();

        while(opModeIsActive()) {

            currentTimeStamp = System.currentTimeMillis();   //* get the current time stamp


            robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);    //* run the motors at 1/2 speed, don't need fast and furious part 2


            // store the current encoder counts (positions) for the drive motors
            encoderArray[0] = robot.motorFrontLeft.getCurrentPosition();
            encoderArray[1] = robot.motorFrontRight.getCurrentPosition();
            encoderArray[2] = robot.motorBackLeft.getCurrentPosition();
            encoderArray[3] = robot.motorBackRight.getCurrentPosition();

            // The method timeToLog() will determine if the logging interval has expired from the
            // last time
            itsTimeToLog = timeToLog();

            // Log the encoder count for each of the motors
            if( itsTimeToLog ) {
                Log.d("ECD","********************************************************");
                Log.d("ECD","Start of Encoder Counts for Drive Motors");
                Log.d("ECD",String.format("motorFrontLeft:  %s", encoderArray[0]));
                Log.d("ECD",String.format("motorFrontRight: %s", encoderArray[1]));
                Log.d("ECD",String.format("motorBackLeft:   %s", encoderArray[2]));
                Log.d("ECD",String.format("motorBackRight:  %s", encoderArray[3]));
                Log.d("ECD","End of Encoder Counts for Drive Motors");
                Log.d("ECD","********************************************************");

                telemetry.addLine().addData("motorFrontLeft:  ", encoderArray[0]);
                telemetry.addLine().addData("motorFrontRight: ", encoderArray[1]);
                telemetry.addLine().addData("motorBackLeft:   ", encoderArray[2]);
                telemetry.addLine().addData("motorBackRight:  ", encoderArray[3]);
                telemetry.update();
            }
        }

        robot.resetEncoders();                                // stop and reset the encoders
    }

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

    }


}

