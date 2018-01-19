/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * {@link HubBotRangeSensorTest} illustrates how to use the Modern Robotics
 * Range Sensor.
 * <p>
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://modernroboticsinc.com/range-sensor">MR Range Sensor</a>
 */
@Autonomous(name = "Sensor: MR/REV Range and Distance", group = "Sensor")

public class HubBotRangeSensorTest extends LinearOpMode {

    ModernRoboticsI2cRangeSensor mrRange;
    ModernRoboticsI2cRangeSensor mrRange2;
    DistanceSensor revDistance;

    boolean cur_Y_ButtonState;                                            // "b" button current state

    boolean prev_Y_ButtonState;

    double distanceFromWall;


    @Override
    public void runOpMode() {

        gamepad1.reset();

        prev_Y_ButtonState = false;

        cur_Y_ButtonState = false;

        distanceFromWall = 0;


        mrRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "mr_range");
        mrRange2 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "mr_range2");
        revDistance = hardwareMap.get(DistanceSensor.class, "rev_distance");


        telemetry.addData("Initialized", true);
        telemetry.update();

        while (!prev_Y_ButtonState) {

            distanceFromWall = mrRange.getDistance(DistanceUnit.INCH);

            if (distanceFromWall < 12) {

                telemetry.addLine("Move the robot farther away from the wall.");

            }

            else if (distanceFromWall >= 12 && distanceFromWall <= 16) {

                telemetry.addLine("The robot is good.");
            }

            else if (distanceFromWall > 16) {

                telemetry.addLine("Move the robot closer to the wall");
            }

            else {

                telemetry.addLine("Please place the robot in front of the wall.");

            }

            telemetry.addData("mr_Range ", "%.2f inches", distanceFromWall);
            telemetry.update();
            cur_Y_ButtonState = gamepad1.y;                           // get the current state of button y

            if (cur_Y_ButtonState) {                                  // when the "y" button on the gamepad is pressed

                if (!prev_Y_ButtonState) {                            // when the previous "y" button was NOT pushed

                    prev_Y_ButtonState = true;                        // indicate that the previous y button state is PUSHED
                }
            }
        }

        // wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("mr_Range cm", "%.2f cm", distanceFromWall);
            telemetry.addData("mr_Range2 cm", "%.2f cm", distanceFromWall);
            telemetry.update();

        }
    }
}
