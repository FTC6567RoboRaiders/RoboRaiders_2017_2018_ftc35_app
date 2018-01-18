package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Steve Kocik on 1/14/2018.
 */

@TeleOp(name = "HubBotRelicTest", group = "Test Bots")

public class HubBotRelicTest extends OpMode {

    Servo relicPincher = null;


    public boolean currStateA = false;   // A is open
    public boolean prevStateA = false;
    public boolean currStateB = false;   // B is close
    public boolean prevStateB = false;

    @Override
    public void init() {

        relicPincher = hardwareMap.get(Servo.class, "relicPincher");

    }

    @Override
    public void loop() {

        currStateA = false;
        prevStateA = false;
        currStateB = false;
        prevStateB = false;


        currStateA = gamepad1.a;
        if (currStateA && currStateA != prevStateA) {

            relicPincher.setPosition(0.75);
            prevStateA = currStateA;
        } else if (!currStateB && currStateB != prevStateB) {

            prevStateB = currStateB;
        }

        // close the lefty and righty
        currStateB = gamepad1.b;
        if (currStateB && currStateB != prevStateB) {

            relicPincher.setPosition(0.0);
            prevStateB = currStateB;
        } else if (!currStateB && currStateB != prevStateB) {

            prevStateB = currStateB;
        }
    }
}