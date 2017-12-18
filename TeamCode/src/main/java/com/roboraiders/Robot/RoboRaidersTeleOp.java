package com.roboraiders.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Katelin Zichittella on 12/18/2017.
 */

public abstract class RoboRaidersTeleOp extends OpMode {

    /**
     * This method will raise a glyph in the omni wheel assembly
     */
    public void glyphUp(Robot bot) {

        bot.resetEncoders();
        bot.runWithEncoders();

        double COUNTS = bot.calculateCOUNTS(8);

        bot.motorGlyphUp.setPower(0.5);

        while (bot.getGlyphUpEncoderCount() < COUNTS) {

        }

        bot.motorGlyphUp.setPower(0.0);

        bot.runWithoutEncoders();
    }

    /**
     * This method will lower a glyph in the omni wheel assembly
     */
    public void glyphDown(Robot bot) {

        bot.resetEncoders();
        bot.runWithEncoders();

        double COUNTS = bot.calculateCOUNTS(8);

        bot.motorGlyphUp.setPower(-0.5);

        while (bot.getGlyphUpEncoderCount() < COUNTS) {

        }

        bot.motorGlyphUp.setPower(0.0);

        bot.runWithoutEncoders();
    }

    /**
     * This method lowers and expels a glyph in autonomous
     *
     * @throws InterruptedException
     */
    public void expelGlyph(Robot bot) throws InterruptedException {

        glyphDown(bot);
        Thread.sleep(500);

        bot.resetEncoders();
        bot.runWithEncoders();

        double COUNTS = bot.calculateCOUNTS(4);

        bot.motorGlyphInLeft.setPower(-0.5);
        bot.motorGlyphInRight.setPower(-0.5);

        while (bot.getGlyphInEncoderCount() < COUNTS) {

        }

        bot.motorGlyphInLeft.setPower(0.0);
        bot.motorGlyphInRight.setPower(0.0);

        bot.runWithoutEncoders();
    }

    public void encodersRelicOut(Robot bot, int distance, double power) {

        bot.resetEncoders();
        bot.runWithEncoders();

        bot.setRelicMotorPower(power);

        double COUNTS = bot.calculateCOUNTS(distance);

        while (bot.motorRelic.getCurrentPosition() < COUNTS) {

            telemetry.addData("Counts", COUNTS);
            telemetry.addData("Encoder Count", bot.motorRelic.getCurrentPosition());
            telemetry.update();
        }
    }

    public void encodersRelicIn(Robot bot, int distance, double power) {

        bot.resetEncoders();
        bot.runWithEncoders();

        bot.setRelicMotorPower(-power);

        double COUNTS = bot.calculateCOUNTS(distance);

        while (bot.motorRelic.getCurrentPosition() < COUNTS) {

            telemetry.addData("Counts", COUNTS);
            telemetry.addData("Encoder Count", bot.motorRelic.getCurrentPosition());
            telemetry.update();
        }
    }
}
