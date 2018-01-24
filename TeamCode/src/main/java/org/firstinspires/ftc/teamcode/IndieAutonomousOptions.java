package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

@Autonomous

public class IndieAutonomousOptions extends RoboRaidersAuto {

    public Robot robot = new Robot();

    boolean cur_B_ButtonState;                                            // "b" button current state
    boolean cur_X_ButtonState;                                            // "x" button current state
    boolean cur_A_ButtonState;                                            // "a" button current state
    boolean cur_Y_ButtonState;                                            // "y" button current state

    boolean prev_B_ButtonState;                                           // "b" button previous state
    boolean prev_X_ButtonState;                                           // "x" button previous state
    boolean prev_A_ButtonState;                                           // "a" button previous state
    boolean prev_Y_ButtonState;                                           // "y" button previous state

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        // Array to store selected options, by design selectedOptions[0][*] is used to store the
        // response for finalizing the selected options.
        String[][] selectedOptions = new String[5][2];                    // Array to store selections

        String allianceSelPrompt = "Alliance Color";                      // Alliance Color prompt
        final String[] alliancePosResps = new String[]                    // Possible Alliance Color selections
                {"red", "blue"};

        String bsSelPrompt = "Balancing Stone Location";                  // Balancing Stone Location prompt
        final String[] bsPosResps = new String[]                          // Possible Balancing Stone Location selections
                {"close", "far"};

        String jewelSelPrompt = "Jewel";                                  // Jewel prompt
        final String[] jewelPosResps = new String[]                       // Possible Jewel selections
                {"no", "yes"};

        String parkCryptSelPrompt = "Parking and/or Cryptobox";           // Parking and/or Cryptobox prompt
        final String[] parkCryptPosResps = new String[]                   // Possible Parking and/or Cryptobox selections
                {"Stay on the Balancing Stone", "Just Park", "Crytobox Random", "Cryptobox Vuforia"};

        String selectionsOk = "Selections Great :)";                      // Finished Selections prompt
        final String[] finishedSelPosResps = new String[]                 // Possible Finished Selections responses
                {"no", "yes"};

        // Default that the options to be selected are not finalized.  This is done since we want
        // the loop below to actually loop.  At the end of the loop, the question will be asked if
        // the selections are ok, if they are, then this will be set to "yes" and the loop will
        // exit.
        selectedOptions[0][1] = "no";

        // Configure for Indie autonomous while the selections are not finalized
        while (selectedOptions[0][1].equals("no")) {

            //                        Prompt            Responses    Index  Options output
            configForAuto2Options(allianceSelPrompt, alliancePosResps, 1, selectedOptions);     // Alliance Color selection
            configForAuto2Options(bsSelPrompt, bsPosResps, 2, selectedOptions);                 // Balancing Stone Location selection
            configForAuto2Options(jewelSelPrompt, jewelPosResps, 3, selectedOptions);           // Parking and/or Cryptobox selection
            configForAuto4Options(parkCryptSelPrompt, parkCryptPosResps, 4, selectedOptions);   // Park selection

            // Loop through all of the selections and tell user what s/he has selected
            for (int i = 1; i <= 4; i++) {

                telemetry.addLine().addData(selectedOptions[i][0], selectedOptions[i][1]);
            }

            // Are you sure about the options selected?
            //                        Prompt       Responses       Index Options output
            configForAuto2Options(selectionsOk, finishedSelPosResps, 0, selectedOptions);       // Finished Selections
        }

        //PUT RANGE SENSOR STUFF HERE

        telemetry.addLine("Options Selected - Press B to Initialize and Wait for Start");       // Tell user that options have been selected

        gamepad1.reset();

        while (!gamepad1.b) {

        }

        robot.initialize(hardwareMap);
        robot.initializeServos();
        vuforiaInitialization(hardwareMap);
        telemetry.addLine("Initialized");
        telemetry.update();

        // Wait for start to be pushed
        waitForStart();

        // Jewel
        if (selectedOptions[3][1].equals("no")) {

        }
        else if (selectedOptions[3][1].equals("yes")) {

            lowerArm(robot);
            selectJewel(robot, selectedOptions[1][1]);
        }

        // Parking and/or Cryptobox
        if (selectedOptions[4][1].equals("Stay on the Balancing Stone")) {

        }
        else if (selectedOptions[4][1].equals("Just Park")) {

            if (selectedOptions[1][1].equals("blue") && selectedOptions[2][1].equals("close")) {

                encodersMove(robot, 32, 0.5, "forward");
                Thread.sleep(500);

                imuTurn(robot, 90, 0.5, "left");
                Thread.sleep(500);

                encodersMove(robot, 2, 0.5, "forward");
                Thread.sleep(500);
            }
            else if (selectedOptions[1][1].equals("blue") && selectedOptions[2][1].equals("far")) {

                encodersMove(robot, 22, 0.5, "forward");
                Thread.sleep(500);

                encodersMove(robot, 18, 0.5, "right");
                Thread.sleep(500);
            }
            else if (selectedOptions[1][1].equals("red") && selectedOptions[2][1].equals("close")) {

                encodersMove(robot, 32, 0.5, "backward");
                Thread.sleep(500);

                imuTurn(robot, 90, 0.5, "left");
                Thread.sleep(500);

                encodersMove(robot, 2, 0.5, "forward");
                Thread.sleep(500);
            }
            else if (selectedOptions[1][1].equals("red") && selectedOptions[2][1].equals("far")) {

                encodersMove(robot, 22, 0.5, "backward");
                Thread.sleep(500);

                encodersMove(robot, 18, 0.5, "right");
                Thread.sleep(500);
            }
        }
        else if (selectedOptions[4][1].equals("Cryptobox Random")) {

            if (selectedOptions[1][1].equals("blue") && selectedOptions[2][1].equals("close")) {

                encodersMove(robot, 28, 0.5, "forward"); //drive forward
                Thread.sleep(500);

                imuTurn(robot, 90, 0.5, "left"); //robot turns so glyph collector faces cryptobox
                Thread.sleep(500);

                placeGlyph(robot); //robot places glyph`
                Thread.sleep(500);
            }
            else if (selectedOptions[1][1].equals("blue") && selectedOptions[2][1].equals("far")) {

                encodersMove(robot, 20, 0.5, "forward"); //drive forward
                Thread.sleep(500);

                imuTurn(robot, 90, 0.5, "right"); //turn right 90 degrees
                Thread.sleep(250);

                encodersMove(robot, 5, 0.5, "forward"); //move forward 5 inches until in front of the center column
                Thread.sleep(250);

                imuTurn(robot, 90, 0.5, "left"); //turn left 90 degrees
                Thread.sleep(250);

                placeGlyph(robot); //robot places glyph
                Thread.sleep(500);
            }
            else if (selectedOptions[1][1].equals("red") && selectedOptions[2][1].equals("close")) {

                encodersMove(robot, 28, 0.5, "backward"); //drive backward
                Thread.sleep(500);

                imuTurn(robot, 90, 0.5, "left"); //robot turns so glyph collector faces cryptobox
                Thread.sleep(500);

                placeGlyph(robot); //robot places glyph`
                Thread.sleep(500);
            }
            else if (selectedOptions[1][1].equals("red") && selectedOptions[2][1].equals("far")) {

                encodersMove(robot, 20, 0.5, "backward"); //drive backward
                Thread.sleep(500);

                imuTurn(robot, 90, 0.5, "right"); //turn right 90 degrees
                Thread.sleep(250);

                encodersMove(robot, 8, 0.5, "forward"); //move forward 8 inches until in front of the center column
                Thread.sleep(250);

                imuTurn(robot, 90, 0.5, "right"); //turn right 90 degrees
                Thread.sleep(250);

                placeGlyph(robot); //robot places glyph
                Thread.sleep(500);
            }
        }
        else if (selectedOptions[4][1].equals("Cryptobox Vuforia")) {

            getRelicRecoveryVuMark();
            Thread.sleep(250);

            selectColumn(robot, selectedOptions[1][1], selectedOptions[2][1], pictograph);
        }
    }

    /**
     * configForAuto2Optiona will save the response (selOptions) from a set of 2 possible responses (posResps)
     * a given prompt (selPrompt)
     *
     * @param selPrompt The given configuration prompt
     * @param posResps The possible responses to a given configuration prompt
     * @param selIndex The position within selOptions array in which to store the responses
     * @param selOptions Where to store the configuration prompt and response
     */
    public void configForAuto2Options(String selPrompt, String[] posResps, int selIndex, String[][] selOptions) {

        // Let the user Select
        gamepad1.reset();

        prev_B_ButtonState = false;
        prev_X_ButtonState = false;
        cur_B_ButtonState = false;
        cur_X_ButtonState = false;

        selOptions[selIndex][0] = selPrompt;

        // Prompt User for Selection
        telemetry.addLine(selPrompt);
        telemetry.addLine("Press B for " + posResps[0] + " or X for " + posResps[1]);
        telemetry.update();

        // Loop until either the "b" button or the "x" button is pressed, initially we set both
        // buttons to indicate they have not been pressed.
        //
        // The logic here says OR the previous button states and when they are both false continue
        // here is a table of how this works
        //    +--------------------+------+--------------------+--------+-------------+
        //    | prev_B_ButtonState | -OR- | prev_X_ButtonState | Result | Neg. Result |
        //    +--------------------+------+--------------------+--------+-------------+
        //    |      FALSE         | -OR- |     FALSE          | FALSE  |   TRUE      |
        //    +--------------------+------+--------------------+--------+-------------+
        //    |      FALSE         | -OR- |     TRUE           | TRUE   |   FALSE     |
        //    +--------------------+------+--------------------+--------+-------------+
        //    |      TRUE          | -OR- |     FALSE          | TRUE   |   FALSE     |
        //    +--------------------+------+--------------------+--------+-------------+
        //    |      TRUE          | -OR- |     TRUE           | TRUE   |   FALSE     |
        //    +--------------------+------+--------------------+--------+-------------+

        while (!(prev_B_ButtonState | prev_X_ButtonState)) {

            cur_B_ButtonState = gamepad1.b;                           // get the current state of button b
            cur_X_ButtonState = gamepad1.x;                           // get the current state of button x

            if (cur_B_ButtonState) {                                  // when the "b" button on the gamepad is pressed
                if (!prev_B_ButtonState) {                            // when the previous "b" button was NOT pushed
                    selOptions[selIndex][1] = posResps[0];            // first response was selected, store the response
                    prev_B_ButtonState = true;                        // indicate that the previous B button state is PUSHED
                }
            }
            else if (cur_X_ButtonState) {                             // when the "x" button on the gamepad is pressed
                if (!prev_X_ButtonState) {                            // when the previous "x" button was NOT pushed
                    selOptions[selIndex][1] = posResps[1];            // second response was selected, store the response
                    prev_X_ButtonState = true;                        // indicate that the previous X button state is PUSHED
                }
            }
        }

        telemetry.addLine().addData(selOptions[selIndex][0], selOptions[selIndex][1]);
        telemetry.update();

        // Wait one second
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * configForAuto4Options will save the response (selOptions) from a set of 4 possible responses (posResps)
     * a given prompt (selPrompt)
     *
     * @param selPrompt The given configuration prompt
     * @param posResps The possible responses to a given configuration prompt
     * @param selIndex The position within selOptions array in which to store the responses
     * @param selOptions Where to store the configuration prompt and response
     */
    public void configForAuto4Options(String selPrompt, String[] posResps, int selIndex, String[][] selOptions) {

        // Let the user Select
        gamepad1.reset();

        prev_B_ButtonState = false;
        prev_X_ButtonState = false;
        prev_A_ButtonState = false;
        prev_Y_ButtonState = false;
        cur_B_ButtonState = false;
        cur_X_ButtonState = false;
        cur_A_ButtonState = false;
        cur_Y_ButtonState = false;

        selOptions[selIndex][0] = selPrompt;

        // Prompt User for Selection
        telemetry.addLine(selPrompt);
        telemetry.addLine("Press B for " + posResps[0] + " or X for " + posResps[1]
                + " or A for " + posResps[2] + " or Y for " + posResps[3]);
        telemetry.update();

        while (!(prev_B_ButtonState | prev_X_ButtonState | prev_A_ButtonState | prev_Y_ButtonState)) {

            cur_B_ButtonState = gamepad1.b;                           // get the current state of button b
            cur_X_ButtonState = gamepad1.x;                           // get the current state of button x
            cur_A_ButtonState = gamepad1.a;                           // get the current state of button a
            cur_Y_ButtonState = gamepad1.y;                           // get the current state of button y

            if (cur_B_ButtonState) {                                  // when the "b" button on the gamepad is pressed
                if (!prev_B_ButtonState) {                            // when the previous "b" button was NOT pushed
                    selOptions[selIndex][1] = posResps[0];            // first response was selected, store the response
                    prev_B_ButtonState = true;                        // indicate that the previous B button state is PUSHED
                }
            }
            else if (cur_X_ButtonState) {                             // when the "x" button on the gamepad is pressed
                if (!prev_X_ButtonState) {                            // when the previous "x" button was NOT pushed
                    selOptions[selIndex][1] = posResps[1];            // second response was selected, store the response
                    prev_X_ButtonState = true;                        // indicate that the previous X button state is PUSHED
                }
            }
            else if (cur_A_ButtonState) {                             // when the "a" button on the gamepad is pressed
                if (!prev_A_ButtonState) {                            // when the previous "a" button was NOT pushed
                    selOptions[selIndex][1] = posResps[2];            // third response was selected, store the response
                    prev_A_ButtonState = true;                        // indicate that the previous A button state is PUSHED
                }
            }
            else if (cur_Y_ButtonState) {                             // when the "y" button on the gamepad is pressed
                if (!prev_Y_ButtonState) {                            // when the previous "y" button was NOT pushed
                    selOptions[selIndex][1] = posResps[3];            // fourth response was selected, store the response
                    prev_Y_ButtonState = true;                        // indicate that the previous Y button state is PUSHED
                }
            }
        }

        telemetry.addLine().addData(selOptions[selIndex][0], selOptions[selIndex][1]);
        telemetry.update();

        // Wait one second
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}