package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

@Autonomous


public class IndieAutonomousOptions extends RoboRaidersAuto {

     public Robot robot = new Robot();

    boolean cur_B_ButtonState;                                            // "b" button current state
    boolean cur_X_ButtonState;                                            // "x" button current state

    boolean prev_B_ButtonState;                                           // "b" button previous state
    boolean prev_X_ButtonState;                                           // "x" button previous state

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        // Array to store selected options, by design selectedOptions[0][*] is used to store the
        // response for finalizing the selected options.
        String[][] selectedOptions = new String[5][2];                    // Array to store selections

        String allianceSelPrompt = "Alliance";                            // Alliance prompt
        final String[] alliancePosResps = new String[]{"red", "blue"};    // Possible Alliance selections

        String bsSelPrompt = "Balancing Stone";                           // Balance Stone prompt
        final String[] bsPosResps = new String[]{"close", "far"};         // Possible Balance Stone selections

        String jewelSelPrompt = "Jewel";                                  // Jewel prompt

        String parkSelPrompt = "Park";                                    // Park prompt

        final String[] yesNoOptions = new String[]{"no", "yes"};          // Possible selections for Jewel and Park

        String selectionsOk = "Selections Great :)";                      // Finalize selections prompt

        // Add new selections and associated responses before here


        // Commented out any code related to robot and running the robot, only because we are still in the process of testing and tweaking it
        robot.initialize(hardwareMap);
        robot.initializeServos();
        //vuforiaInitialization(hardwareMap);
        telemetry.addData("Initialized", true);
        telemetry.update();

        // Default that the options to be selected are not finalized.  This is done since we want
        // the loop below to actually loop.  At the end of the loop, the question will be asked if
        // the selections are ok, if they are, then this will be set to "yes" and the loop will
        // exit
        selectedOptions[0][1] = "no";

        // Configure for Indie autonomous while the selections are not finalized
        while (selectedOptions[0][1].equals("no")) {

            //             Prompt              Responses       Index    Options output
            configForAuto(allianceSelPrompt, alliancePosResps, 1, selectedOptions);   // Alliance selection
            configForAuto(bsSelPrompt, bsPosResps, 2, selectedOptions);   // Balance stone selection
            configForAuto(jewelSelPrompt, yesNoOptions, 3, selectedOptions);   // Jewel selection
            configForAuto(parkSelPrompt, yesNoOptions, 4, selectedOptions);   // Park selection


            // Add new selections above here

            // Loop through all of the selections and tell user what s/he has selected
            for (int i = 1; i <= 4; i++) {

                telemetry.addLine().addData(selectedOptions[i][0], selectedOptions[i][1]);
            }

            // Are you sure about the options selected?
            //             Prompt         Responses  Index    Options output
            configForAuto(selectionsOk, yesNoOptions, 0, selectedOptions); // Finalize Selection

        }

        telemetry.addLine("Options Selected - Waiting for Start");        // Tell user that options have been selected

        //wait for start to be pushed
        waitForStart();

        // Jewel
        if (selectedOptions[3][1].equals("yes")) {

            lowerArm(robot);
            selectJewel(robot, selectedOptions[1][1]);
        }
        else if (selectedOptions[3][1].equals("no")) {

        }

        // Park
        if (selectedOptions[4][1].equals("yes")) {

            if (selectedOptions[1][1].equals("blue") && selectedOptions[2][1].equals("close")) {

                encodersMove(robot, 32, 0.5, "forward");
                Thread.sleep(500);

                imuTurn(robot, 90, 0.5, "left");
                Thread.sleep(500);

                encodersMove(robot, 2, 0.5, "forward");
                Thread.sleep(500);
            }
            if (selectedOptions[1][1].equals("blue") && selectedOptions[2][1].equals("far")) {

                encodersMove(robot, 22, 0.5, "forward");
                Thread.sleep(500);

                encodersMove(robot, 18, 0.5, "right");
                Thread.sleep(500);
            }
            if (selectedOptions[1][1].equals("red") && selectedOptions[2][1].equals("close")) {

                encodersMove(robot, 32, 0.5, "backward");
                Thread.sleep(500);

                imuTurn(robot, 90, 0.5, "left");
                Thread.sleep(500);

                encodersMove(robot, 2, 0.5, "forward");
                Thread.sleep(500);
            }
            if (selectedOptions[1][1].equals("red") && selectedOptions[2][1].equals("far")) {

                encodersMove(robot, 22, 0.5, "backward");
                Thread.sleep(500);

                encodersMove(robot, 18, 0.5, "right");
                Thread.sleep(500);
            }
            else if (selectedOptions[4][1].equals("no")) {

            }
        }
    }

    /**
     * configForAuto will save the response (selOptions) from a set of possible responses (posResps)
     * a given prompt (selPrompt)
     *
     * @param selPrompt The given configuration prompt
     * @param posResps The possible responses to a given configuration prompt
     * @param selIndex The position within selOptions array in which to store the responses
     * @param selOptions where to store the configuration prompt and response
     *
     */
    public void configForAuto(String selPrompt, String[] posResps, int selIndex, String[][] selOptions) {

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

            if (cur_B_ButtonState) {                                  // when the "b" button on the gamepad is pressed set alliance to RED
                if (!prev_B_ButtonState) {                            // when the previous "b" button was NOT pushed
                    selOptions[selIndex][1] = posResps[0];            // First response was selected, store the response
                    prev_B_ButtonState = true;                        // indicate that the previous B button state is PUSHED
                }
            } else if (cur_X_ButtonState) {                           // when the "X" button on the gamepad is pressed set the alliance to BLUE
                if (!prev_X_ButtonState) {                            // when the previous "x" button was NOT pushed
                    selOptions[selIndex][1] = posResps[1];            // Second Response was selected, store the response
                    prev_X_ButtonState = true;                        // indicate that the previous X button state is PUSHED
                }
            }
        }

        telemetry.addLine().addData(selOptions[selIndex][0], selOptions[selIndex][1]);
        telemetry.update();

        // Wait one half of a second
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}