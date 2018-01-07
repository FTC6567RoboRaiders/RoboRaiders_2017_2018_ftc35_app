package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

/**
 * Created by Jason Sember on 11/5/17.
 */

@Autonomous

public class IndieAutonomousOptions extends RoboRaidersAuto {

    public Robot robot = new Robot();

    // Set up strings for yes/no options
    private String[][] selectedOptions = new String[5][2];
    String[] yesNoOptions = new String[]{"no", "yes"};

    // Set up booleans for the b and x buttons
    boolean cur_B_ButtonState;
    boolean cur_X_ButtonState;
    boolean prev_B_ButtonState;
    boolean prev_X_ButtonState;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
        robot.initializeServos();
        vuforiaInitialization(hardwareMap);
        telemetry.addData("Initialized", true);
        telemetry.update();

        // Set up strings for all other options
        String allianceSelPrompt = new String("Alliance");
        final String[] alliancePosResps = new String[]{"red", "blue"};
        String bsSelPrompt = new String("Balancing Stone");
        final String[] bsPosResps = new String[]{"close", "far"};

        String jewelSelPrompt = new String("Jewel");
        String parkSelPrompt = new String("Park");
        String selectionsOk = new String("Selections Great :)");

        selectedOptions[0][1] = "no";

        // Configure for Indie autonomous
        while (selectedOptions[0][1].equals("no")) {

            configForAuto(allianceSelPrompt, alliancePosResps, 1, selectedOptions);
            configForAuto(bsSelPrompt, bsPosResps, 2, selectedOptions);
            configForAuto(jewelSelPrompt, yesNoOptions, 3, selectedOptions);
            configForAuto(parkSelPrompt, yesNoOptions, 4, selectedOptions);

            // Tell user what he or she has selected
            for (int i = 1; i <= 4; i++) {

                telemetry.addLine().addData(selectedOptions[i][0], selectedOptions[i][1]);
            }
            telemetry.update();

            // Are you sure about your options?
            configForAuto(selectionsOk, yesNoOptions, 0, selectedOptions);
        }

        // Wait for start to be pushed
        waitForStart();
    }

    /**
     * @param selPrompt The given configuration prompt
     * @param PosResps The possible responses to a given configuration prompt
     * @param selIndex The position within selOptions array in which to store the responses
     * @param selOptions where to store the configuration prompt and response
     *
     * loop until either the "b" button or the "x" button is pressed
     * the logic here says OR the previous button states and when they are both false continue
     * here is a table of how this works
     *    +--------------------+------+--------------------+--------+-------------+
     *    | prev_B_ButtonState | -OR- | prev_X_ButtonState | Result | Neg. Result |
     *    +--------------------+------+--------------------+--------+-------------+
     *    |      FALSE         | -OR- |     FALSE          | FALSE  |   TRUE      |
     *    +--------------------+------+--------------------+--------+-------------+
     *    |      FALSE         | -OR- |     TRUE           | TRUE   |   FALSE     |
     *    +--------------------+------+--------------------+--------+-------------+
     *    |      TRUE          | -OR- |     FALSE          | TRUE   |   FALSE     |
     *    +--------------------+------+--------------------+--------+-------------+
     *    |      TRUE          | -OR- |     TRUE           | TRUE   |   FALSE     |
     *    +--------------------+------+--------------------+--------+-------------+
     */
    public void configForAuto(String selPrompt, String[] PosResps, int selIndex, String[][] selOptions) {

        selOptions[selIndex][0] = selPrompt;

        // Prompt user for selection
        telemetry.addLine(selPrompt);
        telemetry.addLine("Press B for " + PosResps[0] + " or X for " + PosResps[1]);
        telemetry.update();

        // Let the user select
        gamepad1.reset();

        prev_B_ButtonState = false;
        prev_X_ButtonState = false;
        cur_B_ButtonState = false;
        cur_X_ButtonState = false;

        while (!(prev_B_ButtonState | prev_X_ButtonState)) {

            cur_B_ButtonState = gamepad1.b;                           // get the current state of button b
            cur_X_ButtonState = gamepad1.x;                           // get the current state of button x

            if (cur_B_ButtonState) {                                  // when the "b" button on the gamepad is pressed set alliance to RED
                if (!prev_B_ButtonState) {                            // when the previous "b" button was NOT pushed
                    selOptions[selIndex][1] = PosResps[0];            // First response was selected, store the response
                    prev_B_ButtonState = true;                        // indicate that the previous B button state is PUSHED
                }
            } else if (cur_X_ButtonState) {                           // when the "x" button on the gamepad is pressed set the alliance to BLUE
                if (!prev_X_ButtonState) {                            // when the previous "x" button was NOT pushed
                    selOptions[selIndex][1] = PosResps[1];            // Second Response was selected, store the response
                    prev_X_ButtonState = true;                        // indicate that the previous X button state is PUSHED
                }
            }

            // Wait one second
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}





