package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.roboraiders.Robot.RoboRaidersAuto;
import com.roboraiders.Robot.Robot;

@Autonomous

public class IndieAutonomousOptions extends RoboRaidersAuto {

    //    public Robot robot = new Robot();
    private String[][] selectedOptions = new String[5][2];
    // int soIdx;

    View relativeLayout;

    // Set up strings for alliance selection
   // String allianceTitle = "Alliance Selection";                      // The title of this selection
   // String[] allianceOptions = new String[]{"red", "blue"};          // The options for this selection
    String allianceSelection;                                         // The alliance selection

    /* Set up strings for balancing stone selection
    String bsTitle = "Balancing Stone Selection";
    String[] bsOptions = new String[]{"close", "far"};
    String bsSelection;*/

    // yes/no Options
    String[] yesNoOptions = new String[]{"no", "yes"};

    /* Set up strings for jewel selection
    String jewelTitle = "Jewel Selection";
    String jewelSelection;

    // Set up strings for park selection
    String parkTitle = "Park Selection";
    String parkSelection;*/

    boolean cur_B_ButtonState;                                        // "b" button current state
    boolean cur_X_ButtonState;                                        // "x" button current state

    boolean prev_B_ButtonState;                                       // "b" button previous state
    boolean prev_X_ButtonState;                                       // "x" button previous state

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        //robot.initialize(hardwareMap);
        //robot.initializeServos();

        telemetry.addData("Initialized", true);
        telemetry.update();

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the alliance selection
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        String allianceSelPrompt = new String("Alliance");
        final String[] alliancePosResps = new String[]{"red", "blue"};

        String bsSelPrompt = new String("Balancing Stone");
        String[] bsPosResps = new String[]{"close", "far"};

        String jewelSelPrompt = new String("Jewel");

        String parkSelPrompt = new String("Park");

        String selectionsOk = new String("Selections Great :)");

        // Configure for Indie autonomous
        while (!selectedOptions[0][1].equals("Yes")) {
            configForAuto(allianceSelPrompt, alliancePosResps, 1, selectedOptions);
            configForAuto(bsSelPrompt, bsPosResps, 2, selectedOptions);
            configForAuto(jewelSelPrompt, yesNoOptions, 3, selectedOptions);
            configForAuto(parkSelPrompt, yesNoOptions, 4, selectedOptions);

            //  tell user what s/he has selected
            for (int i = 1; i <= 4; i++) {
                telemetry.addLine().addData(selectedOptions[i][0], selectedOptions[i][1]);
            }
            telemetry.update();

            //Are you sure about your Options?
            configForAuto(selectionsOk, yesNoOptions, 0, selectedOptions);
        }








        //wait for start to be pushed
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
     *
     *
     *
     *
     */

    public void configForAuto(String selPrompt, String[] PosResps, int selIndex, String[][] selOptions) {

        selOptions[selIndex][0] = selPrompt;
        // Prompt User for Selection
        telemetry.addLine(selPrompt);
        telemetry.addLine("Press B for " + PosResps[0] + " or X for " + PosResps[1]);
        telemetry.update();

        // Let the user Select
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
                    selOptions[selIndex][1] = PosResps[0];                   // First response was selected, store the response
                    prev_B_ButtonState = true;                        // indicate that the previous B button state is PUSHED
                }
            } else if (cur_X_ButtonState) {                            // when the "X" button on the gamepad is pressed set the alliance to BLUE
                if (!prev_X_ButtonState) {                            // when the previous "x" button was NOT pushed
                    selOptions[selIndex][1] = PosResps[1];                   // Second Response was selected, store the response
                    prev_X_ButtonState = true;                        // indicate that the previous X button state is PUSHED
                }
            }

            //wait a second
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }


}

        // Jewel
 /*       if (jewelSelection.equals("yes")) {

            lowerArm(robot, 0.99);
            selectJewel(robot, allianceSelection);
        }
        else if (jewelSelection.equals("no")) {

        }
        
        // Park
        if (parkSelection.equals("yes")) {
            
            if (allianceSelection.equals("blue") && bsSelection.equals("close")) {

                encodersMove(robot, 32, 0.5, "forward");
                Thread.sleep(500);

                imuTurn(robot, 90, 0.5, "left");
                Thread.sleep(500);

                encodersMove(robot, 2, 0.5, "forward");
                Thread.sleep(500);
            }
            else if (allianceSelection.equals("blue") && bsSelection.equals("far")) {

                encodersMove(robot, 22, 0.5, "forward");
                Thread.sleep(500);

                encodersMove(robot, 18, 0.5, "right");
                Thread.sleep(500);
            }
            else if (allianceSelection.equals("red") && bsSelection.equals("close")) {

                encodersMove(robot, 32, 0.5, "backward");
                Thread.sleep(500);

                imuTurn(robot, 90, 0.5, "left");
                Thread.sleep(500);

                encodersMove(robot, 2, 0.5, "forward");
                Thread.sleep(500);
            }
            else if (allianceSelection.equals("red") && bsSelection.equals("far")) {

                encodersMove(robot, 22, 0.5, "backward");
                Thread.sleep(500);

                encodersMove(robot, 18, 0.5, "right");
                Thread.sleep(500);
            }
        }
        else if (parkSelection.equals("no")){

        }


        } */





