package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * {@link HubBot_Auto_Options} prototype for autonomous selection of options, things
 * like alliance, how many glyphs to obtain and place, location, etc....
 *
 * more later....
 *
 *
 **/

@Autonomous(name = "HubBot: Autonomous Options", group = "Auto")

public class HubBot_Auto_Options extends LinearOpMode {

    View relativeLayout;

    // Set up strings for alliance selection
    String allianceTitle = "Alliance Selection";                      // The title of this selection
    String[] allianceOptions = new String[] {"Red", "Blue"};          // The options for this selection
    String allianceSelection;                                         // The alliance selection

    // Set up strings for balance stone selection
    String bsTitle = "Balancing Stone Selection";
    String[] bsOptions = new String[] {"Close", "Far"};
    String bsSelection;

    String[] yesNoOptions = new String[] {"No", "Yes"};

    // Set up strings for jewel selection
    String jewelTitle = "Jewel Selection";
    String jewelSelection;

    // Set up strings for park selection
    String parkTitle = "Park Selection";
    String parkSelection;

    boolean cur_B_ButtonState;                                        // "b" button current state
    boolean cur_X_ButtonState;                                        // "x" button current state

    boolean prev_B_ButtonState;                                       // "b" button previous state
    boolean prev_X_ButtonState;                                       // "x" button previous state

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the alliance selection
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // Alliance Selection
        telemetry.addLine(allianceTitle);
        telemetry.addLine("Press B for Red or X for Blue");
        telemetry.update();

        gamepad1.reset();                                             // reset the gamepad to initial state

        /*
        Default the previous button states for the "B" and "X" button to indicate that the
        buttons were not pushed.
         */
        prev_X_ButtonState = false;
        prev_B_ButtonState = false;
        cur_B_ButtonState = false;
        cur_X_ButtonState = false;

        // loop until either the "b" button or the "x" button is pressed
        // the logic here says OR the previous button states and when they are both false continue
        // here is a table of how this works
        /*
             +--------------------+------+--------------------+--------+-------------+
             | prev_B_ButtonState | -OR- | prev_X_ButtonState | Result | Neg. Result |
             +--------------------+------+--------------------+--------+-------------+
             |      FALSE         | -OR- |     FALSE          | FALSE  |   TRUE      |
             +--------------------+------+--------------------+--------+-------------+
             |      FALSE         | -OR- |     TRUE           | TRUE   |   FALSE     |
             +--------------------+------+--------------------+--------+-------------+
             |      TRUE          | -OR- |     FALSE          | TRUE   |   FALSE     |
             +--------------------+------+--------------------+--------+-------------+
             |      TRUE          | -OR- |     TRUE           | TRUE   |   FALSE     |
             +--------------------+------+--------------------+--------+-------------+
         */
        while ( !(prev_B_ButtonState | prev_X_ButtonState) ) {

            cur_B_ButtonState = gamepad1.b;                           // get the current state of button b
            cur_X_ButtonState = gamepad1.x;                           // get the current state of button x

            if (cur_B_ButtonState) {                                  // when the "b" button on the gamepad is pressed set alliance to RED
                if (!prev_B_ButtonState) {                            // when the previous "b" button was NOT pushed
                    allianceSelection = allianceOptions[0];           // set alliance selection to RED
                    prev_B_ButtonState = true;                        // indicate that the previous B button state is PUSHED
                }
            }

            else if (cur_X_ButtonState) {                             // when the "X" button on the gamepad is pressed set the alliance to BLUE
                if (!prev_X_ButtonState) {                            // when the previous "x" button was NOT pushed
                    allianceSelection = allianceOptions[1];           // set alliance selection to BLUE
                    prev_X_ButtonState = true;                        // indicate that the previous X button state is PUSHED
                }
            }

            telemetry.update();                                       // so when this line is removed we get a problem with
            // the state of the prev variables...not sure what java/android
            // thinks is going on here...more investigation is needed
        }

        telemetry.addLine().addData("Alliance Selection: ", allianceSelection);
        telemetry.update();

        // Change the background color to match the alliance selection
        relativeLayout.post(new Runnable() {

            public void run() {

                if ( allianceSelection.equals(allianceOptions[0]) ) { // alliance selection is RED

                    relativeLayout.setBackgroundColor(Color.RED);
                }
                else {                                                // alliance selection is BLUE

                    relativeLayout.setBackgroundColor(Color.BLUE);
                }
            }
        });

        try {

            Thread.sleep(1000);
        }
        catch (InterruptedException e) {

            e.printStackTrace();
        }

        // Balancing Stone Selection
        telemetry.addLine(bsTitle);
        telemetry.addLine("Press B for Near or X for Away");
        telemetry.update();

        gamepad1.reset();                                             // reset the gamepad to initial state

        prev_B_ButtonState = false;
        prev_X_ButtonState = false;
        cur_B_ButtonState = false;
        cur_X_ButtonState = false;

        while ( !(prev_B_ButtonState | prev_X_ButtonState) ) {

            cur_B_ButtonState = gamepad1.b;                           // get the current state of button b
            cur_X_ButtonState = gamepad1.x;                           // get the current state of button x

            if (cur_B_ButtonState) {                                  // when the "b" button on the gamepad is pressed set alliance to RED
                if (!prev_B_ButtonState) {                            // when the previous "b" button was NOT pushed
                    bsSelection = bsOptions[0];                       // set balance stone selection to Near
                    prev_B_ButtonState = true;                        // indicate that the previous B button state is PUSHED
                }
            }

            else if (cur_X_ButtonState) {                             // when the "X" button on the gamepad is pressed set the alliance to BLUE
                if (!prev_X_ButtonState) {                            // when the previous "x" button was NOT pushed
                    bsSelection = bsOptions[1];                       // set balance stone selection to Away
                    prev_X_ButtonState = true;                        // indicate that the previous X button state is PUSHED
                }
            }

            telemetry.update();                                       // so when this line is removed we get a problem with
            // the state of the prev variables...not sure what java/android
            // thinks is going on here...more investigation is needed
        }

        telemetry.addLine().addData("Balancing Stone Selection: ", bsSelection);
        telemetry.update();

        try {

            Thread.sleep(1000);
        }
        catch (InterruptedException e) {

            e.printStackTrace();
        }

        // Jewel Selection
        telemetry.addLine(jewelTitle);
        telemetry.addLine("Press B for No or X for Yes");
        telemetry.update();

        gamepad1.reset();                                             // reset the gamepad to initial state

        prev_B_ButtonState = false;
        prev_X_ButtonState = false;
        cur_B_ButtonState = false;
        cur_X_ButtonState = false;

        while ( !(prev_B_ButtonState | prev_X_ButtonState) ) {

            cur_B_ButtonState = gamepad1.b;                           // get the current state of button b
            cur_X_ButtonState = gamepad1.x;                           // get the current state of button x

            if (cur_B_ButtonState) {                                  // when the "b" button on the gamepad is pressed set alliance to RED
                if (!prev_B_ButtonState) {                            // when the previous "b" button was NOT pushed
                    jewelSelection = yesNoOptions[0];                       // set balance stone selection to Near
                    prev_B_ButtonState = true;                        // indicate that the previous B button state is PUSHED
                }
            }

            else  if (cur_X_ButtonState) {                            // when the "X" button on the gamepad is pressed set the alliance to BLUE
                if (!prev_X_ButtonState) {                            // when the previous "x" button was NOT pushed
                    jewelSelection = yesNoOptions[1];                       // set balance stone selection to Away
                    prev_X_ButtonState = true;                        // indicate that the previous X button state is PUSHED
                }
            }

            telemetry.update();                                       // so when this line is removed we get a problem with
            // the state of the prev variables...not sure what java/android
            // thinks is going on here...more investigation is needed
        }

        telemetry.addLine().addData("Jewel Selection: ", jewelSelection);
        telemetry.update();

        try {

            Thread.sleep(500);
        }
        catch (InterruptedException e) {

            e.printStackTrace();
        }

        // Park Selection
        telemetry.addLine(parkTitle);
        telemetry.addLine("Press B for No or X for Yes");
        telemetry.update();

        gamepad1.reset();

        prev_B_ButtonState = false;
        prev_X_ButtonState = false;
        cur_B_ButtonState = false;
        cur_X_ButtonState = false;

        while ( !(prev_B_ButtonState | prev_X_ButtonState) ) {

            cur_B_ButtonState = gamepad1.b;                           // get the current state of button b
            cur_X_ButtonState = gamepad1.x;                           // get the current state of button x

            if (cur_B_ButtonState) {                                  // when the "b" button on the gamepad is pressed set alliance to RED
                if (!prev_B_ButtonState) {                            // when the previous "b" button was NOT pushed
                    parkSelection = yesNoOptions[0];                  // set balance stone selection to Near
                    prev_B_ButtonState = true;                        // indicate that the previous B button state is PUSHED
                }
            }

            else  if (cur_X_ButtonState) {                            // when the "X" button on the gamepad is pressed set the alliance to BLUE
                if (!prev_X_ButtonState) {                            // when the previous "x" button was NOT pushed
                    parkSelection = yesNoOptions[1];                  // set balance stone selection to Away
                    prev_X_ButtonState = true;                        // indicate that the previous X button state is PUSHED
                }
            }

            telemetry.update();                                       // so when this line is removed we get a problem with
            // the state of the prev variables...not sure what java/android
            // thinks is going on here...more investigation is needed
        }

        telemetry.addLine().addData("Park Selection: ", parkSelection);
        telemetry.update();

        try {

            Thread.sleep(1000);
        }
        catch (InterruptedException e) {

            e.printStackTrace();
        }

        waitForStart();

        // Loop and update the dashboard
        while (opModeIsActive()) {

            telemetry.update();
        }

        // Change the background color back to white
        relativeLayout.post(new Runnable() {
            public void run() {

                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}