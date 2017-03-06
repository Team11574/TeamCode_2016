/*
 * Copyright 2017, FTC Team 11574.
 *
 * A complex Autonomous program which aims to find and press the beacons to
 * the alliance color. It uses encoder-based distance driving, a range sensor,
 * a tape-finding color sensor, and a beacon color sensor to do so.
 */

package org.firstinspires.ftc.teamcode;

import java.util.Locale;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// The program name to show on the driver station phone.
@Autonomous(name="Drive and Push Beacons", group="Autonomous")
@SuppressWarnings("unused")
public class Drive_and_Push_Beacons extends Mecanum_Wheels_Generic {
    @Override
    public void robotRun() {
        // Read the alliance color from the alliance switch.
        AllianceColor color_alliance = check_alliance();

        info("Ready to run for " + color_alliance + " alliance.");

        // Set up strafe_away and strafe_back variables to use the alliance color to decide which
        // way we will need to strafe (left or right). We can then use these consistently in the
        // rest of the code without worrying about which alliance we're on.
        int strafe_away, strafe_back;
        if (color_alliance == AllianceColor.Red) {
            strafe_away = STRAFE_RIGHT;
            strafe_back = STRAFE_LEFT;
        } else if(color_alliance == AllianceColor.Blue) {
            strafe_away = STRAFE_LEFT;
            strafe_back = STRAFE_RIGHT;
        } else {
            // There's nothing we can do here.
            info("Unable to determine alliance. Exiting!");
            return;
        }

        /*
            Calibrate the Tape_color sensor to the alpha value of the floor in the starting
            position, so that if the floor is a different reflectivity it won't matter. We're
            just looking for tape that is *more* reflective by TAPE_ALPHA_DIFFERENCE than the
            calibrated value of the floor at the starting position.
         */
        double floor_alpha = Tape_color.alpha();
        double tape_alpha = floor_alpha + TAPE_ALPHA_DIFFERENCE;

        info(String.format(Locale.US, "Measured floor alpha=%.2f; looking for tape alpha=%.2f.",
                floor_alpha, tape_alpha));

        info("Ready to drive!");

        /*
            The strategy to align to the first beacon here is:
              1. Strafe away from the wall towards the middle of the arena, but remain clear
                 of the cap ball.
              2. Drive forwards until we're within range of the color sensor being able to
                 find the tape line while strafing.
              3. Strafe towards the tape line until we find it.
              4. Drive forwards quickly and then slowly towards the wall until within range of
                 the button pushing code to push the button.
         */
        info("Re-positioning to Beacon 1...");
        drive_distance(strafe_away, 3.0, 0.3);
        drive_distance(strafe_away, 3.0, 0.6);
        drive_distance(strafe_away, 22.0, 1.0);
        drive_distance(strafe_away, 6.0, 0.3);
        stop_all_motors();
        drive_distance(DRIVE_FORWARD, 3.0, 0.3);
        drive_distance(DRIVE_FORWARD, 3.0, 0.6);
        drive_distance(DRIVE_FORWARD, 9.0, 1.0);
        drive_distance(DRIVE_FORWARD, 6.0, 0.6);
        drive_distance(DRIVE_FORWARD, 3.0, 0.3);
        stop_all_motors();
        drive_distance(strafe_away, 12.0, 0.6);
        drive_until_gt_alpha(strafe_away, tape_alpha, 12.0, 0.3);
        stop_all_motors();
        drive_until_lt_range(5.0, 20.0, 0.2);
        stop_all_motors();

        // Check the colors of each side of the beacon and then push the appropriate one.
        check_beacons_and_push_button("Beacon 1", color_alliance, tape_alpha, strafe_away, strafe_back);

        // Back up a bit to avoid hitting anything while driving quickly to the next beacon.
        drive_until_gt_range(10.0, 10.0, 0.6);
        stop_all_motors();

        /*
            The strategy to align to the second beacon here is:
              1. Strafe towards the second beacon quickly around 70% of the total distance.
              2. Strafe towards the tape line until we find it.
              3. Drive forwards quickly and then slowly as above.
         */
        info("Re-positioning to Beacon 2...");
        drive_distance(strafe_away, 3.0, 0.3);
        drive_distance(strafe_away, 6.0, 0.6);
        drive_distance(strafe_away, 25.0, 1.0);
        drive_distance(strafe_away, 6.0, 0.6);
        drive_until_gt_alpha(strafe_away, tape_alpha, 12.0, 0.3);
        stop_all_motors();
        drive_until_lt_range(5.0, 20.0, 0.2);
        stop_all_motors();

        // Check the colors of each side of the beacon and then push the appropriate one.
        check_beacons_and_push_button("Beacon 2", color_alliance, tape_alpha, strafe_away, strafe_back);

        // Back up a bit to avoid hitting anything while driving quickly cap ball.
        drive_until_gt_range(10.0, 10.0, 0.6);
        stop_all_motors();

        // Strafe back to the first beacon's tape line, then back up to bump the cap ball and
        // park on the center.
        info("Re-positioning to bump the cap ball...");
        drive_distance(strafe_back, 3.0, 0.3);
        drive_distance(strafe_back, 3.0, 0.6);
        drive_distance(strafe_back, 30.0, 1.0);
        drive_until_gt_alpha(strafe_back, tape_alpha, 24.0, 0.6);
        stop_all_motors();
        drive_distance(DRIVE_BACKWARD, 36.0, 1.0);
        stop_all_motors();

        info("Driving complete!");
    }
}


