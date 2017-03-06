/*
 * Copyright 2017, FTC Team 11574.
 *
 * A simple Autonomous program to drive away from the wall, turn towards
 * the ramp, and then drive forward to park on the ramp.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Park on Ramp", group="Autonomous")
@SuppressWarnings("unused")
public class Park_on_Ramp extends Mecanum_Wheels_Generic {
    @Override
    public void robotRun() {
        AllianceColor color_alliance = check_alliance();
        int strafe_direction, turn_direction;
        if (color_alliance == AllianceColor.Red) {
            strafe_direction = STRAFE_RIGHT;
            turn_direction = TURN_LEFT;
        } else {
            strafe_direction = STRAFE_LEFT;
            turn_direction = TURN_RIGHT;
        }
        drive_distance(strafe_direction, 35.0, 0.6);
        drive_distance(turn_direction, 10.0, 0.2);
        drive_distance(DRIVE_FORWARD, 35.0, 0.6);
    }
}

