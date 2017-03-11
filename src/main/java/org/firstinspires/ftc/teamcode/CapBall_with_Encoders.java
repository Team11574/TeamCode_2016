/*
 * Copyright 2017, FTC Team 11574.
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CapBall with Encoders", group="Autonomous")
@SuppressWarnings("unused")
public class CapBall_with_Encoders extends Mecanum_Wheels_Generic {
    @Override
    public void robotRun() {
        sleep(10000);   // Waits ten seconds and then starts the program.

        // Drive to push CapBall and Park.
        drive_distance(DRIVE_FORWARD, 64.0, 0.8);
        drive_distance(DRIVE_BACKWARD, 2.0, 0.2);
        stop_all_motors();
    }
}