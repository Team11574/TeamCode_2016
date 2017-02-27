package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by FTC Team 11574 on 2/25/2017.
 *
 * Drive away from the wall and park on the ramp.
 */

@Autonomous(name="Park on Ramp", group="Autonomous")
@SuppressWarnings("unused")
public class Park_on_Ramp extends Mecanum_Wheels_Generic {
    @Override
    public void robotRun() {
        drive_distance(STRAFE_RIGHT, 35.0, 0.6);
        drive_distance(TURN_LEFT, 10.0, 0.2);
        drive_distance(DRIVE_FORWARD, 35.0, 0.6);
    }
}

