package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by FTC Team 11574 on 2/18/2017.
 */

// set name to show on phone
@Autonomous(name="Test Range Sensor", group="Autonomous")

public class Test_Range_Sensor extends Mecanum_Wheels_Generic {

    @Override
    public void runOpMode() throws InterruptedException {
        robotInit();

        while(!isStarted()) {
            telemetry.addData("range", range.cmUltrasonic());
            telemetry.addData("alpha", Tape_color.alpha());
            telemetry.update();
        }

        waitForStart();

        drive_until_gt_alpha(STRAFE_RIGHT, 3.0, 24.0, 0.5);
        drive_until_lt_range(DRIVE_FORWARD, 10.0, 24.0, 0.5);
        drive_until_lt_range(DRIVE_FORWARD, 5.0, 10.0, 0.2);
    }
}
