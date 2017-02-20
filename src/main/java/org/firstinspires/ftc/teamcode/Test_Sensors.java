package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test Sensors", group="Autonomous")
public class Test_Sensors extends Mecanum_Wheels_Generic {

    @Override
    public void runOpMode() throws InterruptedException {
        robotInit();

        while(!isStarted() || isStopRequested()) {
            telemetry.addData("1. alliance", COLOR_NAMES[check_alliance()]);
            telemetry.addData("2. range u/s", range.cmUltrasonic());
            telemetry.addData("3. range opt", range.cmOptical());
            telemetry.addData("4. tape alpha", Tape_color.alpha());
            telemetry.addData("5. beacon red", Beacon_color.red());
            telemetry.addData("6. beacon blue", Beacon_color.blue());
            //telemetry.addData("7. gyro heading", gyro.getHeading());
            telemetry.update();
        }

        waitForStart();
    }
}
