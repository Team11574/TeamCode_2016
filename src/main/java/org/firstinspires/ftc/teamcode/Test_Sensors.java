package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test Sensors", group="Autonomous")
public class Test_Sensors extends Mecanum_Wheels_Generic {

    @Override
    public void runOpMode() throws InterruptedException {
        robotInit();

        while(!isStarted() || isStopRequested()) {
            telemetry.addData("1. range u/s", range.cmUltrasonic());
            telemetry.addData("2. range opt", range.cmOptical());
            telemetry.addData("3. tape alpha", Tape_color.alpha());
            telemetry.addData("4. beacon red", Beacon_color.red());
            telemetry.addData("5. beacon blue", Beacon_color.blue());
            //telemetry.addData("6. gyro heading", gyro.getHeading());
            telemetry.update();
        }

        waitForStart();
    }
}
