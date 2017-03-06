// Basic autonomous program that drives the Omni Wheel robot forward .5 seconds then stops.
// It will then turn to the right.
// Then drive forward, hopefully onto the ramp for 5 points.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

// Below is the Annotation that registers this OpMode with the FtcRobotController app.
// @Autonomous classifies the OpMode as autonomous, name is the OpMode title and the
// optional group places the OpMode into the Omni Wheels group.
// We called this Blue Ramp Omni Wheels so it is used for Autonomous if we are on the Blue
// Alliance team and it uses the Omni Wheels.


@Autonomous(name="Omni Wheels Blue Ramp", group="Omni Wheels")
@Disabled
public class Omni_Wheels_Blue_Ramp extends LinearOpMode
{
    DcMotor leftreardrivemotor;
    DcMotor rightreardrivemotor;

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    // This is called when init button is  pressed.
    // It calls the configuration from the phone so the robot knows which motors are which.

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftreardrivemotor = hardwareMap.dcMotor.get("left_rear_drive_motor");
        rightreardrivemotor = hardwareMap.dcMotor.get("right_rear_drive_motor");
        rightreardrivemotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();
// Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for .5 seconds
        leftreardrivemotor.setPower(-0.30);
        rightreardrivemotor.setPower(-0.30);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        // Step 2:  Spin left for 1.3 seconds
        leftreardrivemotor.setPower(TURN_SPEED);
        rightreardrivemotor.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3 )) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        // Step 3:  Drive Forward for .5 Second
        leftreardrivemotor.setPower(-FORWARD_SPEED);
        rightreardrivemotor.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .5)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
        idle();
    }
}