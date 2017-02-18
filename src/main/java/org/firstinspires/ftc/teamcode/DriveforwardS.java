
// simple autonomous program that drives bot forward 2 seconds then ends.
// created by Sara, Sophia, and Genevieve on 12/9/16
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// below is the Annotation that registers this OpMode with the FtcRobotController app.
// @Autonomous classifies the OpMode as autonomous, name is the OpMode title and the
// optional group places the OpMode into the Exercises group.
// uncomment the @Disable annotation to remove the OpMode from the OpMode list.,

@Autonomous(name="Drive Forward", group="Exercises")
//@Disabled
public class DriveforwardS extends LinearOpMode
{
    DcMotor LFdriveMotor;
    DcMotor RFdriveMotor;

    // called when init button is  pressed.

    @Override
    public void runOpMode() throws InterruptedException
    {
        LFdriveMotor = hardwareMap.dcMotor.get("left_drive_motor");
        RFdriveMotor = hardwareMap.dcMotor.get("right_drive_motor");
        RFdriveMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        sleep(10000);

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set both motors to 30% power.

        LFdriveMotor.setPower(-0.30);
        RFdriveMotor.setPower(-0.30);

        sleep(2000);        // wait for 2 seconds.

        // set motor power to zero to stop motors.

        LFdriveMotor.setPower(0.0);
        RFdriveMotor.setPower(0.0);
    }
}