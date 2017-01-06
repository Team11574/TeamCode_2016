// simple autonomous program that drives bot forward 2 seconds then ends.

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
public class DriveForward extends LinearOpMode
{
    DcMotor leftdriveMotor;
    DcMotor rightdriveMotor;

    // called when init button is  pressed.

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftdriveMotor = hardwareMap.dcMotor.get("left_drive_motor");
        rightdriveMotor = hardwareMap.dcMotor.get("right_drive_motor");
        rightdriveMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();
       
       sleep(10000);           // Waits ten seconds and the starts the program to go forward 10 secs then stop.
       
       telemetry.addData("Mode", "running");
        telemetry.update();

        // set both motors to 30% power.

        leftdriveMotor.setPower(-0.30);
        rightdriveMotor.setPower(-0.30);

        sleep(2000);        // wait for 2 seconds.

        // set motor power to zero to stop motors.

        leftdriveMotor.setPower(0.0);
        rightdriveMotor.setPower(0.0);
    }
}
