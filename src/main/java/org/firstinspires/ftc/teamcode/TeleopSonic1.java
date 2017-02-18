// simple teleop program that drives bot using controller joysticks in tank mode.
// this code monitors the period and stops when the period is ended.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleopSonic1", group="Exercises")
//@Disabled
public class TeleopSonic1 extends LinearOpMode
{
    DcMotor leftdriveMotor, rightdriveMotor;
    float   leftY, rightY;
    double  right_beacon_Position, left_beacon_Position;

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

        right_beacon_Position = .24;                    // set linear slide to in.
        left_beacon_Position = .24;                  // set linear slide to in.

        while (opModeIsActive())
        {
            leftY = gamepad1.left_stick_y;
            rightY = gamepad1.right_stick_y;

            leftdriveMotor.setPower(Range.clip(leftY, -1.0, 1.0));
            rightdriveMotor.setPower(Range.clip(rightY, -1.0, 1.0));

            telemetry.addData("Mode", "running");
            telemetry.addData("sticks", "  left=" + leftY + "  right=" + rightY);



            telemetry.update();
            idle();
        }
    }
}
