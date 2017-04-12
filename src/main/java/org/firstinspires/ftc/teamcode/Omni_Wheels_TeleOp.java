// Basic TeleOp program that drives bot using controller joysticks in tank mode.
// This code monitors the period and stops when the period is ended.
// This uses 2 drive motors and maps them to the joy stick button.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Omni Wheels TeleOp", group="Omni_Wheels")
//@Disabled
public class Omni_Wheels_TeleOp extends LinearOpMode
{
    DcMotor leftreardrivemotor, rightreardrivemotor;
    float   leftY, rightY;

    // This is called when init button is  pressed.
    // This calls the configuration file from the phone
    //  Configuration on phone must match the Parenthesis names.

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftreardrivemotor = hardwareMap.dcMotor.get("left_rear_drive_motor");
        rightreardrivemotor = hardwareMap.dcMotor.get("right_rear_drive_motor");
        rightreardrivemotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // Wait for start button to be pressed on phone.
        //  This part maps the motors to the joy stick button.

        waitForStart();

        while (opModeIsActive())
        {
            leftY = gamepad1.left_stick_y;
            rightY = gamepad1.right_stick_y;

            leftreardrivemotor.setPower(Range.clip(leftY, -1.0, 1.0));
            rightreardrivemotor.setPower(Range.clip(rightY, -1.0, 1.0));

            telemetry.addData("Mode", "running");
            telemetry.addData("sticks", "  left=" + leftY + "  right=" + rightY);

            telemetry.update();

            idle();
        }
    }
}