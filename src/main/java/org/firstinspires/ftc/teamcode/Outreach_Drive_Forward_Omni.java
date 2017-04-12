// Simple autonomous program that drives bot forward 2 seconds then ends.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Outreach_Drive_Forward_Omni")
public class Outreach_Drive_Forward_Omni extends LinearOpMode
{
    DcMotor leftMotor;
    DcMotor rightMotor;

    // Called when init button is pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("left_rear_drive_motor");
        rightMotor = hardwareMap.dcMotor.get("right_rear_drive_motor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // Wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // Set both motors to 25% power.

        leftMotor.setPower(0.25);
        rightMotor.setPower(0.25);

        sleep(2000); // Wait for 2 seconds.

        // Set motor power to zero to stop motors.

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }
}
