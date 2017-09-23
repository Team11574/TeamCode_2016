package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Omni_Four_Wheel", group="Omni_Wheels")
//@Disabled
public class Omni_Four_Wheel extends LinearOpMode {
    DcMotor left_rear_drive_motor, right_rear_drive_motor;
    DcMotor left_front_drive_motor, right_front_drive_motor;
    float   leftY, rightY;

// called when init button is  pressed.
@Override
public void runOpMode() throws InterruptedException {
        left_rear_drive_motor = hardwareMap.dcMotor.get("left_rear_drive_motor");
        right_rear_drive_motor = hardwareMap.dcMotor.get("right_rear_drive_motor");
        left_front_drive_motor = hardwareMap.dcMotor.get ("left_front_drive_motor");
        right_front_drive_motor = hardwareMap.dcMotor.get ("right_front_drive_motor");
        right_rear_drive_motor.setDirection(DcMotor.Direction.REVERSE);
        right_front_drive_motor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        while (opModeIsActive())
        {
        leftY = gamepad1.left_stick_y;
        rightY = gamepad1.right_stick_y;

        left_rear_drive_motor.setPower(Range.clip(leftY, -1.0, 1.0));
        right_rear_drive_motor.setPower(Range.clip(rightY, -1.0, 1.0));
        left_front_drive_motor.setPower(Range.clip(leftY, -1.0, 1.0));
        right_front_drive_motor.setPower(Range.clip(rightY, -1.0, 1.0));

        telemetry.addData("Mode", "running");
        telemetry.addData("sticks", "  left=" + leftY + "  right=" + rightY);

        telemetry.update();

        idle();
        }
        }
        }
