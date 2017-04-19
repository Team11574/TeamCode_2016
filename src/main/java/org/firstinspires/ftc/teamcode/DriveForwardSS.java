package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Sara N on 4/18/2017
 */
@Autonomous(name="DriveForwardSS")
public class DriveForwardSS extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;

    //Called when Init button is pressed.

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("left_front_drive_motor");
        rightMotor = hardwareMap.dcMotor.get("right_front_drive_motor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        //wait for start button

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        //set both motors to 25% power

        leftMotor.setPower(0.25);
        rightMotor.setPower(0.25);

        sleep(2000);  //wait for two seconds

        //set motor power to zero to stop motors

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }
}