package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Sara N on 4/18/2017
 */
@Autonomous(name="DriveForwardMecanumSS")
public class DriveForwardMecanumSS extends LinearOpMode {
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    DcMotor leftRearMotor;
    DcMotor rightRearMotor;

    //Called when Init button is pressed.

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontMotor = hardwareMap.dcMotor.get("mFL");
        rightFrontMotor = hardwareMap.dcMotor.get("mFR");
        leftRearMotor = hardwareMap.dcMotor.get("mBL");
        rightRearMotor = hardwareMap.dcMotor.get("mBR");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        //wait for start button

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        //set both motors to 25% power

        leftFrontMotor.setPower(0.25);
        rightFrontMotor.setPower(0.25);
        leftRearMotor.setPower(0.25);
        rightRearMotor.setPower(0.25);

        sleep(2000);  //wait for two seconds

        //set motor power to zero to stop motors

        leftFrontMotor.setPower(0.0);
        rightFrontMotor.setPower(0.0);
        leftRearMotor.setPower(0.0);
        rightRearMotor.setPower(0.0);
    }
}

