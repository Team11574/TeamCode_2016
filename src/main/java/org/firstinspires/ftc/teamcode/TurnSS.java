package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Sara N on 4/19/2017.
 */

@Autonomous (name = "TurnSS")
public class TurnSS extends LinearOpMode {

    DcMotor LeftFrontMotor;
    DcMotor RightFrontMotor;
    DcMotor LeftRearMotor;
    DcMotor RightRearMotor;

    //Called when Init button is pressed.
    // Naming Motors
    @Override
    public void runOpMode() throws InterruptedException {
        LeftFrontMotor = hardwareMap.dcMotor.get("mFL");
        RightFrontMotor = hardwareMap.dcMotor.get("mFR");
        LeftRearMotor = hardwareMap.dcMotor.get("mBL");
        RightRearMotor = hardwareMap.dcMotor.get("mBR");
        LeftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftRearMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        //wait for start button

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        LeftFrontMotor.setPower(0.25);
        RightFrontMotor.setPower(0.25);
        LeftRearMotor.setPower(0.25);
        RightRearMotor.setPower(0.25);

        sleep(3000);

        LeftFrontMotor.setPower(0.0);
        RightFrontMotor.setPower(0.0);
        LeftRearMotor.setPower(0.0);
        RightRearMotor.setPower(0.0);

        sleep(3000);

        //Turn Left
        LeftFrontMotor.setPower(-0.25);
        RightFrontMotor.setPower(0.25);
        LeftRearMotor.setPower(-0.25);
        RightRearMotor.setPower(0.25);

        sleep(900);

        LeftFrontMotor.setPower(0.0);
        RightFrontMotor.setPower(0.0);
        LeftRearMotor.setPower(0.0);
        RightRearMotor.setPower(0.0);

        LeftFrontMotor.setPower(0.25);
        RightFrontMotor.setPower(0.25);
        LeftRearMotor.setPower(0.25);
        RightRearMotor.setPower(0.25);

        sleep(3000);

        LeftFrontMotor.setPower(0.0);
        RightFrontMotor.setPower(0.0);
        LeftRearMotor.setPower(0.0);
        RightRearMotor.setPower(0.0);

    }
}
