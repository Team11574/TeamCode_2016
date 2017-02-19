// Copyright (c) 2014, 2015 Qualcomm Technologies Inc

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.util.ElapsedTime;

// import java.text.SimpleDateFormat;
// import java.util.Date;

/**
 * TeleOp Mode
 *Enables control of the robot via the gamepad
 */

@TeleOp(name = "TeleOp_New", group = "Mecanum")
public class Mecanum_Wheels_TeleOp_New extends OpMode {

//    private String startDate;
//    private ElapsedTime runtime = new ElapsedTime();

    DcMotor w1, w2, w3, w4;

    @Override
    public void init() {
        w1 = hardwareMap.dcMotor.get("mFL");
        w2 = hardwareMap.dcMotor.get("mBL");
        w3 = hardwareMap.dcMotor.get("mFR");
        w4 = hardwareMap.dcMotor.get("mBR");

        //w1.setDirection(DcMotor.Direction.REVERSE);
        //w2.setDirection(DcMotor.Direction.REVERSE);
        w3.setDirection(DcMotor.Direction.REVERSE);
        w4.setDirection(DcMotor.Direction.REVERSE);

        w1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        w2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        w3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        w4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gamepad1.setJoystickDeadzone(0.2f);
    }

    /*
       * Code to run when the op mode is first enabled goes here
       * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
       */

     /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */

    @Override
    public void loop() {

        double motorPower = 1.0;
        double slowmotorPower = 0.30;
        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double lt = -gamepad1.left_trigger;
        double rt = -gamepad1.right_trigger;

        if (gamepad1.dpad_left) {
            //move left
            w1.setPower(-motorPower);
            w2.setPower(motorPower);
            w3.setPower(motorPower);
            w4.setPower(-motorPower);

        } else if (gamepad1.dpad_right) {
            //move right
            w1.setPower(motorPower);
            w2.setPower(-motorPower);
            w3.setPower(-motorPower);
            w4.setPower(motorPower);

        } else if (gamepad1.dpad_up) {
            //move forward
            w1.setPower(motorPower);
            w2.setPower(motorPower);
            w3.setPower(motorPower);
            w4.setPower(motorPower);
        } else if (gamepad1.dpad_down) {
            //move backward
            w1.setPower(-motorPower);
            w2.setPower(-motorPower);
            w3.setPower(-motorPower);
            w4.setPower(-motorPower);

        } else {
            //This will turn the robot left/right and power the sticks
            w1.setPower((y - x + lt - rt)*slowmotorPower);
            w2.setPower((y + x + lt - rt)*slowmotorPower);
            w3.setPower((y + x - lt + rt)*slowmotorPower);
            w4.setPower((y - x - lt + rt)*slowmotorPower);

        }
    }
}