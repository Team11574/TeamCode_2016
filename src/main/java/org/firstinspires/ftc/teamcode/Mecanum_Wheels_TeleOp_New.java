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

        double aPower = 0.4;//added for 40% power with A Button
        double xPower = 0.2;//added for 20% power with X Button
        double motorPower = 1.0;
        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double lt = -gamepad1.left_trigger;
        double rt = -gamepad1.right_trigger;

        //40% Speed Forward when pushing Button A on Gamepad
        if(gamepad1.a){
            motorPower = aPower;
        }
        //20% Speed Forward when pushing Button X on Gamepad
        if(gamepad1.x) {
            motorPower = xPower;}

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
            //don't change this. This will turn the robot left/right
            w1.setPower(y - x + lt - rt);
            w2.setPower(y + x + lt - rt);
            w3.setPower(y + x - lt + rt);
            w4.setPower(y - x - lt + rt);

        }
    }
}