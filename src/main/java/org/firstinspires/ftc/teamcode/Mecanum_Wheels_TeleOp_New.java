/*
 * Copyright 2017, FTC Team 11574.
 *
 * A TeleOp program to allow control of the robot via a USB Gamepad.
 *
 * This program was initially based on code that was:
 *   Copyright (c) 2014, 2015 Qualcomm Technologies Inc
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp_New", group = "Mecanum")
@SuppressWarnings({"unused", "WeakerAccess"})
public class Mecanum_Wheels_TeleOp_New extends OpMode {
    DcMotor mFL, mBL, mFR, mBR;

    @Override
    public void init() {
        mFL = hardwareMap.dcMotor.get("mFL");
        mBL = hardwareMap.dcMotor.get("mBL");
        mFR = hardwareMap.dcMotor.get("mFR");
        mBR = hardwareMap.dcMotor.get("mBR");

        mFL.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.FORWARD);
        mFR.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.REVERSE);

        mFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gamepad1.setJoystickDeadzone(0.2f);
    }

    @Override
    public void loop() {
        double motorPower = 1.0;
        double slowMotorPower = 0.30;
        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double lt = -gamepad1.left_trigger;
        double rt = -gamepad1.right_trigger;

        if (gamepad1.dpad_left) {
            // Strafe left.
            mFL.setPower(-motorPower);
            mBL.setPower(motorPower);
            mFR.setPower(motorPower);
            mBR.setPower(-motorPower);
        } else if (gamepad1.dpad_right) {
            // Strafe right.
            mFL.setPower(motorPower);
            mBL.setPower(-motorPower);
            mFR.setPower(-motorPower);
            mBR.setPower(motorPower);
        } else if (gamepad1.dpad_up) {
            // Drive forward.
            mFL.setPower(motorPower);
            mBL.setPower(motorPower);
            mFR.setPower(motorPower);
            mBR.setPower(motorPower);
        } else if (gamepad1.dpad_down) {
            // Drive backward.
            mFL.setPower(-motorPower);
            mBL.setPower(-motorPower);
            mFR.setPower(-motorPower);
            mBR.setPower(-motorPower);
        } else {
            // Allow use of the analog left/right sticks to control the robot in differential
            // steering (tank driving) mode. Allow use of the left and right trigger buttons
            // to rotate the robot.
            mFL.setPower((y - x + lt - rt) * slowMotorPower);
            mBL.setPower((y + x + lt - rt) * slowMotorPower);
            mFR.setPower((y + x - lt + rt) * slowMotorPower);
            mBR.setPower((y - x - lt + rt) * slowMotorPower);
        }
    }
}