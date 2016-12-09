// simple teleop program that drives bot using controller joysticks in tank mode.
// this code monitors the period and stops when the period is ended.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Drive Gripper", group="Exercises")
//@Disabled
public class DriveWithGripper extends LinearOpMode
{
    DcMotor leftdriveMotor, rightdriveMotor;
    Servo   rightbeacon, leftbeacon;
    float   leftY, rightY;
    double  beaconPosition, beaconPosition;

    // called when init button is  pressed.
     @Override
    public void runOpMode() throws InterruptedException
    {
        leftdriveMotor = hardwareMap.dcMotor.get("left_drive_motor");
        rightdriveMotor = hardwareMap.dcMotor.get("right_drive_motor");
        rightdriveMotor.setDirection(DcMotor.Direction.REVERSE);

       rightbeacon = hardwareMap.servo.get("right_beacon");
       leftbeacon = hardwareMap.servo.get("left_beacon");

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        rightbeacon = .24;                    // set linear slide to in.
        leftbeacon = .24;                  // set linear slide to in.

        while (opModeIsActive())
        {
            leftY = gamepad1.left_stick_y;
            rightY = gamepad1.right_stick_y;

            leftdriveMotor.setPower(Range.clip(leftY, -1.0, 1.0));
            rightdriveMotor.setPower(Range.clip(rightY, -1.0, 1.0));

            telemetry.addData("Mode", "running");
            telemetry.addData("sticks", "  left=" + leftY + "  right=" + rightY);

            // check the gamepad buttons and if pressed, increment the appropriate position
            // variable to change the servo location.

            // move beacon pusher in on A button if not already at lowest position.
            if (gamepad2.a && rightbeacon > Servo.MIN_POSITION) beaconPosition -= .01;

            // move beacon pusher out on B button if not already at the highest position.
            if (gamepad2.b && rightbeacon < Servo.MAX_POSITION) beaconPosition += .01;

            // move beacon pusher in on X button if not already at most open position.
            if (gamepad2.x && leftbeacon < Servo.MAX_POSITION) beaconPosition = gripPosition + .01;

            // move beacon pusher out on Y button if not already at the closed position.
            if (gamepad2.y && leftbeacon > Servo.MIN_POSITION) beaconPosition = gripPosition - .01;

            // set the servo position values as we have computed them.
            armServo.setPosition(Range.clip(armPosition, Servo.MIN_POSITION, Servo.MAX_POSITION));
            gripServo.setPosition(Range.clip(gripPosition, Servo.MIN_POSITION, Servo.MAX_POSITION));

            telemetry.addData("rightbeacon", "position=" + beaconPosition + "  actual=" + rightbeacon.getPosition());
            telemetry.addData("leftbeacon", "position=" + beaconPosition + "    actual=" + leftbeacon.getPosition());
            //telemetry.addData("rightbeacon", String.format("position=%.2f  actual=%.2f", beaconPosition, rightbeacon.getPosition()));
            //telemetry.addData("leftbeacon", String.format("position=%.2f  actual=%.2f", beaconPosition, leftbeacon.getPosition()));
          

            telemetry.update();

            idle();
        }
    }
}
