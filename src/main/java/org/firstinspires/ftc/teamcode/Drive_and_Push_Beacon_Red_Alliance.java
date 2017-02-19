package org.firstinspires.ftc.teamcode;

/**
 * Created by FTC Team 11574 on 1/28/2017.
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

// set name to show on phone
@Autonomous(name="Drive and Push Beacon Red Alliance", group="Autonomous")
public class Drive_and_Push_Beacon_Red_Alliance extends Mecanum_Wheels_Generic {
    // This is Red because our alliance color is red.
    final private static int COLOR_TEAM = COLOR_RED;

    // The difference in brightness from dark floor to white tape. The floor will be read after
    // init in order to deal with different floor reflectivity.
    final private static double TAPE_ALPHA_DIFFERENCE = 2.0;

    public void push_beacon() {
        drive_distance(DRIVE_FORWARD, 2.0, 0.2);
        drive_until_gt_range(DRIVE_BACKWARD, 5.0, 15.0, 0.2);
    }

    // Check the beacon colors and push the correct button. This assumes that the robot has been
    // aligned on the white line and is positioned with the range sensor 5.0 inches from the
    // beacon.
    public void check_beacons_and_push_button(int strafe_away, int strafe_back) {
        drive_distance(strafe_back, 3.0, 0.2);

        int b1_color = read_beacon_color();
        telemetry.addData("b1_col", COLOR_NAMES[b1_color]);
        telemetry.addData("b2_col", COLOR_NAMES[COLOR_UNKNOWN]);
        telemetry.update();

        drive_distance(strafe_away, 5.0, 0.2);
        int b2_color = read_beacon_color();

        telemetry.addData("b1_col", COLOR_NAMES[b1_color]);
        telemetry.addData("b2_col", COLOR_NAMES[b2_color]);
        telemetry.update();

        if(b1_color == COLOR_TEAM) {
            drive_distance(strafe_back, 8.0, 0.2);
        } else if(b2_color == COLOR_TEAM) {
            drive_distance(strafe_away, 3.0, 0.2);
        }
        push_beacon();

        // Check if the beacon switched to the team's color.
        if(read_beacon_color() != COLOR_TEAM) {
            // We must have mis-pushed the beacon. Wait for 5s rule timeout and just push the
            // beacon again, which will flip it to the other color.
            try {
                Thread.sleep(5000);
            } catch (InterruptedException e) {
                // do nothing.
            }
            push_beacon();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robotInit();

        while(!isStarted() && !isStopRequested()) {
            telemetry.addData("alliance", COLOR_NAMES[COLOR_TEAM]);
            telemetry.addData("range", range.cmUltrasonic());
            telemetry.addData("alpha", Tape_color.alpha());
            telemetry.update();
        }

        if (!isStarted() || isStopRequested())
            return;

        double floor_alpha = Tape_color.alpha();
        double tape_alpha = floor_alpha + TAPE_ALPHA_DIFFERENCE;

        // This next part sets up a function to use strafe right and left for either color.
        int strafe_away, strafe_back;
        if (COLOR_TEAM == COLOR_RED) {
            strafe_away = STRAFE_RIGHT;
            strafe_back = STRAFE_LEFT;
        } else {
            strafe_away = STRAFE_LEFT;
            strafe_back = STRAFE_RIGHT;
        }

        drive_distance(strafe_away, 34.0, 1.0);
        drive_distance(DRIVE_FORWARD, 24.0, 1.0);
        drive_until_gt_alpha(strafe_away, tape_alpha, 28.0, 0.6);
        drive_until_lt_range(DRIVE_FORWARD, 10.0, 24.0, 0.6);
        drive_until_lt_range(DRIVE_FORWARD, 5.0, 10.0, 0.2);

        check_beacons_and_push_button(strafe_away, strafe_back);

        drive_until_gt_range(DRIVE_BACKWARD, 10.0, 10.0, 0.6);

        drive_distance_without_stopping(strafe_away, 36.0, 1.0);
        drive_until_gt_alpha(strafe_away, tape_alpha, 24.0, 0.6);
        drive_until_lt_range(DRIVE_FORWARD, 10.0, 24.0, 0.6);
        drive_until_lt_range(DRIVE_FORWARD, 5.0, 10.0, 0.2);

        check_beacons_and_push_button(strafe_away, strafe_back);

        drive_until_gt_range(DRIVE_BACKWARD, 10.0, 10.0, 0.6);

        drive_distance_without_stopping(strafe_back, 36.0, 1.0);
        drive_until_gt_alpha(strafe_back, tape_alpha, 24.0, 0.6);
        drive_distance(DRIVE_BACKWARD, 36.0, 1.0);


        idle();
    }

    // Stop.
}


