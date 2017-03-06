/*
 * Copyright 2017, FTC Team 11574.
 *
 * A generic framework for initializing and controlling the Mecanum wheel based
 * robot as built by Team 11574. This class should not be used directly, but
 * should be sub-classed for each actual program desired.
 */

package org.firstinspires.ftc.teamcode;

import java.util.Locale;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

@SuppressWarnings({"unused", "WeakerAccess"})
public class Mecanum_Wheels_Generic extends LinearOpMode {
    // Tag to log messages to the Android log with.
    final public static String LOG_TAG = "FTC";

    public void info(String msg) {
        Log.i(LOG_TAG, msg);
    }

    // An exception to throw to indicate that "Stop" was pressed (or fired
    // automatically due to timer expiration). The robot should stop
    // immediately to avoid penalty points or crashing.
    public class StopImmediatelyException extends RuntimeException {
        public StopImmediatelyException() { super(); }
    }

    // Number of encoder counts per wheel revolution.
    final private static int ENCODER_CPR = 1120;

    // Gear Ratio; 1:1 - Direct Drive.
    final private static double GEAR_RATIO = 1.0;

    // Diameter of the wheels, in inches.
    final private static double WHEEL_DIAMETER = 3.937;

    // Circumference of the wheels, in inches.
    final private static double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    // Conversion factor from centimeters to inches.
    final private static double CM_TO_INCH = 2.54;

    // The factor of slippage of wheels when strafing. Measured to be about 8%.
    final private static double STRAFE_SLIPPAGE_FACTOR = 1.08;

    // The number of color samples to read.
    final private static int COLOR_SAMPLE_COUNT = 20;

    // The amount of time to sleep in between color samples.
    final private static int COLOR_SAMPLE_SLEEP = 1;

    // The difference in brightness from dark floor mat to white tape.
    final public static double TAPE_ALPHA_DIFFERENCE = 10.0;

    // Each of the colors we need to know about, only red and blue.
    public enum AllianceColor {
        Unknown,
        Red,
        Blue,
    }

    // Each of the motors on the robot.
    // TODO: This could probably be an enum.
    final private static int MOTOR_COUNT = 4;
    final private static int mFL = 0;
    final private static int mFR = 1;
    final private static int mBL = 2;
    final private static int mBR = 3;
    final private static String[] MOTOR_NAMES = {
            "mFL", "mFR", "mBL", "mBR"
    };

    // The direction that each motor on the robot is oriented. The right-side motors are mounted
    // backwards relative to the left side ones.
    final private static DcMotorSimple.Direction MOTOR_DIRECTIONS[] = {
            DcMotor.Direction.FORWARD, // mFL
            DcMotor.Direction.REVERSE, // mFR
            DcMotor.Direction.FORWARD, // mBL
            DcMotor.Direction.REVERSE, // mBR
    };

    // Each driving direction supported by driving functions.
    // TODO: This could probably be an enum.
    final public static int DRIVE_FORWARD  = 0;
    final public static int DRIVE_BACKWARD = 1;
    final public static int TURN_LEFT      = 2;
    final public static int TURN_RIGHT     = 3;
    final public static int STRAFE_LEFT    = 4;
    final public static int STRAFE_RIGHT   = 5;

    // This serves as both a matrix of the motor directions needed to drive in each direction
    // which is supported (via the sign, + or -) as well as a correction table to allow driving
    // straighter in each supported direction. Reduce the values here to slow the motors slightly
    // or increase the values to speed them up.
    final private static double DRIVE_DIRECTIONS[][] = {
            // mFL,  mFR,   mBL,   mBR
            { +1.00, +1.00, +1.00, +1.00 }, // DRIVE_FORWARD
            { -1.00, -1.00, -1.00, -1.00 }, // DRIVE_BACKWARD
            { -1.00, +1.00, -1.00, +1.00 }, // TURN_LEFT
            { +1.00, -1.00, +1.00, -1.00 }, // TURN_RIGHT
            { -1.00, +1.00, +1.00, -1.00 }, // STRAFE_LEFT
            { +1.00, -0.92, -0.92, +1.04 }, // STRAFE_RIGHT
    };

    // An array of DcMotors to represent all of the motors.
    DcMotor motor[];

    // The front-facing beacon color sensor.
    ColorSensor Beacon_color;

    // The bottom-mounted floor/tape color sensor.
    ColorSensor Tape_color;

    // The front-facing range (ultrasonic + optical) sensor.
    ModernRoboticsI2cRangeSensor range;

    // The chassis-mounted red/blue alliance switch for autonomous mode.
    DigitalChannel alliance_switch;

    // The gyro sensor.
    GyroSensor gyro;

    // Convert a distance, in inches, into an encoder count, including a wheel slippage correction
    // factor.
    public int distance_to_count(double distance, double slippage) {
        return((int)(slippage * ENCODER_CPR * (distance / WHEEL_CIRCUMFERENCE) * GEAR_RATIO));
    }

    // Convert from inches per second to encoder counts per second.
    public int speed_ips_to_cps(double speed_ips) {
        // Counts per inch of actual wheel movement.
        double cpi = ENCODER_CPR / WHEEL_CIRCUMFERENCE / GEAR_RATIO;

        // Counts per second at speed_ips.
        double cps = cpi * speed_ips;

        return (int)cps;
    }

    // Convert from miles per hour to encoder counts per second.
    public int speed_mph_to_cps(double speed_mph) {
        // Convert from *miles* per hour to *inches* per hour.
        double speed_iph = speed_mph * (5280.0 * 12.0) ;

        // Convert from inches per *hour* to inches per *second*.
        double speed_ips = speed_iph * (60.0 * 60.0);

        // Call the above function to convert to counts per second.
        return speed_ips_to_cps(speed_ips);
    }

    // If stop was requested, throw a StopImmediatelyException which will be
    // caught by runOpMode to stop the robot immediately.
    public boolean should_keep_running() {
        if(isStarted() && isStopRequested())
            throw new StopImmediatelyException();
        return true;
    }

    // Stop all motors immediately.
    public void stop_all_motors() {
        for(int i=0; i < MOTOR_COUNT; i++) {
            if(motor[i].getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                motor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }

    // Set the power of a given motor.
    public void set_motor_power(int motor_index, int direction, double power) {
        motor[motor_index].setPower(power);
    }

    // Check if at least one motor is stopped (because it has reached its desired position).
    public boolean one_motor_stopped() {
        for(int i=0; i < MOTOR_COUNT; i++) {
            if(!motor[i].isBusy())
                return true;
        }
        return false;
    }

    // Wait for at least one motor to stop.
    public void wait_for_one_stop() {
        while(should_keep_running()) {
            if (one_motor_stopped())
                return;
        }
    }

    // Check if all motors are stopped (because they have all reached their desired positions).
    public boolean all_motors_stopped() {
        for(int i=0; i < MOTOR_COUNT; i++) {
            if(motor[i].isBusy())
                return false;
        }
        return true;
    }

    // Wait for all motors to stop.
    public void wait_for_all_stop() {
        while(should_keep_running()) {
            if (all_motors_stopped())
                return;
        }
    }

    // Check if at least one encoder has reached its desired position.
    public boolean one_encoder_satisfied() {
        for(int i=0; i < MOTOR_COUNT; i++) {
            // We're advancing forwards.
            if(motor[i].getPower() > 0.0 && motor[i].getCurrentPosition() >= motor[i].getTargetPosition())
                return true;

            // We're advancing backwards.
            if(motor[i].getPower() < 0.0 && motor[i].getCurrentPosition() <= motor[i].getTargetPosition())
                return true;
        }
        return false;
    }

    // Wait for at least one encoder to have reached its desired position.
    public void wait_for_one_encoder_satisfied() {
        while(should_keep_running()) {
            if (one_encoder_satisfied())
                return;
        }
    }

    // Drive in a given direction at a given speed until at least one encoder reaches the
    // given count.
    public void drive_to_position(int direction, int count, double speed) {
        info(String.format(Locale.US, "Called drive_to_position: direction=%d, count=%d, speed=%.2f",
                direction, count, speed));

        for(int i=0; i < MOTOR_COUNT; i++) {
            if(motor[i].getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                motor[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            motor[i].setTargetPosition((int)((double)count * DRIVE_DIRECTIONS[direction][i]));
            set_motor_power(i, direction, 0.20 * speed * DRIVE_DIRECTIONS[direction][i]);
        }

        sleep(100);
        for(int i=0; i < MOTOR_COUNT; i++) {
            set_motor_power(i, direction, 0.50 * speed * DRIVE_DIRECTIONS[direction][i]);
        }

        sleep(30);
        for(int i=0; i < MOTOR_COUNT; i++) {
            set_motor_power(i, direction, 1.00 * speed * DRIVE_DIRECTIONS[direction][i]);
        }
    }

    // Drive at a constant speed
    public void drive_constant_speed(int direction, double speed) {
        info(String.format(Locale.US, "Called drive_constant_speed: direction=%d, speed=%.2f",
                direction, speed));

        // TODO: Possibly implement the speed ramp-up for this mode, too.
        for(int i=0; i < MOTOR_COUNT; i++) {
            if(motor[i].getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                motor[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            set_motor_power(i, direction, speed * DRIVE_DIRECTIONS[direction][i]);
        }
    }

    // Start driving in a given direction at a given speed for a maximum of the given distance,
    // but return immediately rather than waiting to reach the position.
    public void drive_distance_start(int direction, double distance, double speed) {
        double slippage = 1.0;
        if(direction == STRAFE_LEFT || direction == STRAFE_RIGHT)
            slippage = STRAFE_SLIPPAGE_FACTOR;
        for(int i=0; i < MOTOR_COUNT; i++) {
            int new_position = motor[i].getCurrentPosition();
            new_position += DRIVE_DIRECTIONS[direction][i] * distance_to_count(distance, slippage);
            // In constant-speed RUN_USING_ENCODER mode, the setTargetPosition is advisory
            // only and we'll check it ourselves against getCurrentPosition.
            motor[i].setTargetPosition(new_position);
        }
        drive_constant_speed(direction, speed);
    }

    // Drive in a given direction at a given speed until reaching the given distance.
    public void drive_distance(int direction, double distance, double speed) {
        drive_distance_start(direction, distance, speed);
        wait_for_one_encoder_satisfied();
    }


    // Drive forwards towards a wall at a given speed until reaching the desired range.
    // As a safety measure, stop after max_distance to avoid crashing too badly.
    public void drive_until_lt_range(double desired_range, double max_distance, double speed) {
        drive_distance_start(DRIVE_FORWARD, max_distance, speed);
        while(!one_motor_stopped() && should_keep_running()) {
            double current_range = (range.cmUltrasonic() / CM_TO_INCH);
            if(current_range <= desired_range)
                break;
        }
    }

    // Drive backwards away from a wall at a given speed until reaching the desired range.
    // As a safety measure, stop after max_distance to avoid crashing too badly.
    public void drive_until_gt_range(double desired_range, double max_distance, double speed) {
        drive_distance_start(DRIVE_BACKWARD, max_distance, speed);
        while(!one_motor_stopped() && should_keep_running()) {
            double current_range = (range.cmUltrasonic() / CM_TO_INCH);
            if(current_range >= desired_range)
                break;
        }
    }

    // Drive in a given direction until the bottom-mounted color sensor sees an alpha greater than
    // the given desired_alpha (such as a tape line). As a safety measure, stop after max_distance
    // regardless to avoid crashing too badly.
    public void drive_until_gt_alpha(int direction,
                                     double desired_alpha,
                                     double max_distance,
                                     double speed) {
        drive_distance_start(direction, max_distance, speed);
        while(!one_motor_stopped()) {
            double current_alpha = Tape_color.alpha();
            if(current_alpha >= desired_alpha)
                break;
        }
    }

    // Drive forwards slowly, push the button, and then back up again.
    public void push_beacon() {
        drive_distance(DRIVE_FORWARD, 2.0, 0.2);
        drive_until_gt_range(5.0, 15.0, 0.2);
    }

    // Check the beacon colors and push the correct button. This assumes that the robot has been
    // aligned on the white line and is positioned with the range sensor 5.0 inches from the
    // beacon.
    public void check_beacons_and_push_button(String beacon_name,
                                              AllianceColor color_alliance,
                                              double tape_alpha,
                                              int strafe_away,
                                              int strafe_back) {
        String log_prefix = "[" + beacon_name + "] ";

        info(log_prefix +
                "Checking beacon for " + color_alliance + " alliance.");

        // Align to the first (near) side of the beacon, and read its color.
        info(log_prefix + "Reading near side color...");
        drive_distance(strafe_back, 3.0, 0.2);
        stop_all_motors();
        AllianceColor near_color = read_beacon_color();

        info(log_prefix + "Near side color: " + near_color);

        // Send the color telemetry data for debugging.
        telemetry.addData(beacon_name, near_color + "/" + AllianceColor.Unknown);
        telemetry.update();

        // Align to the second (far) side of the beacon, and read its color.
        info(log_prefix + "Reading far side color...");
        drive_until_gt_alpha(strafe_away, tape_alpha, 5.0, 0.2);
        drive_distance(strafe_away, 3.0, 0.2);
        stop_all_motors();
        AllianceColor far_color = read_beacon_color();

        info(log_prefix + "Far side color: " + far_color);

        // Send the color telemetry data for debugging.
        telemetry.addData(beacon_name, near_color + "/" + far_color);
        telemetry.update();

        if(near_color == color_alliance) {
            info(log_prefix + "Pushing near side.");
            drive_until_gt_alpha(strafe_back, tape_alpha, 5.0, 0.2);
            drive_distance(strafe_back, 5.0, 0.2);
            stop_all_motors();
        } else if(far_color == color_alliance) {
            info(log_prefix + "Pushing far side.");
            drive_distance(strafe_away, 2.0, 0.2);
            stop_all_motors();
        } else {
            // Don't push either button...
            info(log_prefix + "Unable to determine which side to press!");
            return;
        }
        push_beacon();
        stop_all_motors();

        // Check if the beacon switched to the alliance's color, if not, we'll wait the 5s rule
        // timeout and re-push it, which should flip the color. If we leave the beacon on
        // the wrong color, it is 30 points for the other alliance. It doesn't matter which button
        // we push at this point, so no need to reposition.
        AllianceColor checked_beacon_color = read_beacon_color();
        if(checked_beacon_color != color_alliance) {
            info(log_prefix + "Appears to be mis-pushed; got color " +
                    checked_beacon_color + "; waiting to re-push!");

            sleep(5000);

            info(log_prefix + "Re-pushing beacon. Hopefully fixed!");
            drive_distance(DRIVE_FORWARD, 1.0, 0.2);
            push_beacon();
            stop_all_motors();
        }
    }

    // Figure out the color (red or blue) that the beacon color sensor is seeing. In order to
    // get a somewhat more accurate color reading, multiple readings are taken and averaged.
    public AllianceColor read_beacon_color() {
        // Accumulator variables for red and blue values.
        double r = 0.0, b = 0.0;

        // Loop reading from the color sensor accumulating the readings into r and b.
        for(int i=0; i < COLOR_SAMPLE_COUNT; i++) {
            // Accumulate a sum for each color.
            r += Beacon_color.red();
            b += Beacon_color.blue();

            sleep(COLOR_SAMPLE_SLEEP);
        }

        // Produce an average red and blue value by dividing the accumulated sum in the r and b
        // variables by the number of samples taken.
        r /= COLOR_SAMPLE_COUNT;
        b /= COLOR_SAMPLE_COUNT;

        info(String.format(Locale.US, "Average color values: red=%.2f, blue=%.2f", r, b));

        if(b > r)       // The sensor saw more blue than red.
            return AllianceColor.Blue;
        else if(r > b)  // The sensor saw more red than blue.
            return AllianceColor.Red;

        // Most likely, we didn't get blue *or* red data, or they were the same value, so it is
        // not possible to tell the color.
        return AllianceColor.Unknown;
    }

    public AllianceColor check_alliance() {
        if (alliance_switch.getState())
            return AllianceColor.Red;
        else
            return AllianceColor.Blue;
    }

    // Initialize the robot and all its sensors.
    public void robotInit() {
        info("Initialization starting...");

        // Initialize all motors in a loop.
        info("* Initializing motors...");
        motor = new DcMotor[MOTOR_COUNT];
        for(int i=0; i < MOTOR_COUNT; i++) {
            motor[i] = hardwareMap.dcMotor.get(MOTOR_NAMES[i]);
            motor[i].setDirection(MOTOR_DIRECTIONS[i]);
            motor[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor[i].setPower(0.0);
            // TODO: We may want to tune the maximum speed a bit.
            motor[i].setMaxSpeed(speed_mph_to_cps(3.0));
        }

        // Make sure everything starts out stopped.
        stop_all_motors();

        // Initialize the front-facing beacon color sensor.
        info("* Initializing beacon color sensor...");
        Beacon_color = hardwareMap.colorSensor.get("Beacon_color");
        Beacon_color.setI2cAddress(I2cAddr.create8bit(0x3c));
        Beacon_color.enableLed(false);

        // Initialize the bottom-mounted tape color sensor.
        info("* Initializing tape color sensor...");
        Tape_color = hardwareMap.colorSensor.get("Tape_color");
        Tape_color.setI2cAddress(I2cAddr.create8bit(0x3a));
        Tape_color.enableLed(true);

        // Initialize the front-facing range sensor.
        info("* Initializing range sensor...");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        // Initialize the alliance switch.
        info("* Initializing alliance switch...");
        alliance_switch = hardwareMap.digitalChannel.get("alliance_switch");
        alliance_switch.setMode(DigitalChannelController.Mode.INPUT);

        // Initialize the gyro.
        info("* Initializing gyro sensor...");
        gyro = hardwareMap.gyroSensor.get("gyro");
        // It takes several seconds to calibrate, so show a message on the telemetry data.
        telemetry.addData(">", "Calibrating the gyro sensor...");
        telemetry.update();
        gyro.calibrate();
        while(gyro.isCalibrating() && !isStopRequested());

        info("Initialization complete.");
    }

    public void robotWaitForStart() {
        while(!isStarted() && !isStopRequested()) {
            // Send some basic sensor data telemetry for confirmation and testing.
            telemetry.addData("1. alliance", check_alliance());
            telemetry.addData("2. range", range.cmUltrasonic());
            telemetry.addData("3. tape_alpha", Tape_color.alpha());
            telemetry.addData("4. beacon_blue", Beacon_color.blue());
            telemetry.addData("5. beacon_red", Beacon_color.red());
            telemetry.addData("6. gyro", gyro.getHeading());
            telemetry.update();
        }
    }

    public void robotRun() {
        // Do nothing. This should be overridden in a subclass.
    }

    @Override
    public void runOpMode() {
        // The entire execution of the OpMode is inside a try block so that
        // any exceptions generated can be caught and a stack trace logged.
        try {
            info("Calling robotInit()...");
            robotInit();

            info("Calling robotWaitForStart()...");
            robotWaitForStart();

            // Exit immediately if stop was pressed, otherwise continue.
            if (!isStarted() || isStopRequested()) {
                info("Stop requested!");
                return;
            }

            info("Calling robotRun()...");
            robotRun();

            // Stop just in case robotRun did not.
            stop_all_motors();
        } catch (Throwable t) {
            // Expected due to timer expiration or "Stop" button pressed.
            if (t instanceof StopImmediatelyException) {
                info("Stop requested!");
                stop_all_motors();
                return;
            }

            // Unexpected exception; log it, and then re-throw a RuntimeException.
            Log.e(LOG_TAG, "Exception caught!", t);

            if (t instanceof RuntimeException) {
                throw (RuntimeException) t;
            }

            throw new RuntimeException(t);
        }
    }

}


