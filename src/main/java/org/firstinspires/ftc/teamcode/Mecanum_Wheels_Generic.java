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
import com.qualcomm.robotcore.hardware.I2cAddr;


public class Mecanum_Wheels_Generic extends LinearOpMode {
    final private static int ENCODER_CPR = 1120;  // Encoder Counters per Revolution
    final private static double GEAR_RATIO = 1.0;   // Gear Ratio - 1:1 - Direct Drive
    final private static double WHEEL_DIAMETER = 3.937;  // Diameter of the wheel in inches
    final private static double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    final private static double STRAFE_SLIPPAGE_FACTOR = 1.08;  // slippage of motors when strafing

    final public static int COLOR_UNKNOWN = 0;
    final public static int COLOR_RED = 1;
    final public static int COLOR_BLUE = 2;
    final public static String[] COLOR_NAMES = {
            "unknown", "red", "blue"
    };

    DcMotor motor[];
    ColorSensor Beacon_color;
    ColorSensor Tape_color;
    ModernRoboticsI2cRangeSensor range;

    final private static int MOTOR_COUNT = 4;
    final private static int mFL = 0;
    final private static int mFR = 1;
    final private static int mBL = 2;
    final private static int mBR = 3;
    final private static String[] MOTOR_NAMES = {
            "mFL", "mFR", "mBL", "mBR"
    };
    final private static DcMotorSimple.Direction MOTOR_DIRECTIONS[] = {
            DcMotor.Direction.FORWARD, // mFL
            DcMotor.Direction.REVERSE, // mFR
            DcMotor.Direction.FORWARD, // mBL
            DcMotor.Direction.REVERSE, // mBR
    };

    final public static int DRIVE_FORWARD  = 0;
    final public static int DRIVE_BACKWARD = 1;
    final public static int TURN_LEFT      = 2;
    final public static int TURN_RIGHT     = 3;
    final public static int STRAFE_LEFT    = 4;
    final public static int STRAFE_RIGHT   = 5;

    final private static double DRIVE_DIRECTIONS[][] = {
            //mFL,  mFR,   mBL,   mBR
            {+1.00, +1.00, +1.00, +1.00}, // DRIVE_FORWARD
            {-1.00, -1.00, -1.00, -1.00}, // DRIVE_BACKWARD
            {+1.00, -1.00, +1.00, -1.00}, // TURN_LEFT
            {-1.00, +1.00, -1.00, +1.00}, // TURN_RIGHT
            {-1.00, +1.00, +1.00, -1.00}, // STRAFE_LEFT
            {+1.00, -0.92, -0.92, +1.04}, // STRAFE_RIGHT
    };

    public void stop_all_motors() {
        for(int i=0; i < MOTOR_COUNT; i++) {
            motor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public int distance_to_count(double distance, double slippage) {
        return((int)(slippage * ENCODER_CPR * (distance / WHEEL_CIRCUMFERENCE) * GEAR_RATIO));
    }

    public boolean one_motor_stopped() {
        for(int i=0; i < MOTOR_COUNT; i++) {
            if(motor[i].isBusy() != true)
                return true;
        }
        return false;
    }

    public void wait_for_one_stop() {
        while(true) {
            if (one_motor_stopped())
                return;
        }
    }

    public boolean all_motors_stopped() {
        for(int i=0; i < MOTOR_COUNT; i++) {
            if(motor[i].isBusy() == true)
                return false;
        }
        return true;
    }

    public void wait_for_all_stop() {
        while(true) {
            if (all_motors_stopped())
                return;
        }
    }

    public boolean one_encoder_satisfied() {
        for(int i=0; i < MOTOR_COUNT; i++) {
            if(motor[i].getCurrentPosition() >= motor[i].getTargetPosition())
                return true;
        }
        return false;
    }

    public void wait_for_one_encoder_satisfied() {
        while(true) {
            if (one_encoder_satisfied())
                return;
        }
    }

    public void set_motor_power(int motor_index, int direction, double power) {
        motor[motor_index].setPower(power);
    }

    public void drive_to_position(int direction, int count, double speed) {
        try {
            for(int i=0; i < MOTOR_COUNT; i++) {
                motor[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor[i].setTargetPosition((int)((double)count * DRIVE_DIRECTIONS[direction][i]));
                set_motor_power(i, direction, 0.20 * speed * DRIVE_DIRECTIONS[direction][i]);
            }
            telemetry.update();

            Thread.sleep(100);
            for(int i=0; i < MOTOR_COUNT; i++) {
                set_motor_power(i, direction, 0.50 * speed * DRIVE_DIRECTIONS[direction][i]);
            }
            telemetry.update();

            Thread.sleep(30);
            for(int i=0; i < MOTOR_COUNT; i++) {
                set_motor_power(i, direction, 1.00 * speed * DRIVE_DIRECTIONS[direction][i]);
            }
            telemetry.update();

        } catch(InterruptedException e) {
        };
    }

    public void drive_distance_start(int direction, double distance, double speed) {
        double slippage = 1.0;
        if(direction == STRAFE_LEFT || direction == STRAFE_RIGHT)
            slippage = STRAFE_SLIPPAGE_FACTOR;
        drive_to_position(direction, distance_to_count(distance, slippage), speed);
    }

    public void drive_distance(int direction, double distance, double speed) {
        double slippage = 1.0;
        if(direction == STRAFE_LEFT || direction == STRAFE_RIGHT)
            slippage = STRAFE_SLIPPAGE_FACTOR;
        drive_to_position(direction, distance_to_count(distance, slippage), speed);
        wait_for_one_stop();
        stop_all_motors();
    }

    /*
        TODO(jeremycole): Tune drive_distance_without_stopping.
        This should ask the motors to drive slightly further and returns once the distance
        requested has been driven, so that the next driving move can be made without stopping.
        This should save 500ms or so per move.
    */

    public void drive_distance_without_stopping(int direction, double distance, double speed) {
        drive_distance_start(direction, distance + 2.0, speed);
        wait_for_one_encoder_satisfied();
    }

    public void drive_until_lt_range(int direction, double desired_range, double max_distance, double speed) {
        drive_distance_start(direction, max_distance, speed);
        while(!one_motor_stopped()) {
            double current_range = (range.cmUltrasonic() / 2.54);
            if(current_range <= desired_range)
                break;
        }
        stop_all_motors();
    }

    public void drive_until_gt_range(int direction, double desired_range, double max_distance, double speed) {
        drive_distance_start(direction, max_distance, speed);
        while(!one_motor_stopped()) {
            double current_range = (range.cmUltrasonic() / 2.54);
            if(current_range >= desired_range)
                break;
        }
        stop_all_motors();
    }

    public void drive_until_gt_alpha(int direction, double desired_alpha, double max_distance, double speed) {
        drive_distance_start(direction, max_distance, speed);
        while(!one_motor_stopped()) {
            double current_alpha = Tape_color.alpha();
            if(current_alpha >= desired_alpha)
                break;
        }
        stop_all_motors();
    }

    public int read_beacon_color() {
        // TODO(jeremycole): Tune color_samples to save time.
        int color_samples = 100;
        double r = 0.0, b = 0.0;
        for(int i=0; i < color_samples; i++) {
            r += Beacon_color.red();
            b += Beacon_color.blue();
            try {
                Thread.sleep(2);
            } catch (InterruptedException e) {
                return COLOR_UNKNOWN;
            }
        }
        r /= color_samples;
        b /= color_samples;

        if(b > r)
            return COLOR_BLUE;
        else if(r > b)
            return COLOR_RED;

        return COLOR_UNKNOWN;
    }

    public void robotInit() {
        motor = new DcMotor[MOTOR_COUNT];

        for(int i=0; i < MOTOR_COUNT; i++) {
            motor[i] = hardwareMap.dcMotor.get(MOTOR_NAMES[i]);
            motor[i].setDirection(MOTOR_DIRECTIONS[i]);
        }

        Beacon_color = hardwareMap.colorSensor.get("Beacon_color");
        Beacon_color.setI2cAddress(I2cAddr.create8bit(0x3c));
        Beacon_color.enableLed(false);

        Tape_color = hardwareMap.colorSensor.get("Tape_color");
        Tape_color.setI2cAddress(I2cAddr.create8bit(0x3a));
        Tape_color.enableLed(true);

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        stop_all_motors();
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}


