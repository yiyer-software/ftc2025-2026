package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp MANUAL", group="Competition")
public class TeleOpMANUAL extends LinearOpMode {

    private static final Alliance SELECTED_ALLIANCE = Alliance.BLUE;
    public enum Alliance { RED, BLUE }

    private static final String MOTOR_FRONT_LEFT = "frontLeft", MOTOR_FRONT_RIGHT = "frontRight";
    private static final String MOTOR_BACK_LEFT = "backLeft", MOTOR_BACK_RIGHT = "backRight";
    private static final String MOTOR_LAUNCHER_LEFT = "launcherLeft", MOTOR_LAUNCHER_RIGHT = "launcherRight";
    private static final String MOTOR_INTAKE = "intakeMotor";
    private static final String SERVO_LIFT_LEFT = "liftServoLeft", SERVO_LIFT_RIGHT = "liftServoRight";

    private static final double LIFT_DOWN_POS = 0.0, LIFT_UP_POS = 0.85;

    private static final double FLYWHEEL_LAUNCH_POWER = 0.90, FLYWHEEL_IDLE_POWER = 0.30, INTAKE_POWER = 0.75;
    private static final double DRIVE_SPEED_NORMAL = 1.0, DRIVE_SPEED_SLOW = 0.5, DRIVE_DEADZONE = 0.05;

    private static final long FLYWHEEL_SPINUP_TIME = 1000;
    private static final long LAUNCH_HOLD_TIME = 300;

    private DcMotor frontLeft, frontRight, backLeft, backRight, launcherLeft, launcherRight, intakeMotor;
    private Servo liftServoLeft, liftServoRight;

    private boolean lastA, lastB, lastX, lastY, lastLeftBumper, lastRightBumper;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean liftIsUp = false, precisionMode = false;

    private String[] currentPattern = {"-", "-", "-"};

    @Override
    public void runOpMode() {
        initializeHardware();
        displayStartupTelemetry();
        waitForStart();
        runtime.reset();
        
        while (opModeIsActive()) {
            updateButtonStates();
            runManualMode();
            updateTelemetry();
            sleep(20);
        }
        cleanup();
    }

    private void initializeHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, MOTOR_FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, MOTOR_FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, MOTOR_BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, MOTOR_BACK_RIGHT);
        launcherLeft = hardwareMap.get(DcMotor.class, MOTOR_LAUNCHER_LEFT);
        launcherRight = hardwareMap.get(DcMotor.class, MOTOR_LAUNCHER_RIGHT);
        intakeMotor = hardwareMap.get(DcMotor.class, MOTOR_INTAKE);

        liftServoLeft = hardwareMap.get(Servo.class, SERVO_LIFT_LEFT);
        liftServoRight = hardwareMap.get(Servo.class, SERVO_LIFT_RIGHT);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(DcMotor.Direction.FORWARD);
        launcherRight.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        setDriveBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setLiftPosition(liftIsUp);
    }

    private void displayStartupTelemetry() {
        telemetry.addData("Status", "Hardware Initialized");
        telemetry.addData("Mode", "MANUAL CONTROL ONLY");
        telemetry.update();
    }

    private void updateButtonStates() {
        boolean currentA = gamepad1.a, currentB = gamepad1.b, currentX = gamepad1.x, currentY = gamepad1.y;
        boolean currentLeftBumper = gamepad1.left_bumper, currentRightBumper = gamepad1.right_bumper;

        if (currentLeftBumper && !lastLeftBumper) { onLeftBumper(); }
        if (currentRightBumper && !lastRightBumper) { onRightBumper(); }
        if (currentX && !lastX) { onButtonX(); }

        lastA = currentA; lastB = currentB; lastX = currentX; lastY = currentY;
        lastLeftBumper = currentLeftBumper; lastRightBumper = currentRightBumper;
    }
    
    private void onButtonX() {
        launchSequence();
    }
    
    private void onLeftBumper() { toggleLift(); }
    private void onRightBumper() { precisionMode = !precisionMode; }
    
    private void runManualMode() {
        handleDriveControl();
        handleManualFlywheelControl();
        handleIntakeControl();
    }
    
    private void handleDriveControl() {
        double drive = applyDeadzone(gamepad1.left_stick_y);
        double strafe = applyDeadzone(gamepad1.left_stick_x);
        double turn = applyDeadzone(-gamepad1.right_stick_x);

        double speed = precisionMode ? DRIVE_SPEED_SLOW : DRIVE_SPEED_NORMAL;

        setMecanumDrive(drive * speed, strafe * speed, turn * speed);
    }
    
    private void handleManualFlywheelControl() {
        if (gamepad1.right_trigger > DRIVE_DEADZONE) {
            setFlywheelPower(FLYWHEEL_LAUNCH_POWER);
        } else if (gamepad1.left_trigger > DRIVE_DEADZONE) {
            setFlywheelPower(FLYWHEEL_IDLE_POWER);
        } else {
            setFlywheelPower(0.0);
        }
    }
    
    private void handleIntakeControl() {
        if (gamepad1.a) {
            intakeMotor.setPower(INTAKE_POWER);
        } else if (gamepad1.b) {
            intakeMotor.setPower(-INTAKE_POWER);
        } else {
            intakeMotor.setPower(0.0);
        }
    }

    private void launchSequence() {
        setFlywheelPower(FLYWHEEL_LAUNCH_POWER);
        sleep(FLYWHEEL_SPINUP_TIME);
    }

    private void setMecanumDrive(double d, double s, double t) {
        double frontLeftPower = d + s + t;
        double frontRightPower = d - s - t;
        double backLeftPower = d - s + t;
        double backRightPower = d + s - t;

        double maxPower = Math.max(Math.abs(frontLeftPower),
                            Math.max(Math.abs(frontRightPower),
                            Math.max(Math.abs(backLeftPower),
                            Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private void setFlywheelPower(double p) {
        launcherLeft.setPower(p);
        launcherRight.setPower(p);
    }

    private void setLiftPosition(boolean u) {
        liftIsUp = u;
        double pos = u ? LIFT_UP_POS : LIFT_DOWN_POS;
        liftServoLeft.setPosition(pos);
        liftServoRight.setPosition(1.0 - pos);
    }

    private void toggleLift() { setLiftPosition(!liftIsUp); }

    private void stopAllMotors() {
        frontLeft.setPower(0); frontRight.setPower(0);
        backLeft.setPower(0); backRight.setPower(0);
        launcherLeft.setPower(0); launcherRight.setPower(0);
        intakeMotor.setPower(0);
    }

    private void setDriveBehavior(DcMotor.ZeroPowerBehavior b) {
        frontLeft.setZeroPowerBehavior(b);
        frontRight.setZeroPowerBehavior(b);
        backLeft.setZeroPowerBehavior(b);
        backRight.setZeroPowerBehavior(b);
    }

    private double applyDeadzone(double v) { return Math.abs(v) < DRIVE_DEADZONE ? 0 : v; }

    private void updateTelemetry() {
        telemetry.addData("Time", "%.1f", runtime.seconds());
        telemetry.addData("PATTERN", String.format("[%s] [%s] [%s]", currentPattern[0], currentPattern[1], currentPattern[2]));
        telemetry.addData("Drive Speed", precisionMode ? "**SLOW**" : "NORMAL");
        telemetry.addData("Lift Position", liftIsUp ? "UP" : "DOWN");
        telemetry.addData("Flywheel Power", "%.2f", launcherLeft.getPower());
        telemetry.addData("Intake Power", "%.2f", intakeMotor.getPower());
        telemetry.update();
    }

    private void cleanup() {
        stopAllMotors();
    }
}
