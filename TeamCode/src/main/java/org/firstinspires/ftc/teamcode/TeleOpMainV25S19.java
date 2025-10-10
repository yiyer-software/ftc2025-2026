package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpMainV25S19")
public class TeleOpMainV25S19 extends LinearOpMode {

    // =======================
    // EDITABLE SETTINGS
    // =======================
    // Hardware names - match your Robot Configuration
    private static final String LEFT_MOTOR_NAME  = "leftMotor";
    private static final String RIGHT_MOTOR_NAME = "rightMotor";
    private static final String LAUNCHER_NAME    = "launcherMotor";

    // Drive tuning
    private static final double DRIVE_MAX_POWER          = 1.0;   // maximum drive power
    private static final double DRIVE_DEADZONE          = 0.05;  // sticks within this range considered 0
    private static final double DRIVE_CUBIC_FACTOR      = 1.5;   // >1 increases sensitivity curve (cubic-like)
    private static final double DRIVE_SLEW_ALPHA        = 0.12;  // low-pass filter alpha for smoothing (0-1) lower = smoother/slower
    private static final double DRIVE_ACCEL_LIMIT       = 0.06;  // max change in power per loop (prevents jerky acceleration)

    // Launcher tuning
    private static final double LAUNCHER_FULL_POWER     = 1.0;
    private static final double LAUNCHER_HALF_POWER     = 0.55;
    private static final double LAUNCHER_REVERSE_POWER  = -0.5;
    private static final double LAUNCHER_BOOST_POWER    = 1.0;   // held RB
    private static final double LAUNCHER_SLEW_ALPHA     = 0.18;  // launcher motor low-pass smoothing

    // Button debounce (seconds)
    private static final double DEBOUNCE_SECS = 0.18;

    // Zero power behavior
    private static final ZeroPowerBehavior BRAKE_BEHAVIOR = ZeroPowerBehavior.BRAKE;
    // =======================
    // End editable settings
    // =======================

    // Motors
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor launcherMotor = null;

    // States and helpers
    private boolean launcherToggleState = false;
    private boolean launcherReverseToggle = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private final ElapsedTime debounceTimer = new ElapsedTime();

    // Smoothed outputs (persist between loops)
    private double smoothedLeftPower = 0.0;
    private double smoothedRightPower = 0.0;
    private double smoothedLauncherPower = 0.0;

    @Override
    public void runOpMode() {
        // Hardware map
        leftMotor = hardwareMap.get(DcMotor.class, LEFT_MOTOR_NAME);
        rightMotor = hardwareMap.get(DcMotor.class, RIGHT_MOTOR_NAME);
        launcherMotor = hardwareMap.get(DcMotor.class, LAUNCHER_NAME);

        // Motor direction - change if your wiring requires it
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        launcherMotor.setDirection(DcMotor.Direction.FORWARD);

        // Brake for better control
        leftMotor.setZeroPowerBehavior(BRAKE_BEHAVIOR);
        rightMotor.setZeroPowerBehavior(BRAKE_BEHAVIOR);
        launcherMotor.setZeroPowerBehavior(BRAKE_BEHAVIOR);

        telemetry.addData("Status", "Initialized - Edit top constants to tune");
        telemetry.update();

        waitForStart();
        debounceTimer.reset();

        while (opModeIsActive()) {
            // ---------- Read raw inputs ----------
            double rawLeftStick  = -gamepad1.left_stick_y;   // stick up = positive
            double rawRightStick = -gamepad1.right_stick_y;

            // ---------- Apply deadzone ----------
            double left = applyDeadzone(rawLeftStick, DRIVE_DEADZONE);
            double right = applyDeadzone(rawRightStick, DRIVE_DEADZONE);

            // ---------- Apply cubic-ish scaling for finer low-speed control ----------
            // Uses sign preserving power curve: out = sign*|in|^p where p = DRIVE_CUBIC_FACTOR
            left = Math.signum(left) * Math.pow(Math.abs(left), DRIVE_CUBIC_FACTOR);
            right = Math.signum(right) * Math.pow(Math.abs(right), DRIVE_CUBIC_FACTOR);

            // ---------- Optional precision slow mode (hold left_trigger for fine control) ----------
            double precisionFactor = 1.0 - 0.6 * gamepad1.left_trigger; // LT reduces speed to 0.4..1.0
            left *= precisionFactor;
            right *= precisionFactor;

            // ---------- Scale to max power ----------
            left *= DRIVE_MAX_POWER;
            right *= DRIVE_MAX_POWER;

            // ---------- Slew (low-pass) filter for smoothness ----------
            smoothedLeftPower  = lowPass(smoothedLeftPower, left, DRIVE_SLEW_ALPHA);
            smoothedRightPower = lowPass(smoothedRightPower, right, DRIVE_SLEW_ALPHA);

            // ---------- Acceleration limiter (prevents sudden jumps) ----------
            smoothedLeftPower  = limitDelta(smoothedLeftPower, leftMotor.getPower(), DRIVE_ACCEL_LIMIT);
            smoothedRightPower = limitDelta(smoothedRightPower, rightMotor.getPower(), DRIVE_ACCEL_LIMIT);

            // ---------- Apply to motors ----------
            leftMotor.setPower(clamp(smoothedLeftPower, -1.0, 1.0));
            rightMotor.setPower(clamp(smoothedRightPower, -1.0, 1.0));

            // ---------- Launcher controls (debounced toggles + overrides) ----------
            // A toggles launcher on/off
            if (gamepad1.a && !lastA && debounceTimer.seconds() > DEBOUNCE_SECS) {
                launcherToggleState = !launcherToggleState;
                debounceTimer.reset();
            }
            lastA = gamepad1.a;

            // B toggles a persistent reverse mode (useful for clearing jams)
            if (gamepad1.b && !lastB && debounceTimer.seconds() > DEBOUNCE_SECS) {
                launcherReverseToggle = !launcherReverseToggle;
                debounceTimer.reset();
            }
            lastB = gamepad1.b;

            // X while held -> half-speed mode (temporary); toggling X is not used to keep UX simple
            boolean halfModeHeld = gamepad1.x;

            // RB for temporary boost while held
            boolean boostHeld = gamepad1.right_bumper;

            // Compute target launcher power
            double targetLauncher = 0.0;
            if (launcherReverseToggle) {
                targetLauncher = LAUNCHER_REVERSE_POWER;
            } else if (launcherToggleState) {
                if (halfModeHeld) {
                    targetLauncher = LAUNCHER_HALF_POWER;
                } else if (boostHeld) {
                    targetLauncher = LAUNCHER_BOOST_POWER;
                } else {
                    targetLauncher = LAUNCHER_FULL_POWER;
                }
            } else {
                targetLauncher = 0.0;
            }

            // If B is currently held, override with immediate reverse (temporary while held)
            if (gamepad1.b) {
                targetLauncher = LAUNCHER_REVERSE_POWER;
            }

            // Slew filter for launcher for smooth spin-up/spin-down
            smoothedLauncherPower = lowPass(smoothedLauncherPower, targetLauncher, LAUNCHER_SLEW_ALPHA);
            launcherMotor.setPower(clamp(smoothedLauncherPower, -1.0, 1.0));

            // ---------- Telemetry (concise) ----------
            telemetry.addData("Drive L (raw->smoothed)", "%.2f -> %.2f", left, leftMotor.getPower());
            telemetry.addData("Drive R (raw->smoothed)", "%.2f -> %.2f", right, rightMotor.getPower());
            telemetry.addData("Launcher target/smoothed", "%.2f / %.2f", targetLauncher, launcherMotor.getPower());
            telemetry.addData("Launcher Toggled", launcherToggleState);
            telemetry.addData("Launcher Reverse Persist", launcherReverseToggle);
            telemetry.addData("Precision factor (LT)", "%.2f", precisionFactor);
            telemetry.update();
        }
    }

    // =======================
    // Utility functions
    // =======================
    private double applyDeadzone(double value, double deadzone) {
        return Math.abs(value) < deadzone ? 0.0 : value;
    }

    // Exponential moving average low-pass filter
    private double lowPass(double currentSmoothed, double target, double alpha) {
        return currentSmoothed + alpha * (target - currentSmoothed);
    }

    // Limit how much power can change relative to the currently applied motor power
    // This compares the requested smoothed value to the actual motor power to avoid abrupt jumps
    private double limitDelta(double requested, double actualMotorPower, double maxDelta) {
        double delta = requested - actualMotorPower;
        if (Math.abs(delta) > maxDelta) {
            return actualMotorPower + Math.signum(delta) * maxDelta;
        } else {
            return requested;
        }
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}