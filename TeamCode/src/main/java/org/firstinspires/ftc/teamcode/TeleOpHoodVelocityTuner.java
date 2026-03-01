package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "HoodVelocityTuner", group = "Tuning")
public class TeleOpHoodVelocityTuner extends LinearOpMode {

    // ================= HARDWARE =================
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx launcherLeft, launcherRight;
    private DcMotor   sortMotor;
    private Servo     hoodLeft, hoodRight, liftLeft, liftRight;

    // ================= DASHBOARD TUNABLE =================
    public static double LAUNCH_RPM        = 3000.0;  // tune this
    public static double HOOD_POSITION     = 0.5;     // tune this (0.01 – 0.70)

    // ================= DASHBOARD SORTER VARIABLES =================
    public static double SORT_DEGREES      = 173.0;
    public static double SORT_POWER        = 0.5;

    // ================= CONSTANTS =================
    private final double TICKS_PER_REV     = 28.0;
    private final double MAX_RPM           = 7000.0;
    private       double DRIVE_TICKS_PER_SEC;
    private final double DRIVE_DEADZONE    = 0.03;
    static final  double COUNTS_PER_REV    = 1378.0;
    static final  double COUNTS_PER_DEGREE = COUNTS_PER_REV / 360.0;

    // ================= DASHBOARD =================
    private FtcDashboard      dashboard;
    private MultipleTelemetry telemetryDashboard;

    // ================= SHOOTING STATE =================
    private boolean shootingActive = false;
    private boolean lastRB         = false;
    private int     shootState     = 0;
    private double  sorterTimer    = 0.0;
    private double  liftTimer      = 0.0;
    private int     indexerPos     = 0;

    @Override
    public void runOpMode() {
        // ---- HARDWARE MAP ----
        frontLeft  = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft   = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight  = hardwareMap.get(DcMotorEx.class, "rightBack");

        launcherLeft  = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        launcherRight = hardwareMap.get(DcMotorEx.class, "rightLauncher");

        sortMotor = hardwareMap.get(DcMotor.class, "sortMotor");

        hoodLeft  = hardwareMap.get(Servo.class, "leftHood");
        hoodRight = hardwareMap.get(Servo.class, "rightHood");
        liftLeft  = hardwareMap.get(Servo.class, "liftLeft");
        liftRight = hardwareMap.get(Servo.class, "liftRight");

        // ---- MOTOR SETUP ----
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sortMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DRIVE_TICKS_PER_SEC = (MAX_RPM / 60.0) * TICKS_PER_REV;

        dashboard          = FtcDashboard.getInstance();
        telemetryDashboard = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Start kicker retracted
        liftLeft.setPosition(0.99);
        liftRight.setPosition(0.01);

        waitForStart();

        while (opModeIsActive()) {
            handleDrive();
            applyHood();
            shootingSequence();
            updateTelemetry();
        }
    }

    // ================= DRIVE =================
    private void handleDrive() {
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x * 1.15;
        double rx =  gamepad1.right_stick_x;

        if (Math.abs(y)  < DRIVE_DEADZONE) y  = 0;
        if (Math.abs(x)  < DRIVE_DEADZONE) x  = 0;
        if (Math.abs(rx) < DRIVE_DEADZONE) rx = 0;

        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1) { fl /= max; fr /= max; bl /= max; br /= max; }

        frontLeft.setVelocity(fl * DRIVE_TICKS_PER_SEC);
        frontRight.setVelocity(fr * DRIVE_TICKS_PER_SEC);
        backLeft.setVelocity(bl * DRIVE_TICKS_PER_SEC);
        backRight.setVelocity(br * DRIVE_TICKS_PER_SEC);
    }

    // ================= HOOD — live updates from dashboard =================
    private void applyHood() {
        double clamped = Math.max(0.01, Math.min(0.70, HOOD_POSITION));
        hoodLeft.setPosition(clamped);
        hoodRight.setPosition(1.0 - clamped);
    }

    // ================= SHOOTING =================
    //  Press RB once → spin up flywheels, shift sorter one slot, kick, retract, done.
    //  Press RB again for the next shot. Each press fires one ball.
    //
    //  State 0 — Spin up + shift sorter one slot forward.
    //  State 1 — Wait for sorter to finish.
    //  State 2 — Settle 0.35 s, then extend kicker.
    //  State 3 — Wait 0.75 s, retract kicker.
    //  State 4 — Wait 0.75 s, stop flywheels, mark done.
    //
    private void shootingSequence() {
        boolean rb = gamepad1.right_bumper;
        if (rb && !lastRB && !shootingActive) {
            shootingActive = true;
            shootState     = 0;
        }
        lastRB = rb;

        if (!shootingActive) return;

        // Hold flywheel velocity every loop so PID stays locked
        double vel = (LAUNCH_RPM / 60.0) * TICKS_PER_REV;
        launcherLeft.setVelocity(vel);
        launcherRight.setVelocity(vel);

        switch (shootState) {

            case 0:
                // Shift sorter one slot (SORT_DEGREES/2) to bring a ball to the kicker
                if (!sortMotor.isBusy()) {
                    int moveTicks = (int)((SORT_DEGREES / 2.0) * COUNTS_PER_DEGREE);
                    int target    = sortMotor.getCurrentPosition() + moveTicks;
                    sortMotor.setTargetPosition(target);
                    sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sortMotor.setPower(SORT_POWER);
                    indexerPos = (indexerPos + 1) % 3;
                    shootState = 1;
                }
                break;

            case 1:
                // Wait for sorter to reach position
                if (!sortMotor.isBusy()) {
                    sorterTimer = getRuntime();
                    shootState  = 2;
                }
                break;

            case 2:
                // Settle, then kick
                if (getRuntime() - sorterTimer >= 0.35) {
                    liftLeft.setPosition(0.15);
                    liftRight.setPosition(0.85);
                    liftTimer  = getRuntime();
                    shootState = 3;
                }
                break;

            case 3:
                // Wait for ball to clear, then retract
                if (getRuntime() - liftTimer >= 0.75) {
                    liftLeft.setPosition(0.99);
                    liftRight.setPosition(0.01);
                    liftTimer  = getRuntime();
                    shootState = 4;
                }
                break;

            case 4:
                // Short pause after retract, then stop flywheels and finish
                if (getRuntime() - liftTimer >= 1.0) {
                    launcherLeft.setVelocity(0);
                    launcherRight.setVelocity(0);
                    shootingActive = false;
                }
                break;
        }
    }

    // ================= TELEMETRY =================
    private void updateTelemetry() {
        telemetryDashboard.addData("=== TUNING ===",      "");
        telemetryDashboard.addData("LAUNCH_RPM",          LAUNCH_RPM);
        telemetryDashboard.addData("HOOD_POSITION",       HOOD_POSITION);
        telemetryDashboard.addData("=== STATE ===",       "");
        telemetryDashboard.addData("Shoot State",         shootState);
        telemetryDashboard.addData("Shooting Active",     shootingActive);
        telemetryDashboard.addData("Indexer Pos",         indexerPos);
        telemetryDashboard.addData("Sorter Position",     sortMotor.getCurrentPosition());
        telemetryDashboard.addData("Sorter Busy",         sortMotor.isBusy());
        telemetryDashboard.addData("Launcher L Velocity", launcherLeft.getVelocity());
        telemetryDashboard.addData("Launcher R Velocity", launcherRight.getVelocity());
        telemetryDashboard.update();
    }
}