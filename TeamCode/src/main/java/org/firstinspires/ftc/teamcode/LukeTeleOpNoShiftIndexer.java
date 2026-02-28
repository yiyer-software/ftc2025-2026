package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "TeleOp2026FullShootingSequence", group = "Competition")
public class TeleOp2026FullShootingSequence extends LinearOpMode {
    // ================= HARDWARE =================
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx launcherLeft, launcherRight;
    private DcMotor intakeMotor, sortMotor;
    private Servo hoodLeft, hoodRight, liftLeft, liftRight;
    private ColorSensor colorSensor;

    // ================= DASHBOARD SORTER VARIABLES =================
    public static double SORT_MOTOR_TICKS_PER_REV = 560.0; // REV 20:1
    public static double SORT_GEAR_NUM = 16.0;
    public static double SORT_GEAR_DEN = 13.0;
    public static double SORT_DEGREES = 173.0;
    public static double SORT_POWER = 0.5;
    private double ticksPerOutputRev;
    private double ticksPerDegree;
    private int SORT_MOVE_TICKS;

    // ================= DRIVE CONSTANTS =================
    private final double TICKS_PER_REV = 28.0;
    private final double MAX_RPM = 7000.0;
    private double DRIVE_TICKS_PER_SEC;
    private final double DRIVE_DEADZONE = 0.03;

    // ================= STATE =================
    private boolean shooterOn = false;
    private FtcDashboard dashboard;
    private MultipleTelemetry telemetryDashboard;
    private double lastTime = 0.0;
    private double hoodPosX = 0.8;

    // ================= LIFT STATE =================
    private boolean liftActive = false;
    private double liftStartTime = 0.0;

    // ================= TRACKING BALLS AND INDEXER =================
    private int[] indexOrder = new int[]{2, 2, 2};
    private int indexerPos = 0;
    private boolean notShifting = true;

    // ================= SHOOTING STATE =================
    private boolean shootingActive = false;
    private boolean lastLB = false;
    private boolean flywheelsStarted = false;
    static final double COUNTS_PER_REV = 1378.0;
    static final double COUNTS_PER_DEGREE = COUNTS_PER_REV / 360.0;
    private int shootState = 0;
    private int ballsLaunched = 0;
    private double sorterTimer = 0.0;
    private double liftTimer = 0.0;
    // ================= SHOOTING STATE MOTIF =================
    private boolean lastY = false;
    private int[] motifOrder = new int[]{0, 0, 0}; // TODO: populate from color sensor during intake
    private int[] shootOrder = new int[]{3, 3, 3};

    // ================= INTAKE STATE =================
    private boolean lastRB = false;
    private boolean intakeSequenceActive = false;
    private int intakeState = 0;
    private int ballsIn = 0;
    private double intakeTimer = 0.0;

    @Override
    public void runOpMode() {
        // HARDWARE MAP
        frontLeft  = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft   = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight  = hardwareMap.get(DcMotorEx.class, "rightBack");

        launcherLeft  = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        launcherRight = hardwareMap.get(DcMotorEx.class, "rightLauncher");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        sortMotor   = hardwareMap.get(DcMotor.class, "sortMotor");

        hoodLeft  = hardwareMap.get(Servo.class, "leftHood");
        hoodRight = hardwareMap.get(Servo.class, "rightHood");
        liftLeft  = hardwareMap.get(Servo.class, "liftLeft");
        liftRight = hardwareMap.get(Servo.class, "liftRight");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // MOTOR SETUP
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sortMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DRIVE_TICKS_PER_SEC = (MAX_RPM / 60.0) * TICKS_PER_REV;

        dashboard = FtcDashboard.getInstance();
        telemetryDashboard = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // COMPUTE SORTER MATH
        ticksPerOutputRev = SORT_MOTOR_TICKS_PER_REV * (SORT_GEAR_NUM / SORT_GEAR_DEN);
        ticksPerDegree = ticksPerOutputRev / 360.0;
        SORT_MOVE_TICKS = (int) Math.round(SORT_DEGREES * ticksPerDegree);

        waitForStart();
        lastTime = getRuntime();

        while (opModeIsActive()) {
            handleDrive();
            shootingSequenceMotif(); // FIX 4: was calling shootingSequence(); motif version is the intended competition method
            intakeSequence();
            handleLift();
            shiftIndexer();
            handleHoodManual();
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
        if (max > 1) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeft.setVelocity(fl  * DRIVE_TICKS_PER_SEC);
        frontRight.setVelocity(fr * DRIVE_TICKS_PER_SEC);
        backLeft.setVelocity(bl   * DRIVE_TICKS_PER_SEC);
        backRight.setVelocity(br  * DRIVE_TICKS_PER_SEC);
    }

    // ================= LIFT =================
    private void handleLift() {
        boolean x = gamepad1.x;

        if (!liftActive && x) {
            liftActive = true;
            liftStartTime = getRuntime();
            liftLeft.setPosition(0.35);
            liftRight.setPosition(0.65);
        }

        if (liftActive && getRuntime() - liftStartTime >= 1) {
            liftLeft.setPosition(0.99);
            liftRight.setPosition(0.01);
            liftActive = false;
        }
    }

    // ================= MANUAL SHIFT INDEXER =================
    private void shiftIndexer() {
        boolean a = gamepad1.a;
        if (a && notShifting) {
            notShifting = false;
            int currentPos = sortMotor.getCurrentPosition();
            SORT_MOVE_TICKS = (int) ((5.0) * COUNTS_PER_DEGREE); // Shift 5 degrees
            int target = currentPos + SORT_MOVE_TICKS;
            sortMotor.setTargetPosition(target);
            sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sortMotor.setPower(SORT_POWER);
            sorterTimer = getRuntime();
        }
        if (getRuntime() - sorterTimer >= 0.1) {
            notShifting = true;
        }
    }

    // ================= HOOD =================
    private void handleHoodManual() {
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        double speed = 0.05;

        if (gamepad1.dpad_up)   hoodPosX -= speed * dt;
        if (gamepad1.dpad_down) hoodPosX += speed * dt;

        // FIX 1: was "heoodPosX" (typo) — corrected to "hoodPosX"
        hoodPosX = Math.max(0.01, Math.min(0.7, hoodPosX));
        hoodLeft.setPosition(hoodPosX);
        hoodRight.setPosition(1.0 - hoodPosX);
    }

    // ================= SHIFTING INDEXER (parameterized) =================
    // FIX 2: parameter was missing its type — added "double" before degreesShift
    private void shiftIndexer(double degreesShift) {
        int currentPos = sortMotor.getCurrentPosition();
        SORT_MOVE_TICKS = (int) ((degreesShift) * COUNTS_PER_DEGREE);
        int target = currentPos + SORT_MOVE_TICKS;
        sortMotor.setTargetPosition(target);
        sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sortMotor.setPower(SORT_POWER);
    }

    // ================= SHOOTING (non-motif, kept for reference) =================
    private void shootingSequence() {
        boolean lb = gamepad1.left_bumper;
        if (lb && !lastLB && !shootingActive) {
            shootingActive = true;
            shootState = 0;
            flywheelsStarted = false;
        }
        lastLB = lb;

        if (!shootingActive) return;

        double vel = 3000.0 / 60.0 * TICKS_PER_REV;
        launcherLeft.setVelocity(vel);
        launcherRight.setVelocity(vel);

        switch (shootState) {
            case 0:
                if (!flywheelsStarted) {
                    flywheelsStarted = true;
                }
                shootState = 1;
                break;

            case 1:
                if (!sortMotor.isBusy()) {
                    if (ballsIn != 3 && ballsLaunched == 0) {
                        shiftIndexer(SORT_DEGREES / 4.0);
                        indexerPos += 2;
                        indexerPos = indexerPos % 3;
                        sorterTimer = getRuntime();
                    } else if (ballsLaunched >= 1) {
                        shiftIndexer(SORT_DEGREES / 2.0);
                        indexerPos++;
                        indexerPos = indexerPos % 3;
                        sorterTimer = getRuntime();
                    }
                    shootState = 2;
                }
                break;

            case 2:
                if (!sortMotor.isBusy() && getRuntime() - sorterTimer >= 0.35) {
                    liftLeft.setPosition(0.35);
                    liftRight.setPosition(0.65);
                    liftTimer = getRuntime();
                    shootState = 3;
                }
                break;

            case 3:
                if (getRuntime() - liftTimer >= 0.75) {
                    liftLeft.setPosition(0.99);
                    liftRight.setPosition(0.01);
                    liftTimer = getRuntime();
                    ballsLaunched++;
                    // FIX 6 (applied here too): guard against ballsIn going negative
                    if (ballsIn > 0) ballsIn--;
                    shootState = 4;
                }
                break;

            case 4:
                if (ballsIn == 0) {
                    shootState = 5;
                } else if (getRuntime() - liftTimer >= 0.75) {
                    shootState = 1;
                }
                break;

            case 5:
                shootingActive = false;
                ballsLaunched = 0;
                indexOrder = new int[]{2, 2, 2};
                launcherLeft.setVelocity(0);
                launcherRight.setVelocity(0);

                shiftIndexer(SORT_DEGREES / 4.0);
                indexerPos += 2;
                indexerPos = indexerPos % 3;
                break;
        }
    }

    private int[] findShootingOrder(int[] motif, int[] indexer) {
        int[] shoot = new int[]{3, 3, 3};
        for (int ball = 0; ball < 3; ball++) {
            int motifBall = motif[ball];
            for (int index = 0; index < 3; index++) {
                if (motifBall == indexer[index]) {
                    shoot[ball] = index;
                    indexer[index] = 2; // mark as used in the local copy
                    break;
                }
            }
        }
        // Fill any unfilled shooting slots with remaining open indexer positions
        for (int i = 0; i < 3; i++) {
            if (shoot[i] == 3) {
                for (int ball = 0; ball < 3; ball++) {
                    if (indexer[ball] == 1 || indexer[ball] == 0) {
                        shoot[i] = ball;
                        indexer[ball] = 2;
                        break;
                    }
                }
            }
        }
        // Fallback: if any slot is still unfilled, default to 0
        for (int i = 0; i < 3; i++) {
            if (shoot[i] == 3) {
                shoot[i] = 0;
            }
        }
        return shoot;
    }

    // ================= SHOOTING MOTIF =================
    private void shootingSequenceMotif() {
        boolean lb = gamepad1.left_bumper;
        if (lb && !lastLB && !shootingActive) {
            shootingActive = true;
            shootState = 0;
            flywheelsStarted = false;
        }
        lastLB = lb;

        if (!shootingActive) return;

        double vel = 3000.0 / 60.0 * TICKS_PER_REV;
        launcherLeft.setVelocity(vel);
        launcherRight.setVelocity(vel);

        switch (shootState) {
            // FIX 5: case 0 was missing entirely — sequence would stall at shootState=0 forever
            case 0:
                flywheelsStarted = true;
                // FIX 6: pass indexOrder.clone() so findShootingOrder does not corrupt indexOrder
                shootOrder = findShootingOrder(motifOrder, indexOrder.clone());
                shootState = 1;
                break;

            case 1:
                if (!sortMotor.isBusy()) {
                    // FIX 3: rotationsShifted declared before the if/else to avoid out-of-scope error
                    int rotationsShifted;
                    if (ballsIn != 3 && ballsLaunched == 0) {
                        shiftIndexer(SORT_DEGREES / 4.0);
                        indexerPos += 2;
                        indexerPos = indexerPos % 3;
                        sorterTimer = getRuntime();

                        rotationsShifted = shootOrder[ballsLaunched] - indexerPos;
                        shiftIndexer(rotationsShifted * SORT_DEGREES / 2.0);
                        indexerPos = shootOrder[ballsLaunched];

                    } else if (ballsLaunched >= 1) {
                        rotationsShifted = shootOrder[ballsLaunched] - indexerPos;
                        shiftIndexer(rotationsShifted * SORT_DEGREES / 2.0);
                        indexerPos = shootOrder[ballsLaunched] % 3;
                        sorterTimer = getRuntime();
                    }
                    shootState = 2;
                }
                break;

            case 2:
                if (!sortMotor.isBusy() && getRuntime() - sorterTimer >= 0.35) {
                    liftLeft.setPosition(0.35);
                    liftRight.setPosition(0.65);
                    liftTimer = getRuntime();
                    shootState = 3;
                }
                break;

            case 3:
                if (getRuntime() - liftTimer >= 0.75) {
                    liftLeft.setPosition(0.99);
                    liftRight.setPosition(0.01);
                    liftTimer = getRuntime();
                    ballsLaunched++;
                    // FIX 6: guard against ballsIn going negative
                    if (ballsIn > 0) ballsIn--;
                    shootState = 4;
                }
                break;

            case 4:
                if (ballsIn == 0) {
                    shootState = 5;
                } else if (getRuntime() - liftTimer >= 0.5) {
                    shootState = 1;
                }
                break;

            case 5:
                shootingActive = false;
                ballsLaunched = 0;
                indexOrder = new int[]{2, 2, 2};
                launcherLeft.setVelocity(0);
                launcherRight.setVelocity(0);

                shiftIndexer(SORT_DEGREES / 4.0);
                indexerPos += 2;
                indexerPos = indexerPos % 3;
                break;
        }
    }

    // ================= INTAKE =================
    private void intakeSequence() {
        boolean rb = gamepad1.right_bumper;
        if (rb && !lastRB && !intakeSequenceActive) {
            intakeSequenceActive = true;
            intakeState = 0;
            ballsIn = 0;
        }
        lastRB = rb;

        if (!intakeSequenceActive) return;

        switch (intakeState) {
            case 0:
                intakeMotor.setPower(-0.8);
                boolean greenDetected  = (colorSensor.green() > 100);
                boolean purpleDetected = (colorSensor.red() > 55 && colorSensor.blue() > 80);

                if (greenDetected || purpleDetected) {
                    intakeTimer = getRuntime();

                    if (ballsIn >= 0 && ballsIn < 3) {
                        if (greenDetected)  indexOrder[ballsIn] = 1;
                        if (purpleDetected) indexOrder[ballsIn] = 0;
                    }

                    intakeState = 1;
                }
                // Allow shooting sequence to interrupt intake
                if (shootingActive) {
                    intakeSequenceActive = false;
                    intakeMotor.setPower(0);
                }
                break;

            case 1:
                if (getRuntime() - intakeTimer >= 0.75 && !sortMotor.isBusy()) {
                    shiftIndexer(SORT_DEGREES / 2.0);
                    indexerPos++;
                    indexerPos = indexerPos % 3;
                    ballsIn++;
                    intakeState = 2;
                }
                break;

            case 2:
                if (!sortMotor.isBusy() && ballsIn <= 2) {
                    intakeState = 0;
                }
                if (ballsIn >= 3) {
                    intakeSequenceActive = false;
                    shiftIndexer(SORT_DEGREES / 4.0);
                    // FIX (tip): replaced sleep(350) with a non-blocking timer approach.
                    // The sorter motor will finish on its own; we just stop the intake immediately.
                    // If you need a post-sort delay before stopping power, add a new state here.
                    sortMotor.setPower(0);
                    intakeMotor.setPower(0);
                    indexerPos += 2;
                    indexerPos = indexerPos % 3;
                }
                break;
        }
    }

    // ================= TELEMETRY =================
    private void updateTelemetry() {
        telemetryDashboard.addData("Sorter position",  sortMotor.getCurrentPosition());
        telemetryDashboard.addData("Red",              colorSensor.red());
        telemetryDashboard.addData("Green",            colorSensor.green());
        telemetryDashboard.addData("Blue",             colorSensor.blue());
        telemetryDashboard.addData("Balls in",         ballsIn);
        telemetryDashboard.addData("Balls launched",   ballsLaunched);
        telemetryDashboard.addData("Ball Order",       indexOrder);
        telemetryDashboard.addData("Indexer position", indexerPos);
        telemetryDashboard.addData("Shoot order",      shootOrder);
        telemetryDashboard.update();
    }
}
