package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

@Config
@TeleOp(name = "TeleOp2026FullShootingSequence", group = "Competition")
public class TeleOp2026FullShootingSequence extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx launcherLeft, launcherRight;
    private DcMotor   intakeMotor, sortMotor;

    private MecanumDrive drive;

    private Servo     hoodLeft, hoodRight, liftLeft, liftRight;
    private ColorSensor colorSensor;

    // sorter constants
    public static double SORT_MOTOR_TICKS_PER_REV = 560.0;
    public static double SORT_GEAR_NUM  = 16.0;
    public static double SORT_GEAR_DEN  = 13.0;
    public static double SORT_DEGREES   = 173.0;
    public static double SORT_POWER     = 0.4;
    private double ticksPerOutputRev;
    private double ticksPerDegree;
    private int    SORT_MOVE_TICKS;

    private final double TICKS_PER_REV    = 28.0;
    private final double MAX_RPM          = 7000.0;
    private       double DRIVE_TICKS_PER_SEC;
    private final double DRIVE_DEADZONE   = 0.03;

    static final double COUNTS_PER_REV    = 1378.0;

    private double  headingOffset = 0.0;

    static final double COUNTS_PER_DEGREE = COUNTS_PER_REV / 360.0;

    // CHANGED: how long (seconds) to pause after retracting the kicker before
    // allowing the sorter to rotate. Increase if kicker is still in the way.
    public static double KICKER_DOWN_WAIT_SEC = 0.35;
    private double kickerDownTimer = 0.0;

    private FtcDashboard       dashboard;
    private MultipleTelemetry  telemetryDashboard;
    private double             lastTime = 0.0;
    private double             hoodPosX = 0.8;

    // currently selected launcher RPM (rpm)
    private double launcherRPM = 3000.0;

    // edge tracking for preset buttons
    private boolean lastLB = false; // still used to detect shooting triggers for left bumper
    private boolean lastB  = false;
    private boolean lastY  = false;
    private boolean lastX  = false;

    // --- motif / limelight state (first-detection locks pattern) ---
    private LimelightSubSystem limelight = null;
    private boolean motifDetected = false;
    private int motifTagId = -1;
    // motifPattern holds color codes: 1 = green, 0 = purple; length=3
    private int[] motifPattern = null;

    // dpad edge tracking for color-shoot triggers
    private boolean lastDpadUp = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    private boolean liftActive    = false;
    public static double DRIVE_PRECISION_SCALE = 0.35;
    private double  liftStartTime = 0.0;

    // indexer / balls tracking (your original)
    private int[] indexOrder = new int[]{2, 2, 2};
    private int   indexerPos = 0;

    private boolean shootingActive = false;
    private int     shootState     = 0;
    private int     ballsLaunched  = 0;
    private int     ballsIn        = 0;
    private double  sorterTimer    = 0.0;
    private double  liftTimer      = 0.0;
    private int[]   shootOrder     = new int[]{0, 1, 2};

    private boolean lastRB               = false;
    private boolean intakeSequenceActive = false;
    private int     intakeState          = 0;
    private double  intakeTimer          = 0.0;

    private boolean notShifting = true;
    private double  shiftTimer  = 0.0;

    @Override
    public void runOpMode() {
//        frontLeft  = hardwareMap.get(DcMotorEx.class, "leftFront");
//        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
//        backLeft   = hardwareMap.get(DcMotorEx.class, "leftBack");
//        backRight  = hardwareMap.get(DcMotorEx.class, "rightBack");

        Pose2d startPose = PoseStorage.currentPose != null
                ? PoseStorage.currentPose
                : new Pose2d(0, 0, 0);

        launcherLeft  = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        launcherRight = hardwareMap.get(DcMotorEx.class, "rightLauncher");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        sortMotor   = hardwareMap.get(DcMotor.class, "sortMotor");

        hoodLeft  = hardwareMap.get(Servo.class, "leftHood");
        hoodRight = hardwareMap.get(Servo.class, "rightHood");
        liftLeft  = hardwareMap.get(Servo.class, "liftLeft");
        liftRight = hardwareMap.get(Servo.class, "liftRight");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        drive = new MecanumDrive(hardwareMap, startPose);


//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);

//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sortMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DRIVE_TICKS_PER_SEC = (MAX_RPM / 60.0) * TICKS_PER_REV;

        dashboard          = FtcDashboard.getInstance();
        telemetryDashboard = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        ticksPerOutputRev = SORT_MOTOR_TICKS_PER_REV * (SORT_GEAR_NUM / SORT_GEAR_DEN);
        ticksPerDegree    = ticksPerOutputRev / 360.0;
        SORT_MOVE_TICKS   = (int) Math.round(SORT_DEGREES * ticksPerDegree);

        // instantiate LimelightSubSystem (your provided class)
        try {
            limelight = new LimelightSubSystem(hardwareMap);
        } catch (Exception e) {
            limelight = null;
            telemetryDashboard.addData("Limelight", "init failed: " + e.getMessage());
            telemetryDashboard.update();
        }

        waitForStart();
        lastTime = getRuntime();

        while (opModeIsActive()) {
            // Keep calling identifyMotifTag() every loop so the first motif tag seen locks pattern
            try {
                if (limelight != null && !motifDetected) {
                    boolean ok = limelight.identifyMotifTag();
                    if (ok && !motifDetected) {
                        // read motif pattern from limelight subsystem's static mapping
                        // LimelightSubSystem maps 21->GPP,22->PGP,23->PPG as earlier provided
                        motifDetected = true;
                        motifTagId = limelight.getResult() != null && limelight.getResult().isValid()
                                ? limelight.getResult().getFiducialResults().get(0).getFiducialId()
                                : motifTagId; // best-effort, not required
                        // The identifyMotifTag() in your provided class wrote the pattern into MotifStorage in its version;
                        // since you asked for no MotifStorage, we'll copy pattern by re-deriving from tag id:
                        // Actually identifyMotifTag() has already saved to MotifStorage in your earlier implementation,
                        // but to be self-contained we'll set motifPattern using the tag id returned via identifying flow:
                        // We'll attempt to read motifPattern via the tag id that was found in the fiducials list:
                        // Simpler: call limelight.getResult() and locate motif tag id again to set motifPattern.
                        // (If your identifyMotifTag() already wrote somewhere else, this branch still sets motifPattern.)
                        LLResult res = (limelight != null) ? limelight.getResult() : null;
                        if (res != null && res.getFiducialResults() != null) {
                            for (LLResultTypes.FiducialResult f : res.getFiducialResults()) {
                                int id = f.getFiducialId();
                                if (id == 21) {
                                    motifPattern = new int[]{1,0,0};
                                    motifTagId = 21;
                                    break;
                                } else if (id == 22) {
                                    motifPattern = new int[]{0,1,0};
                                    motifTagId = 22;
                                    break;
                                } else if (id == 23) {
                                    motifPattern = new int[]{0,0,1};
                                    motifTagId = 23;
                                    break;
                                }
                            }
                        }
                        // If we still don't have motifPattern (edge case), leave motifDetected true but pattern null -> handled later.
                        if (motifPattern != null) {
                            telemetryDashboard.addData("Motif Locked", "tag=" + motifTagId + " pattern=" + Arrays.toString(motifPattern));
                            telemetryDashboard.update();
                        } else {
                            // fallback: motifDetected true but no pattern found yet
                            telemetryDashboard.addData("Motif", "locked but pattern unknown");
                            telemetryDashboard.update();
                        }
                    }
                }
            } catch (Exception ex) {
                // don't let limelight exceptions crash TeleOp
                telemetryDashboard.addData("LimelightErr", ex.getMessage());
                telemetryDashboard.update();
            }

            handleDrive();
            handlePresetAndFireInput(); // read B/Y/X/LB presses and begin shooting with selected profile
            // dpad color-shoot triggers:
            handleDpadColorShootingInputs();

            shootingSequence();
            intakeSequence();
            //handleLift();
            shiftIndexerLarge();
            //handleHoodManual();
            updateTelemetry();

            // loop-end; continue
        }
    }

    /**
     * Read preset buttons (LB, B, Y, X) as rising edges.
     * When any of those buttons is pressed, select the corresponding
     * hood + RPM preset and start the shooting sequence immediately.
     *
     * Mapping chosen per your request:
     *  - LEFT BUMPER -> 2150 rpm, hood 0.99 (formerly RB preset)
     *  - B           -> 5000 rpm, hood 0.47
     *  - Y           -> 3500 rpm, hood 0.53
     *  - X           -> 5850 rpm, hood 0.49
     *
     * This function performs edge-detection and triggers shootingActive.
     */
    private void handlePresetAndFireInput() {
        boolean lb = gamepad1.left_bumper;
        boolean b  = gamepad1.b;
        boolean y  = gamepad1.y;
        boolean x  = gamepad1.x;

        // LEFT BUMPER preset + fire: 2150 rpm, hood 0.99
        if (lb && !lastLB && !shootingActive) {
            launcherRPM = 2150.0;
            hoodPosX = 0.99;
            applyHoodImmediate();
            shootingActive = true;
            shootState = 0;
            ballsLaunched = 0;
        }

        // B preset + fire: 5000 rpm, hood 0.47
        if (b && !lastB && !shootingActive) {
            launcherRPM = 5000.0;
            hoodPosX = 0.47;
            applyHoodImmediate();
            shootingActive = true;
            shootState = 0;
            ballsLaunched = 0;
        }

        // Y preset + fire: 3500 rpm, hood 0.53
        if (y && !lastY && !shootingActive) {
            launcherRPM = 3500.0;
            hoodPosX = 0.53;
            applyHoodImmediate();
            shootingActive = true;
            shootState = 0;
            ballsLaunched = 0;
        }

        // X preset + fire: 5850 rpm, hood 0.49
        if (x && !lastX && !shootingActive) {
            launcherRPM = 5850.0;
            hoodPosX = 0.49;
            applyHoodImmediate();
            shootingActive = true;
            shootState = 0;
            ballsLaunched = 0;
        }

        // update last-state trackers (edge detection)
        lastLB = lb;
        lastB  = b;
        lastY  = y;
        lastX  = x;
    }

    private void applyHoodImmediate() {
        // clamp to allowed range and apply servos immediately
        hoodPosX = Math.max(0.01, Math.min(0.99, hoodPosX));
        hoodLeft.setPosition(hoodPosX);
        hoodRight.setPosition(1.0 - hoodPosX);
    }

    // ------------------ DPAD color-shoot input handling ------------------
    private void handleDpadColorShootingInputs() {
        boolean dpadUp    = gamepad1.dpad_up;
        boolean dpadRight = gamepad1.dpad_right;
        boolean dpadLeft  = gamepad1.dpad_left;

        // rising-edge dpad up -> 2100 rpm hood 0.99
        if (dpadUp && !lastDpadUp) {
            if (!motifDetected || motifPattern == null) {
                telemetryDashboard.addData("ColorShoot", "No motif detected yet");
                telemetryDashboard.update();
            } else {
                double prevLauncherRPM = launcherRPM;
                double prevHoodPos = hoodPosX;

                launcherRPM = 2100.0;
                hoodPosX = 0.99;
                applyHoodImmediate();

                boolean ok = runColorAccurateShoot(motifPattern);
                if (!ok) {
                    telemetryDashboard.addData("ColorShoot", "CANNOT SORT: insufficient / mismatch");
                    telemetryDashboard.update();
                    // restore
                    launcherRPM = prevLauncherRPM;
                    hoodPosX = prevHoodPos;
                    applyHoodImmediate();
                }
            }
        }

        // rising-edge dpad right -> 5000 rpm hood 0.47
        if (dpadRight && !lastDpadRight) {
            if (!motifDetected || motifPattern == null) {
                telemetryDashboard.addData("ColorShoot", "No motif detected yet");
                telemetryDashboard.update();
            } else {
                double prevLauncherRPM = launcherRPM;
                double prevHoodPos = hoodPosX;

                launcherRPM = 5000.0;
                hoodPosX = 0.47;
                applyHoodImmediate();

                boolean ok = runColorAccurateShoot(motifPattern);
                if (!ok) {
                    telemetryDashboard.addData("ColorShoot", "CANNOT SORT: insufficient / mismatch");
                    telemetryDashboard.update();
                    // restore
                    launcherRPM = prevLauncherRPM;
                    hoodPosX = prevHoodPos;
                    applyHoodImmediate();
                }
            }
        }

        // rising-edge dpad left -> 5850 rpm hood 0.49
        if (dpadLeft && !lastDpadLeft) {
            if (!motifDetected || motifPattern == null) {
                telemetryDashboard.addData("ColorShoot", "No motif detected yet");
                telemetryDashboard.update();
            } else {
                double prevLauncherRPM = launcherRPM;
                double prevHoodPos = hoodPosX;

                launcherRPM = 5850.0;
                hoodPosX = 0.49;
                applyHoodImmediate();

                boolean ok = runColorAccurateShoot(motifPattern);
                if (!ok) {
                    telemetryDashboard.addData("ColorShoot", "CANNOT SORT: insufficient / mismatch");
                    telemetryDashboard.update();
                    // restore
                    launcherRPM = prevLauncherRPM;
                    hoodPosX = prevHoodPos;
                    applyHoodImmediate();
                }
            }
        }

        lastDpadUp = dpadUp;
        lastDpadRight = dpadRight;
        lastDpadLeft = dpadLeft;
    }

    /**
     * Blocking routine that attempts to shoot in the exact color order `desiredColors`.
     * Re-uses your existing sorter RUN_TO_POSITION commands, kicker timings, and bookkeeping.
     * Returns true if executed successfully, false if impossible.
     *
     * This will not run if intakeSequenceActive or shootingActive (to avoid conflicts).
     */
    private boolean runColorAccurateShoot(int[] desiredColors) {
        // Safety: don't run concurrently with other sequences
        if (shootingActive || intakeSequenceActive) {
            telemetryDashboard.addData("ColorShoot", "Busy: sequence in progress");
            telemetryDashboard.update();
            return false;
        }

        // Quick feasibility check: count available colors
        int[] availCounts = new int[3]; // 0=purple,1=green,2=empty
        for (int c : indexOrder) {
            if (c >= 0 && c <= 2) availCounts[c]++;
        }
        int[] reqCounts = new int[3];
        for (int d : desiredColors) {
            if (d < 0 || d > 1) return false; // invalid desired color
            reqCounts[d]++;
        }
        if (reqCounts[0] > availCounts[0] || reqCounts[1] > availCounts[1]) {
            // cannot satisfy
            return false;
        }

        // All good: perform the blocking sequence.
        // Work on temporary copies, then commit at end.
        int[] tempOrder = Arrays.copyOf(indexOrder, indexOrder.length);
        int tempIndexerPos = indexerPos;

        // Hold launcher at requested rpm during routine
        double velTicksPerSec = (launcherRPM / 60.0) * TICKS_PER_REV;
        launcherLeft.setVelocity(velTicksPerSec);
        launcherRight.setVelocity(velTicksPerSec);

        for (int desiredColor : desiredColors) {
            // find a slot that currently contains desiredColor in tempOrder
            int foundSlot = -1;
            for (int s = 0; s < tempOrder.length; s++) {
                if (tempOrder[s] == desiredColor) {
                    foundSlot = s;
                    break;
                }
            }
            if (foundSlot == -1) {
                // should not happen given feasibility check, but guard
                telemetryDashboard.addData("ColorShoot", "unexpected: slot not found");
                telemetryDashboard.update();
                // stop launcher
                launcherLeft.setVelocity(0);
                launcherRight.setVelocity(0);
                return false;
            }

            // compute steps to bring foundSlot to kicker
            int stepsNeeded = (foundSlot - tempIndexerPos + 3) % 3;

            if (stepsNeeded != 0) {
                int moveTicks = (int)(stepsNeeded * (SORT_DEGREES / 2.0) * COUNTS_PER_DEGREE);
                int target    = sortMotor.getCurrentPosition() + moveTicks;
                sortMotor.setTargetPosition(target);
                sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sortMotor.setPower(SORT_POWER);

                // wait for sorter to finish (blocking)
                while (opModeIsActive() && sortMotor.isBusy()) {
                    // maintain launcher speed while waiting
                    launcherLeft.setVelocity(velTicksPerSec);
                    launcherRight.setVelocity(velTicksPerSec);
                    idle();
                }
                // update tempIndexerPos after move: the foundSlot is now at kicker position
                tempIndexerPos = foundSlot;
            }

            // settle then kick (same timing as shootingSequence)
            double settleStart = getRuntime();
            while (opModeIsActive() && getRuntime() - settleStart < 0.35) {
                launcherLeft.setVelocity(velTicksPerSec);
                launcherRight.setVelocity(velTicksPerSec);
                idle();
            }

            // extend kicker
            liftLeft.setPosition(0.15);
            liftRight.setPosition(0.85);
            double kickStart = getRuntime();
            while (opModeIsActive() && getRuntime() - kickStart < 1.00) {
                launcherLeft.setVelocity(velTicksPerSec);
                launcherRight.setVelocity(velTicksPerSec);
                idle();
            }

            // retract kicker
            liftLeft.setPosition(0.99);
            liftRight.setPosition(0.01);

            // bookkeeping: mark that kicker slot as now empty
            tempOrder[tempIndexerPos] = 2;
            // update counters (these are overall counters that live outside; we update them here)
            ballsLaunched++;
            ballsIn = Math.max(0, ballsIn - 1);

            // give the kicker time to be down before next sort (respect KICKER_DOWN_WAIT_SEC)
            double kdStart = getRuntime();
            while (opModeIsActive() && getRuntime() - kdStart < KICKER_DOWN_WAIT_SEC) {
                launcherLeft.setVelocity(velTicksPerSec);
                launcherRight.setVelocity(velTicksPerSec);
                idle();
            }
        }

        // stop launcher after sequence
        launcherLeft.setVelocity(0);
        launcherRight.setVelocity(0);

        // commit temp state back to main state
        indexOrder = Arrays.copyOf(tempOrder, tempOrder.length);
        indexerPos = tempIndexerPos;

        telemetryDashboard.addData("ColorShoot", "Done; new indexOrder: " + Arrays.toString(indexOrder));
        telemetryDashboard.update();

        return true;
    }

    // ----------------- the rest of your original code (unchanged) -----------------
    private void handleDrive() {
        double scale = (gamepad1.right_trigger > 0.3) ? DRIVE_PRECISION_SCALE : 1.0;

        double y  = applyDeadzone(-gamepad1.left_stick_y);
        double x  = applyDeadzone( gamepad1.left_stick_x);
        double rx = applyDeadzone( gamepad1.right_stick_x);

        y  = cube(y);
        x  = cube(x);
        rx = cube(rx);

        if (gamepad1.b) headingOffset = drive.localizer.getPose().heading.toDouble();

        double heading = drive.localizer.getPose().heading.toDouble() - headingOffset;
        double cosH    = Math.cos(-heading);
        double sinH    = Math.sin(-heading);
        double rotX    =  x * cosH - y * sinH;
        double rotY    =  x * sinH + y * cosH;

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(rotY * scale, -rotX * scale),
                -rx * scale
        ));
    }

    private double applyDeadzone(double v) {
        return Math.abs(v) < DRIVE_DEADZONE ? 0.0 : v;
    }

    private double cube(double v) {
        return v * v * v;
    }

    private void shiftIndexerLarge() {
        boolean a = gamepad1.dpad_down;
        if (a && notShifting) {
            notShifting = false;
            int moveTicks = (int)(SORT_DEGREES/2.0 * COUNTS_PER_DEGREE);
            int target = sortMotor.getCurrentPosition() + moveTicks;
            sortMotor.setTargetPosition(target);
            sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sortMotor.setPower(SORT_POWER);
            shiftTimer = getRuntime();
            ballsIn++;
        }
        if (getRuntime() - shiftTimer >= 0.1) {
            notShifting = true;
        }
    }

    private void shiftIndexerSmall() {
        boolean a = gamepad1.dpad_down;
        if (a && notShifting) {
            notShifting = false;
            int moveTicks = (int)(SORT_DEGREES/2.0 * COUNTS_PER_DEGREE);
            int target = sortMotor.getCurrentPosition() + moveTicks;
            sortMotor.setTargetPosition(target);
            sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sortMotor.setPower(SORT_POWER);
            shiftTimer = getRuntime();
            ballsIn++;
        }
        if (getRuntime() - shiftTimer >= 0.1) {
            notShifting = true;
        }
    }
    
    
    // ================= SHOOTING (UNCHANGED) =================
    private void shootingSequence() {
        // The sequence is started by handlePresetAndFireInput() on the rising edge
        // of LB, B, Y or X (which set launcherRPM and hoodPosX before starting the sequence).

        if (!shootingActive) return;

        // Use selected launcherRPM preset (rpm) converted to ticks/sec
        double vel = (launcherRPM / 60.0) * TICKS_PER_REV;
        shootingActive = true;
        launcherLeft.setVelocity(vel);
        launcherRight.setVelocity(vel);

        switch (shootState) {

            case 0:
                if (!sortMotor.isBusy()) {
                    if (ballsIn <= 0) {
                        shootState = 5;
                        break;
                    }

                    int moveTicks = (int)((SORT_DEGREES / 4.0) * COUNTS_PER_DEGREE);
                    int target    = sortMotor.getCurrentPosition() + moveTicks;
                    sortMotor.setTargetPosition(target);
                    sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sortMotor.setPower(SORT_POWER);
                    indexerPos = (indexerPos+2)%3;

                    int targetSlot  = shootOrder[ballsLaunched];
                    int stepsNeeded = (targetSlot - indexerPos + 3) % 3;

                    if (stepsNeeded == 0) {
                        sorterTimer = getRuntime();
                        shootState  = 2;
                    } else {
                        moveTicks = (int)(stepsNeeded * (SORT_DEGREES / 2.0) * COUNTS_PER_DEGREE);
                        target    = sortMotor.getCurrentPosition() + moveTicks;
                        sortMotor.setTargetPosition(target);
                        sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        sortMotor.setPower(SORT_POWER);
                        indexerPos  = targetSlot;
                        sorterTimer = getRuntime();
                        shootState  = 1;
                    }
                }
                break;

            case 1:
                if (!sortMotor.isBusy()) {
                    sorterTimer = getRuntime();
                    shootState  = 2;
                }
                break;

            case 2:
                if (!sortMotor.isBusy() && getRuntime() - sorterTimer >= 0.35) {
                    liftLeft.setPosition(0.15);
                    liftRight.setPosition(0.85);
                    liftTimer  = getRuntime();
                    shootState = 3;
                }
                break;

            case 3:
                if (getRuntime() - liftTimer >= 1.00) {
                    liftLeft.setPosition(0.99);
                    liftRight.setPosition(0.01);
                    liftTimer     = getRuntime();
                    ballsLaunched++;
                    ballsIn--;
                    kickerDownTimer = getRuntime();
                    shootState = 4;
                }
                break;

            case 4:
                if (getRuntime() - kickerDownTimer >= KICKER_DOWN_WAIT_SEC) {
                    if (ballsIn <= 0 || ballsLaunched >= 3) {
                        shootState = 5;
                    } else {
                        shootState = 0;
                    }
                }
                break;

            case 5:
                launcherLeft.setVelocity(0);
                launcherRight.setVelocity(0);

                int moveTicks = (int)((SORT_DEGREES / 4.0) * COUNTS_PER_DEGREE);
                int target    = sortMotor.getCurrentPosition() + moveTicks;
                sortMotor.setTargetPosition(target);
                sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sortMotor.setPower(SORT_POWER);
                indexerPos    = (indexerPos + 2) % 3;

                shootingActive = false;
                ballsLaunched  = 0;
                indexOrder     = new int[]{2, 2, 2};
                break;
        }
    }

    // ================= INTAKE (UNCHANGED) =================
    private void intakeSequence() {
        boolean rb = gamepad1.right_bumper;
        if (rb && !lastRB && !intakeSequenceActive) {
            intakeSequenceActive = true;
            intakeState          = 0;
            ballsIn              = 0;
        }
        lastRB = rb;

        if (!intakeSequenceActive) return;

        switch (intakeState) {

            case 0:
                intakeMotor.setPower(-0.8);
                boolean greenDetected  = (colorSensor.green() > 115);
                boolean purpleDetected = (colorSensor.red() > 58 && colorSensor.blue() > 82);

                if (greenDetected || purpleDetected) {
                    intakeTimer = getRuntime();
                    if (ballsIn >= 0 && ballsIn < 3) {
                        if (greenDetected)  indexOrder[ballsIn] = 1;
                        if (purpleDetected) indexOrder[ballsIn] = 0;
                    }
                    intakeState = 1;
                }

                if (shootingActive) {
                    intakeSequenceActive = false;
                    intakeMotor.setPower(0);
                }
                break;

            case 1:
                if (getRuntime() - intakeTimer >= 0.75 && !sortMotor.isBusy()) {
                    int moveTicks = (int)((SORT_DEGREES / 2.0) * COUNTS_PER_DEGREE);
                    int target    = sortMotor.getCurrentPosition() + moveTicks;
                    sortMotor.setTargetPosition(target);
                    sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sortMotor.setPower(SORT_POWER);
                    indexerPos = (indexerPos + 1) % 3;
                    ballsIn++;
                    intakeState = 2;
                }
                break;

            case 2:
                if (!sortMotor.isBusy()) {
                    if (ballsIn >= 3) {
                        kickerDownTimer = getRuntime();
                        intakeState = 3;
                    } else {
                        intakeState = 0;
                    }
                }
                break;

            case 3:
               if (getRuntime() - kickerDownTimer >= KICKER_DOWN_WAIT_SEC && !sortMotor.isBusy()) {
                   intakeState = 4;
               }
                break;

            case 4:
                if (!sortMotor.isBusy()) {
                    sortMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    sortMotor.setPower(0);
                    intakeMotor.setPower(0);
                    intakeSequenceActive = false;
                }
                break;
        }
    }

    // ================= TELEMETRY =================
    private void updateTelemetry() {
        telemetryDashboard.addData("Shoot State",     shootState);
        telemetryDashboard.addData("Sorter position", sortMotor.getCurrentPosition());
        telemetryDashboard.addData("Sorter busy",     sortMotor.isBusy());
        telemetryDashboard.addData("Red",             colorSensor.red());
        telemetryDashboard.addData("Green",           colorSensor.green());
        telemetryDashboard.addData("Blue",           colorSensor.blue());
        telemetryDashboard.addData("Balls in",        ballsIn);
        telemetryDashboard.addData("Balls launched",  ballsLaunched);
        telemetryDashboard.addData("Ball Order",      Arrays.toString(indexOrder)); // print contents
        telemetryDashboard.addData("Indexer pos",     indexerPos);
        telemetryDashboard.addData("Shoot order",     Arrays.toString(shootOrder));
        telemetryDashboard.addData("Launcher RPM (selected)", launcherRPM);
        telemetryDashboard.addData("Hood position (selected)", hoodPosX);
        telemetryDashboard.addData("Motif Detected", motifDetected ? ("ID=" + motifTagId + " pat=" + Arrays.toString(motifPattern)) : "none");
        telemetry.addData("Drive",    "LS fwd/strafe | RS rotate");
        telemetryDashboard.update();
    }
}
