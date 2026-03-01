package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TeleOp_Main — Full Robot Controller
 *
 * ── GAMEPAD 1 (Driver) ──────────────────────────────────
 *   Left  Stick  Y/X     → Forward / Strafe  (robot-centric)
 *   Right Stick  X       → Rotate
 *   Left  Bumper (hold)  → Precision / Slow mode (40 %)
 *   Right Trigger        → Flywheel HIGH speed (3 500 RPM)
 *   Left  Trigger        → Flywheel LOW  speed (2 500 RPM)
 *   (neither trigger)    → Flywheel OFF
 *   X Button             → Auto-Align with Obelisk (Limelight)
 *
 * ── GAMEPAD 2 (Operator) ────────────────────────────────
 *   A Button             → Intake toggle ON / OFF
 *   B Button (hold)      → Intake REVERSE
 *   Right Stick Y        → Hood UP / DOWN  (fast continuous)
 *   D-Pad Right          → Sorter forward  one fixed step
 *   D-Pad Left           → Sorter backward one fixed step
 *   Right Bumper         → Manual FIRE (kicker)
 *   Y Button             → AUTO-INTAKE sequence (3 balls, store colors)
 *   X Button             → AUTO-LAUNCH sequence (3 balls)
 */
@TeleOp(name = "TeleOp_Main", group = "TeleOp")
public class TeleOp_Main extends LinearOpMode {

    // ── Hardware ──────────────────────────────────────────
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx intakeMotor;
    private DcMotorEx sortMotor;
    private DcMotorEx launcherLeft, launcherRight;
    private Servo     liftLeft, liftRight;
    private Servo     hoodLeft,  hoodRight;
    private ColorSensor        colorSensor;
    private LimelightSubSystem limelight;

    // ── Physical constants ────────────────────────────────
    /** GoBilda / REV 28-tick motors used for the flywheel */
    private static final double TICKS_PER_REV_FLYWHEEL = 28.0;
    /** Sort-motor encoder ticks per revolution (e.g., NeveRest 40 = 1 120 + gearing) */
    private static final double COUNTS_PER_REV          = 1378.0;
    private static final double COUNTS_PER_DEGREE        = COUNTS_PER_REV / 360.0;
    /** Angular sweep per sorter step (half-revolution of the indexer drum) */
    private static final double SORT_STEP_DEGREES        = 173.0;

    // ── Tunable constants ─────────────────────────────────
    private static final double FLYWHEEL_HIGH_RPM  = 3500.0;
    private static final double FLYWHEEL_LOW_RPM   = 2500.0;
    private static final double SORT_POWER         = 0.9;   // high power for fast indexing
    /** Hood increment per control loop (~20 ms) — larger = faster */
    private static final double HOOD_INCREMENT      = 0.012;
    /** P-gain for limelight auto-align rotation */
    private static final double ALIGN_KP            = 0.035;
    private static final double ALIGN_MAX_POWER     = 0.50;
    private static final double ALIGN_DEADBAND_DEG  = 1.5;

    // ── Color codes stored in ballColors[] ───────────────
    private static final int COLOR_PURPLE = 0;
    private static final int COLOR_GREEN  = 1;
    private static final int COLOR_EMPTY  = 2;

    // ── Runtime state ─────────────────────────────────────
    private double  hoodPosition   = 0.5;
    private boolean intakeOn       = false;

    // Ball storage (3 slots, indexed by indexerPos)
    private final int[] ballColors = {COLOR_EMPTY, COLOR_EMPTY, COLOR_EMPTY};
    private int         ballsStored = 0;
    private int         indexerPos  = 0;

    // Auto-intake sequence
    private boolean autoIntakeActive = false;
    private int     aiState          = 0;

    // Auto-launch sequence
    private boolean autoLaunchActive = false;
    private int     alState          = 0;
    private int     alShotsFired     = 0;

    // Edge-detection toggles
    private boolean prevA           = false;
    private boolean prevY           = false;
    private boolean prevX2          = false;
    private boolean prevDpadLeft    = false;
    private boolean prevDpadRight   = false;
    private boolean prevRB2         = false;

    private final ElapsedTime seqTimer  = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    // ─────────────────────────────────────────────────────
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        telemetry.addData("Status", "Ready — waiting for Start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            loopTimer.reset();

            // ── Gamepad 1: Drive + flywheel + align ──────
            robotCentricDrive();
            flywheelControl();
            autoAlignControl();

            // ── Gamepad 2: Mechanisms ─────────────────────
            hoodControl();
            intakeManualControl();
            sorterManualControl();
            kickerManualControl();
            sequenceButtonCheck();

            // ── Active sequences ──────────────────────────
            if (autoIntakeActive) runAutoIntake();
            if (autoLaunchActive) runAutoLaunch();

            showTelemetry();
        }
    }

    // ══════════════════════════════════════════════════════
    //  INIT
    // ══════════════════════════════════════════════════════
    private void initHardware() {
        // Drive
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{leftFront, rightFront, leftBack, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // raw power for max drive speed
        }

        // Launcher
        launcherLeft  = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        launcherRight = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Sorter
        sortMotor = hardwareMap.get(DcMotorEx.class, "sortMotor");
        sortMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sortMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos
        liftLeft  = hardwareMap.get(Servo.class, "liftLeft");
        liftRight = hardwareMap.get(Servo.class, "liftRight");
        hoodLeft  = hardwareMap.get(Servo.class, "leftHood");
        hoodRight = hardwareMap.get(Servo.class, "rightHood");

        liftLeft.setPosition(0.99);
        liftRight.setPosition(0.01);
        hoodLeft.setPosition(hoodPosition);
        hoodRight.setPosition(1.0 - hoodPosition);

        // Sensors
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        limelight   = new LimelightSubSystem(hardwareMap);
    }

    // ══════════════════════════════════════════════════════
    //  DRIVE  (robot-centric, max speed via raw motor power)
    // ══════════════════════════════════════════════════════
    private void robotCentricDrive() {

        double y  = -gamepad1.left_stick_y;   // forward/back
        double x  =  gamepad1.left_stick_x;   // strafe
        double rx =  gamepad1.right_stick_x;  // rotate

        // Precision mode: hold left bumper → 40 % speed
        double speed = gamepad1.left_bumper ? 0.40 : 1.0;

        double lf = (y + x + rx) * speed;
        double rf = (y - x - rx) * speed;
        double lb = (y - x + rx) * speed;
        double rb = (y + x - rx) * speed;

        // Normalise so max magnitude = 1
        double maxVal = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lb), Math.abs(rb)))));

        leftFront.setPower(lf  / maxVal);
        rightFront.setPower(rf / maxVal);
        leftBack.setPower(lb   / maxVal);
        rightBack.setPower(rb  / maxVal);
    }

    // ══════════════════════════════════════════════════════
    //  FLYWHEEL  (RT = high, LT = low, neither = off)
    // ══════════════════════════════════════════════════════
    private void flywheelControl() {
        if (autoLaunchActive) return; // auto-launch manages flywheel itself

        double rt = gamepad1.right_trigger;
        double lt = gamepad1.left_trigger;

        if (rt > 0.05) {
            setFlywheelRPM(FLYWHEEL_HIGH_RPM);
        } else if (lt > 0.05) {
            setFlywheelRPM(FLYWHEEL_LOW_RPM);
        } else {
            launcherLeft.setVelocity(0);
            launcherRight.setVelocity(0);
        }
    }

    private void setFlywheelRPM(double rpm) {
        double tps = (rpm / 60.0) * TICKS_PER_REV_FLYWHEEL;
        launcherLeft.setVelocity(tps);
        launcherRight.setVelocity(tps);
    }

    // ══════════════════════════════════════════════════════
    //  AUTO-ALIGN  (Gamepad1 X — P-loop on Limelight tx)
    // ══════════════════════════════════════════════════════
    private void autoAlignControl() {
        if (!gamepad1.x) return;

        LimelightSubSystem.AprilTagData tag = limelight.getAprilTagData();
        if (tag == null || !tag.tagVisible) {
            telemetry.addData("AutoAlign", "No target visible");
            return;
        }

        // tx: horizontal angle error (degrees).
        // Positive tx → target is to the RIGHT → rotate CW (positive rx).
        // Assumes LimelightSubSystem.AprilTagData exposes a `tx` field (degrees).
        // If your implementation uses a different field name, rename here.
        double tx = tag.tx;

        if (Math.abs(tx) < ALIGN_DEADBAND_DEG) {
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            telemetry.addData("AutoAlign", "LOCKED  tx=%.1f°", tx);
            return;
        }

        double rotPow = tx * ALIGN_KP;
        rotPow = Math.max(-ALIGN_MAX_POWER, Math.min(ALIGN_MAX_POWER, rotPow));

        // Pure rotation: left side forward, right side backward (or vice-versa)
        leftFront.setPower(-rotPow);
        leftBack.setPower(-rotPow);
        rightFront.setPower(rotPow);
        rightBack.setPower(rotPow);

        telemetry.addData("AutoAlign", "Aligning  tx=%.1f°  pwr=%.2f", tx, rotPow);
    }

    // ══════════════════════════════════════════════════════
    //  HOOD  (Gamepad2 right stick Y — fast continuous)
    // ══════════════════════════════════════════════════════
    private void hoodControl() {
        double input = -gamepad2.right_stick_y;
        if (Math.abs(input) > 0.05) {
            hoodPosition += input * HOOD_INCREMENT;
            hoodPosition  = Math.max(0.0, Math.min(1.0, hoodPosition));
            hoodLeft.setPosition(hoodPosition);
            hoodRight.setPosition(1.0 - hoodPosition);
        }
    }

    // ══════════════════════════════════════════════════════
    //  INTAKE  (A = toggle, B held = reverse)
    // ══════════════════════════════════════════════════════
    private void intakeManualControl() {
        if (autoIntakeActive) return; // sequence controls intake

        // B held → reverse (overrides toggle)
        if (gamepad2.b) {
            setIntakeVelocityFraction(+1.0); // +1.0 = reverse direction
            intakeOn = false;
            prevA    = gamepad2.a;
            return;
        }

        // A rising edge → toggle
        if (gamepad2.a && !prevA) {
            intakeOn = !intakeOn;
        }
        prevA = gamepad2.a;

        setIntakeVelocityFraction(intakeOn ? -1.0 : 0.0); // -1.0 = intake-in direction
    }

    /** fraction: -1 = full intake-in, +1 = full reverse, 0 = stop */
    private void setIntakeVelocityFraction(double fraction) {
        // Max encoder velocity for RUN_USING_ENCODER — tune INTAKE_MAX_TPS for your motor
        final double INTAKE_MAX_TPS = 2800.0;
        intakeMotor.setVelocity(fraction * INTAKE_MAX_TPS);
    }

    // ══════════════════════════════════════════════════════
    //  SORTER  (DPad left/right — fixed step each press)
    // ══════════════════════════════════════════════════════
    private void sorterManualControl() {
        if (autoIntakeActive || autoLaunchActive) return;

        boolean dRight = gamepad2.dpad_right;
        boolean dLeft  = gamepad2.dpad_left;

        if (dRight && !prevDpadRight && !sortMotor.isBusy()) {
            sorterStep(+1); // forward one slot
        }
        if (dLeft && !prevDpadLeft && !sortMotor.isBusy()) {
            sorterStep(-1); // backward one slot
        }

        prevDpadRight = dRight;
        prevDpadLeft  = dLeft;
    }

    /**
     * Move the sorter ±1 slot.
     * @param direction +1 = forward, -1 = backward
     */
    private void sorterStep(int direction) {
        int moveTicks = (int) ((SORT_STEP_DEGREES / 2.0) * COUNTS_PER_DEGREE);
        int target    = sortMotor.getCurrentPosition() + direction * moveTicks;
        sortMotor.setTargetPosition(target);
        sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sortMotor.setPower(SORT_POWER);
        indexerPos = ((indexerPos + direction) % 3 + 3) % 3;
    }

    // ══════════════════════════════════════════════════════
    //  KICKER  (Gamepad2 Right Bumper — fire one shot)
    // ══════════════════════════════════════════════════════
    private void kickerManualControl() {
        if (autoLaunchActive) return;

        boolean rb = gamepad2.right_bumper;
        if (rb && !prevRB2) {
            // Press → extend kicker
            liftLeft.setPosition(0.25);
            liftRight.setPosition(0.85);
        }
        if (!rb && prevRB2) {
            // Release → retract kicker
            liftLeft.setPosition(0.99);
            liftRight.setPosition(0.01);
            if (ballsStored > 0) ballsStored--;
        }
        prevRB2 = rb;
    }

    // ══════════════════════════════════════════════════════
    //  SEQUENCE BUTTON CHECK
    // ══════════════════════════════════════════════════════
    private void sequenceButtonCheck() {
        // Only start a new sequence when none is active
        // Gamepad2 Y → auto-intake 3 balls
        boolean yBtn = gamepad2.y;
        if (yBtn && !prevY && !autoIntakeActive && !autoLaunchActive) {
            startAutoIntake();
        }
        prevY = yBtn;

        // Gamepad2 X → auto-launch 3 balls
        boolean x2Btn = gamepad2.x;
        if (x2Btn && !prevX2 && !autoIntakeActive && !autoLaunchActive) {
            startAutoLaunch();
        }
        prevX2 = x2Btn;
    }

    // ══════════════════════════════════════════════════════
    //  AUTO-INTAKE SEQUENCE
    //  Intakes up to 3 balls; stores color in ballColors[].
    //  State machine so it never blocks the main loop.
    // ══════════════════════════════════════════════════════
    private void startAutoIntake() {
        autoIntakeActive = true;
        aiState          = 0;
        ballsStored      = 0;
        ballColors[0]    = COLOR_EMPTY;
        ballColors[1]    = COLOR_EMPTY;
        ballColors[2]    = COLOR_EMPTY;
        seqTimer.reset();
        intakeOn = false;
    }

    private void runAutoIntake() {
        switch (aiState) {

            case 0: // ── Run intake, wait for colour detection ──────
                setIntakeVelocityFraction(-1.0);

                boolean green  = colorSensor.green() > 100;
                boolean purple = colorSensor.red() > 55 && colorSensor.blue() > 80;

                if (green || purple) {
                    int destSlot = (indexerPos + 1) % 3;
                    ballColors[destSlot] = green ? COLOR_GREEN : COLOR_PURPLE;
                    seqTimer.reset();
                    aiState = 1;
                }

                // Timeout: give up after 7 s waiting for a ball
                if (seqTimer.seconds() > 7.0) {
                    endAutoIntake();
                }
                break;

            case 1: // ── Short dwell so ball travels to sorter ───────
                if (seqTimer.seconds() >= 0.75 && !sortMotor.isBusy()) {
                    // Advance sorter one step to lock ball into slot
                    int moveTicks = (int) ((SORT_STEP_DEGREES / 2.0) * COUNTS_PER_DEGREE);
                    sortMotor.setTargetPosition(sortMotor.getCurrentPosition() + moveTicks);
                    sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sortMotor.setPower(SORT_POWER);
                    indexerPos = (indexerPos + 1) % 3;
                    ballsStored++;
                    seqTimer.reset();
                    aiState = 2;
                }
                break;

            case 2: // ── Wait for sorter, decide continue or done ────
                if (!sortMotor.isBusy()) {
                    if (ballsStored >= 3) {
                        endAutoIntake();
                    } else {
                        seqTimer.reset();
                        aiState = 0;
                    }
                }
                break;
        }
    }

    private void endAutoIntake() {
        setIntakeVelocityFraction(0);
        autoIntakeActive = false;
    }

    // ══════════════════════════════════════════════════════
    //  AUTO-LAUNCH SEQUENCE
    //  Spins flywheel, fires all stored balls one at a time.
    // ══════════════════════════════════════════════════════
    private void startAutoLaunch() {
        autoLaunchActive = true;
        alState          = 0;
        alShotsFired     = 0;
        seqTimer.reset();
        setFlywheelRPM(FLYWHEEL_HIGH_RPM);
    }

    private void runAutoLaunch() {
        // Keep flywheel at speed throughout sequence
        setFlywheelRPM(FLYWHEEL_HIGH_RPM);

        switch (alState) {

            case 0: // ── Wait for flywheel to spin up (0.6 s) ────────
                if (seqTimer.seconds() >= 0.60) {
                    alState = 1;
                    seqTimer.reset();
                }
                break;

            case 1: // ── Check if there are balls to fire ───────────
                if (ballsStored <= 0 || alShotsFired >= 3) {
                    alState = 4; // done
                } else {
                    // Extend kicker
                    liftLeft.setPosition(0.35);
                    liftRight.setPosition(0.65);
                    seqTimer.reset();
                    alState = 2;
                }
                break;

            case 2: // ── Hold kicker extended ─────────────────────
                if (seqTimer.seconds() >= 0.75) {
                    liftLeft.setPosition(0.99);
                    liftRight.setPosition(0.01);
                    alShotsFired++;
                    if (ballsStored > 0) ballsStored--;
                    seqTimer.reset();
                    alState = 3;
                }
                break;

            case 3: // ── Brief pause between shots ─────────────────
                if (seqTimer.seconds() >= 0.40) {
                    alState = 1; // fire next ball
                }
                break;

            case 4: // ── Sequence complete ─────────────────────────
                launcherLeft.setVelocity(0);
                launcherRight.setVelocity(0);
                autoLaunchActive = false;
                break;
        }
    }

    // ══════════════════════════════════════════════════════
    //  TELEMETRY
    // ══════════════════════════════════════════════════════
    private void showTelemetry() {
        telemetry.addLine("── GAMEPAD 1 ──────────────────────");
        telemetry.addData("Drive",    "LS fwd/strafe | RS rotate");
        telemetry.addData("Speed",    "Normal (hold LB = slow 40%%)");
        telemetry.addData("Flywheel", "RT=%.0f RPM  LT=%.0f RPM  (off=release)",
                FLYWHEEL_HIGH_RPM, FLYWHEEL_LOW_RPM);
        telemetry.addData("AutoAlign","X button");

        telemetry.addLine("── GAMEPAD 2 ──────────────────────");
        telemetry.addData("Intake",   "A=toggle  B(hold)=reverse");
        telemetry.addData("Hood",     "Right Stick Y");
        telemetry.addData("Sorter",   "DPad R=fwd step  DPad L=bk step");
        telemetry.addData("Fire",     "Right Bumper (manual 1 shot)");
        telemetry.addData("Sequence", "Y=AutoIntake3  X=AutoLaunch3");

        telemetry.addLine("── STATUS ─────────────────────────");
        telemetry.addData("Flywheel L/R TPS", "%.0f / %.0f",
                launcherLeft.getVelocity(), launcherRight.getVelocity());
        telemetry.addData("Hood Pos",    "%.3f", hoodPosition);
        telemetry.addData("Intake On",   intakeOn);
        telemetry.addData("Balls Stored",ballsStored);
        telemetry.addData("Ball Colors", "[0]=%s [1]=%s [2]=%s",
                colorName(ballColors[0]), colorName(ballColors[1]), colorName(ballColors[2]));
        telemetry.addData("IndexerPos",  indexerPos);
        telemetry.addData("AutoIntake",  autoIntakeActive ? "RUNNING state=" + aiState : "idle");
        telemetry.addData("AutoLaunch",  autoLaunchActive ? "RUNNING shot=" + alShotsFired : "idle");
        telemetry.update();
    }

    private String colorName(int code) {
        switch (code) {
            case COLOR_PURPLE: return "PURPLE";
            case COLOR_GREEN:  return "GREEN";
            default:           return "empty";
        }
    }
}
