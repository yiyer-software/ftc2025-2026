package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoV5", group = "Competition")
public class AutoV5 extends LinearOpMode {

    // ---------------- CONFIG ----------------

    // Launcher
    private double LAUNCHER_RPM = -3000;
    private long FIRST_LAUNCHER_SPINUP_MS = 3000;
    private long SUBSEQUENT_LAUNCHER_SPINUP_MS = 500;

    // Flaps

    private double FLAP_UP_POS = 0.50;
    private double FLAP_DOWN_POS = -0.25;
    private long FLAP_UP_DURATION_MS = 300;

    // Intake
    private double INTAKE_POWER = 1.0;
    private long INTAKE_STEP4_MS = 860;
    private long INTAKE_STEP7_MS = 3000;

    // Drive
    private double DRIVE_POWER = 0.55;      // Smooth speed

    // ---------------------------------------

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx launcherLeft, launcherRight;
    private DcMotorEx intakeMotor;
    private Servo liftLeft, liftRight;

    private boolean firstLauncherSpinupDone = false;

    @Override
    public void runOpMode() {

        // ---- Hardware Mapping ----
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        liftLeft = hardwareMap.get(Servo.class, "liftServoLeft");
        liftRight = hardwareMap.get(Servo.class, "liftServoRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (!opModeIsActive()) return;

        // ---------------------------------------------------------
        //                AUTONOMOUS SEQUENCE
        // ---------------------------------------------------------

        stopLauncher();

        // Step 1 — small reverse
        driveMS(-DRIVE_POWER, 150);   // ~12 inches

        // Step 2 — spin up launcher
        startLauncher();

        // Step 3 — flap bump
        flapPulse();

        // Step 4 — intake to load first ring
        intakeMotor.setPower(INTAKE_POWER);
        sleep(INTAKE_STEP4_MS);
        intakeMotor.setPower(0);

        // Step 6 — flap bump again

        flapPulse();

        // Step 7 — intake before final shoot
        intakeMotor.setPower(INTAKE_POWER);
        sleep(INTAKE_STEP7_MS);
        intakeMotor.setPower(0);

        // Step 7c — final flap + stop launcher
        flapPulse();
        stopLauncher();
        // --- TURN FOR 250 ms ---
        frontLeft.setPower(-0.5);
        backLeft.setPower(-0.5);
        frontRight.setPower(0.5);
        backRight.setPower(0.5);

        sleep(250);

// stop
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);



        // Step 8 — final long reverse
        //driveMS(-DRIVE_POWER, 700);  // ~84 inches
        frontLeft.setPower(1);
        backLeft.setPower(-1);

        frontRight.setPower(-1);
        backRight.setPower(1);

        sleep(700);
        frontLeft.setPower(0);
        backLeft.setPower(0);

        frontRight.setPower(0);
        backRight.setPower(0);
    }

    // ---------------------------------------------------------
    //                 UTILITY FUNCTIONS
    // ---------------------------------------------------------

    private void startLauncher() {
        double vel = (LAUNCHER_RPM / 60.0) * 28.0;
        launcherLeft.setVelocity(vel);
        launcherRight.setVelocity(vel);

        if (!firstLauncherSpinupDone) {
            sleep(FIRST_LAUNCHER_SPINUP_MS);
            firstLauncherSpinupDone = true;
        } else {
            sleep(SUBSEQUENT_LAUNCHER_SPINUP_MS);
        }
    }

    private void stopLauncher() {
        launcherLeft.setVelocity(0);
        launcherRight.setVelocity(0);
    }

    private void flapPulse() {
        liftLeft.setPosition(FLAP_UP_POS);
        liftRight.setPosition(1 - FLAP_UP_POS);
        sleep(FLAP_UP_DURATION_MS);
        liftLeft.setPosition(FLAP_DOWN_POS);
        liftRight.setPosition(1 - FLAP_DOWN_POS);
    }

    // ---------------------------------------------------------
    //               TIMED SMOOTH DRIVE
    // ---------------------------------------------------------
    private void driveMS(double power, long timeMS) {
        // Ramp up
        long steps = 10;
        long RAMP_TIME_MS = 100;
        for (int i = 1; i <= steps; i++) {
            double p = (power * i / steps);
            setDrivePower(p);
            sleep(RAMP_TIME_MS / steps);
        }

        // Hold
        setDrivePower(power);
        sleep(timeMS);

        // Ramp down
        for (int i = 1; i <= steps; i++) {
            double p = power - (power * i / steps);
            setDrivePower(p);
            sleep(RAMP_TIME_MS / steps);
        }

        setDrivePower(0);
    }

    private void setDrivePower(double p) {
        frontLeft.setPower(p);
        frontRight.setPower(p);
        backLeft.setPower(p);
        backRight.setPower(p);
    }
}