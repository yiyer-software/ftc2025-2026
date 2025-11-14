package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "FTCDecodeCompetitionTeleOpManualPID_THEGREAT", group = "Competition")
public class FTCDecodeCompetitionTeleOpManualPID extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeMotor;
    private DcMotorEx launcherLeft, launcherRight;
    private Servo liftLeft, liftRight;
    private VoltageSensor battery;

    // Launcher RPM targets
    private double highRPM = 3800;
    private double lowRPM = 3500;
    private double targetRPM = 0;

    private static final double TICKS_PER_REV = 28.0;

    // Intake toggle
    private boolean intakeToggle = false;
    private boolean lastIntakeToggle = false;

    // Flaps
    private boolean flapUp = false;
    private boolean lastFlapToggle = false;
    private long flapTimer = 0;
    private final double FLAP_UP_POS = 0.5; // slightly less lift
    private final double FLAP_DOWN_POS = 0.0;

    // Drive constants
    private final double DRIVE_RPM = 2500;
    private final double DRIVE_TICKS_PER_SEC = (DRIVE_RPM / 60.0) * TICKS_PER_REV; // ≈ 1166.67

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
            mecanumDrive();      // 🚗 Perfect instant drive control
            controlIntake();     // 🍽️ Intake toggle
            controlFlaps();      // 🪶 Flap control
            updateLauncherVelocity(); // 🚀 Launcher RPM

            telemetry.addData("Target RPM", targetRPM);
            double leftVelTicksPerSec = launcherLeft.getVelocity();
            double rightVelTicksPerSec = launcherRight.getVelocity();
            double leftRPM = leftVelTicksPerSec / TICKS_PER_REV * 60.0;
            double rightRPM = rightVelTicksPerSec / TICKS_PER_REV * 60.0;
            telemetry.addData("Left RPM", leftRPM);
            telemetry.addData("Right RPM", rightRPM);
            telemetry.update();
        }
    }

    private void initHardware() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");
        liftLeft = hardwareMap.get(Servo.class, "liftServoLeft");
        liftRight = hardwareMap.get(Servo.class, "liftServoRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        battery = hardwareMap.voltageSensor.iterator().next(); // telemetry only
    }

    /** 🚗 Perfect, instant-response mecanum drive with smooth velocity control **/
    private void mecanumDrive() {
        double y = -gamepad1.left_stick_y; // forward/back
        double x = gamepad1.left_stick_x * 1.1; // strafe correction
        double rx = -gamepad1.right_stick_x; // rotation

        // Deadzone to prevent drift
        double DEADZONE = 0.05;
        if (Math.abs(y) < DEADZONE) y = 0;
        if (Math.abs(x) < DEADZONE) x = 0;
        if (Math.abs(rx) < DEADZONE) rx = 0;

        // Optional slow mode (hold left trigger)
        double speedScale = gamepad1.left_trigger > 0.5 ? 0.4 : 1.0;

        // Mecanum formula
        double fl = (y + x + rx) * speedScale;
        double fr = (y - x - rx) * speedScale;
        double bl = (y - x + rx) * speedScale;
        double br = (y + x - rx) * speedScale;

        // Normalize powers
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        // Direct velocity control — instant and precise
        frontLeft.setVelocity(fl * DRIVE_TICKS_PER_SEC);
        frontRight.setVelocity(fr * DRIVE_TICKS_PER_SEC);
        backLeft.setVelocity(bl * DRIVE_TICKS_PER_SEC);
        backRight.setVelocity(br * DRIVE_TICKS_PER_SEC);

        telemetry.addData("Speed Mode", speedScale == 1.0 ? "FAST" : "SLOW");
    }

    /** 🚀 Launcher velocity control (RPM-based) **/
    private void updateLauncherVelocity() {
        if (gamepad1 != null) {
            if (gamepad1.right_trigger > 0.5) targetRPM = highRPM;
            else if (gamepad1.left_trigger > 0.5) targetRPM = lowRPM;
            else targetRPM = 0;
        }

        if (targetRPM > 50) {
            double ticksPerSecond = (targetRPM / 60.0) * TICKS_PER_REV;
            launcherLeft.setVelocity(ticksPerSecond);
            launcherRight.setVelocity(ticksPerSecond);
        } else {
            launcherLeft.setVelocity(0.0);
            launcherRight.setVelocity(0.0);
        }
    }

    /** 🍽️ Intake control (instant toggle) **/
    private void controlIntake() {
        if (gamepad1.right_bumper && !lastIntakeToggle) intakeToggle = !intakeToggle;
        lastIntakeToggle = gamepad1.right_bumper;

        double intakePower = 0.0;
        if (gamepad1.a) intakePower = 1.0;
        else if (gamepad1.b) intakePower = -1.0;
        else if (intakeToggle) intakePower = 1.0;

        intakeMotor.setPower(intakePower);
        telemetry.addData("Intake Power", intakePower);
    }

    /** 🪶 Flap control (toggle + timed auto-reset) **/
    private void controlFlaps() {
        if (gamepad1.left_bumper && !lastFlapToggle) {
            flapUp = !flapUp;
            if (flapUp) flapTimer = System.currentTimeMillis();
        }
        lastFlapToggle = gamepad1.left_bumper;

        if (flapUp && System.currentTimeMillis() - flapTimer > 500) flapUp = false;

        liftLeft.setPosition(flapUp ? FLAP_UP_POS : FLAP_DOWN_POS);
        liftRight.setPosition(flapUp ? 1 - FLAP_UP_POS : 1 - FLAP_DOWN_POS);

        telemetry.addData("Flap", flapUp ? "UP" : "DOWN");
    }
}