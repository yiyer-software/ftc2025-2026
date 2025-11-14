package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "FTCDecodeCompetitionTeleOpManualPID_THEGREAT_V2", group = "Competition")
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
    private final double FLAP_UP_POS = 0.5;
    private final double FLAP_DOWN_POS = 0.0;

    // Drive constants
    private final double DRIVE_RPM = 2500;
    private final double DRIVE_TICKS_PER_SEC = (DRIVE_RPM / 60.0) * TICKS_PER_REV; // ≈ 1166.67

    // --- Wheel power multipliers for fine-tuning ---
    private double wheelPowerFL = 1.0;
    private double wheelPowerFR = 1.0;
    private double wheelPowerBL = 0.98;
    private double wheelPowerBR = 0.96;

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
            mecanumDrive();           // 🚗 Perfect, tunable drive
            controlIntake();          // 🍽️ Intake toggle
            controlFlaps();           // 🪶 Flap control
            updateLauncherVelocity(); // 🚀 Launcher RPM constant

            // Telemetry
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Left Launcher RPM", launcherLeft.getVelocity() / TICKS_PER_REV * 60.0);
            telemetry.addData("Right Launcher RPM", launcherRight.getVelocity() / TICKS_PER_REV * 60.0);
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

        battery = hardwareMap.voltageSensor.iterator().next();
    }

    /** 🚗 Mecanum drive with tunable wheel powers **/
    private void mecanumDrive() {
        double y = -gamepad1.left_stick_y; // forward/back
        double x = gamepad1.left_stick_x * 1.1; // strafe correction
        double rx = -gamepad1.right_stick_x; // rotation

        double DEADZONE = 0.05;
        if (Math.abs(y) < DEADZONE) y = 0;
        if (Math.abs(x) < DEADZONE) x = 0;
        if (Math.abs(rx) < DEADZONE) rx = 0;

        double speedScale = gamepad1.left_trigger > 0.5 ? 0.4 : 1.0;

        // Mecanum formula
        double fl = (y + x + rx) * speedScale * wheelPowerFL;
        double fr = (y - x - rx) * speedScale * wheelPowerFR;
        double bl = (y - x + rx) * speedScale * wheelPowerBL;
        double br = (y + x - rx) * speedScale * wheelPowerBR;

        // Normalize powers
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        // Stop immediately if sticks are released
        if (Math.abs(y) < DEADZONE && Math.abs(x) < DEADZONE && Math.abs(rx) < DEADZONE) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return;
        }

        // Apply precise velocity control
        frontLeft.setVelocity(fl * DRIVE_TICKS_PER_SEC);
        frontRight.setVelocity(fr * DRIVE_TICKS_PER_SEC);
        backLeft.setVelocity(bl * DRIVE_TICKS_PER_SEC);
        backRight.setVelocity(br * DRIVE_TICKS_PER_SEC);

        telemetry.addData("Speed Mode", speedScale == 1.0 ? "FAST" : "SLOW");
    }

    /** 🚀 Voltage-compensated launcher RPM **/
    private void updateLauncherVelocity() {
        if (gamepad1.right_trigger > 0.5) targetRPM = highRPM;
        else if (gamepad1.left_trigger > 0.5) targetRPM = lowRPM;
        else targetRPM = 0;

        double batteryVoltage = battery.getVoltage();
        double voltageCompensation = 12.0 / batteryVoltage; // maintain constant RPM

        if (targetRPM > 50) {
            double velocityTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REV * voltageCompensation;
            launcherLeft.setVelocity(velocityTicksPerSec);
            launcherRight.setVelocity(velocityTicksPerSec);
        } else {
            launcherLeft.setVelocity(0);
            launcherRight.setVelocity(0);
        }
    }

    /** 🍽️ Intake toggle **/
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

    /** 🪶 Flap control **/
    private void controlFlaps() {
        if (gamepad1.left_bumper && !lastFlapToggle) {
            flapUp = !flapUp;
            if (flapUp) flapTimer = System.currentTimeMillis();
        }
        lastFlapToggle = gamepad1.left_bumper;

        if (flapUp && System.currentTimeMillis() - flapTimer > 300) flapUp = false;

        liftLeft.setPosition(flapUp ? FLAP_UP_POS : FLAP_DOWN_POS);
        liftRight.setPosition(flapUp ? 1 - FLAP_UP_POS : 1 - FLAP_DOWN_POS);

        telemetry.addData("Flap", flapUp ? "UP" : "DOWN");
    }

    // --- Wheel power setters for tuning ---
    public void setWheelPowers(double fl, double fr, double bl, double br) {
        wheelPowerFL = fl;
        wheelPowerFR = fr;
        wheelPowerBL = bl;
        wheelPowerBR = br;
    }
}
