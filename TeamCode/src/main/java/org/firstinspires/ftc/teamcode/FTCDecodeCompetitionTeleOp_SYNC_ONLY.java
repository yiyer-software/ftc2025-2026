package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "FTCDecodeCompetitionTeleOp_SYNC_ONLY", group = "Competition")
public class FTCDecodeCompetitionTeleOp_SYNC_ONLY extends LinearOpMode {

    // Drive motors
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // Intake
    private DcMotor intakeMotor;

    // Launcher flywheels
    private DcMotorEx launcherLeft, launcherRight;

    // Flaps
    private Servo liftLeft, liftRight;

    private VoltageSensor battery;

    private static final double TICKS_PER_REV = 28.0;

    // Shooter RPM
    private double highRPM = 3800;
    private double lowRPM = 3500;
    private double targetRPM = 0;

    // Intake toggle
    private boolean intakeToggle = false;
    private boolean lastIntakeToggle = false;

    // Flap control
    private boolean flapUp = false;
    private boolean lastFlapToggle = false;
    private long flapTimer = 0;
    private final double FLAP_UP_POS = 0.5;
    private final double FLAP_DOWN_POS = 0.0;

    // Drive synchro velocity defaults
    private final double DRIVE_RPM = 2500;
    private final double DRIVE_TICKS_PER_SEC = (DRIVE_RPM / 60.0) * TICKS_PER_REV;

    @Override
    public void runOpMode() {
        initHardware();

        waitForStart();

        while (opModeIsActive()) {
            mecanumDriveSync();
            controlIntake();
            controlFlaps();
            updateLauncherVelocitySync();
            updateTelemetry();
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

        // Motor directions
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Encoder control
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        battery = hardwareMap.voltageSensor.iterator().next();
    }

    /** PERFECT SYNCHRONIZED MECANUM DRIVE USING VELOCITY **/
    private void mecanumDriveSync() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;

        double DEADZONE = 0.05;
        if (Math.abs(y) < DEADZONE) y = 0;
        if (Math.abs(x) < DEADZONE) x = 0;
        if (Math.abs(rx) < DEADZONE) rx = 0;

        double speedScale = gamepad1.left_trigger > 0.5 ? 0.4 : 1.0;

        double fl = (y + x + rx) * speedScale;
        double fr = (y - x - rx) * speedScale;
        double bl = (y - x + rx) * speedScale;
        double br = (y + x - rx) * speedScale;

        // Normalize motor powers
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max; fr /= max; bl /= max; br /= max;

        // Immediate stop if driver lets go
        if (Math.abs(y) < DEADZONE && Math.abs(x) < DEADZONE && Math.abs(rx) < DEADZONE) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return;
        }

        // Convert normalized values to velocity in ticks/sec
        frontLeft.setVelocity(fl * DRIVE_TICKS_PER_SEC);
        frontRight.setVelocity(fr * DRIVE_TICKS_PER_SEC);
        backLeft.setVelocity(bl * DRIVE_TICKS_PER_SEC);
        backRight.setVelocity(br * DRIVE_TICKS_PER_SEC);

        telemetry.addData("Drive Mode", speedScale == 1.0 ? "FAST" : "SLOW");
    }

    /** LAUNCHER FAIR SYNC RPM + VOLTAGE COMPENSATION **/
    private void updateLauncherVelocitySync() {
        if (gamepad1.right_trigger > 0.5) targetRPM = highRPM;
        else if (gamepad1.left_trigger > 0.5) targetRPM = lowRPM;
        else targetRPM = 0;

        double batteryVoltage = battery.getVoltage();
        double compensation = 12.0 / batteryVoltage;

        double velocity = targetRPM > 50 ? (targetRPM / 60.0) * TICKS_PER_REV * compensation : 0;
        launcherLeft.setVelocity(velocity);
        launcherRight.setVelocity(velocity);
    }

    private void controlIntake() {
        if (gamepad1.right_bumper && !lastIntakeToggle)
            intakeToggle = !intakeToggle;
        lastIntakeToggle = gamepad1.right_bumper;

        double power = 0;
        if (gamepad1.a) power = 1.0;
        else if (gamepad1.b) power = -1.0;
        else if (intakeToggle) power = 1.0;

        intakeMotor.setPower(power);
        telemetry.addData("Intake", power);
    }

    private void controlFlaps() {
        if (gamepad1.left_bumper && !lastFlapToggle) {
            flapUp = !flapUp;
            if (flapUp) flapTimer = System.currentTimeMillis();
        }
        lastFlapToggle = gamepad1.left_bumper;

        if (flapUp && System.currentTimeMillis() - flapTimer > 500)
            flapUp = false;

        liftLeft.setPosition(flapUp ? FLAP_UP_POS : FLAP_DOWN_POS);
        liftRight.setPosition(flapUp ? 1 - FLAP_UP_POS : 1 - FLAP_DOWN_POS);
    }

    private void updateTelemetry() {
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Left RPM", launcherLeft.getVelocity() / TICKS_PER_REV * 60.0);
        telemetry.addData("Right RPM", launcherRight.getVelocity() / TICKS_PER_REV * 60.0);
        telemetry.update();
    }
}
