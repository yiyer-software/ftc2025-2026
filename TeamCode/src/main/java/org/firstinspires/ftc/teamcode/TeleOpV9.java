package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.FlywheelConstants;
@TeleOp(name = "TeleOpV9", group = "Competition")
public class TeleOpV9 extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeMotor;
    private DcMotorEx launcherLeft, launcherRight;
    private Servo liftLeft, liftRight;
    private double HIGH_RPM = FlywheelConstants.HIGH_RPM;
    private double DEFAULT_RPM = FlywheelConstants.DEFAULT_RPM;
    private double LOW_RPM = FlywheelConstants.LOW_RPM;
    private double FLAP_UP_POS = 0.5;
    private double FLAP_DOWN_POS = -0.25;
    private double DRIVE_DEADZONE = 0.03;
    private double DRIVE_MAX_RPM = 7000;
    private double BACKUP_SPEED = 1.0;

    private double targetRPM = 0;
    private boolean launcherOn = false;
    private boolean lastY1 = false;
    private boolean lastY2 = false;
    private boolean intakeToggle = false;
    private boolean lastIntakeToggle = false;
    private boolean flapUp = false;
    private boolean lastFlapToggle = false;
    private long flapTimer = 0;

    private boolean isBackingUp = false;
    private long backupEndTimeMs = 0;
    private boolean lastX1 = false;
    private boolean lastX2 = false;

    private static final double TICKS_PER_REV = 28.0;
    private double DRIVE_TICKS_PER_SEC;
    private double TICKS_TO_RPM = 60.0 / TICKS_PER_REV;

    @Override
    public void runOpMode() {
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

        DRIVE_TICKS_PER_SEC = (DRIVE_MAX_RPM / 60.0) * TICKS_PER_REV;

        waitForStart();

        while (opModeIsActive()) {
            handleBackup();
            if (!isBackingUp) drive();
            controlLauncherToggle();
            updateLauncher();
            controlIntake();
            controlFlaps();
            updateTelemetry();
            lastX1 = gamepad1.x;
            lastX2 = gamepad2.x;
            lastY1 = gamepad1.y;
            lastY2 = gamepad2.y;
        }
    }

    private double combinedAxis(double a, double b) {
        return Math.abs(b) > Math.abs(a) ? b : a;
    }

    private boolean combinedButton(boolean a, boolean b) {
        return a || b;
    }

    private void drive() {
        double y = -combinedAxis(gamepad1.left_stick_y, gamepad2.left_stick_y);
        double x = combinedAxis(gamepad1.left_stick_x, gamepad2.left_stick_x) * 1.15;
        double rx = -combinedAxis(gamepad1.right_stick_x, gamepad2.right_stick_x);

        if (Math.abs(y) < DRIVE_DEADZONE) y = 0;
        if (Math.abs(x) < DRIVE_DEADZONE) x = 0;
        if (Math.abs(rx) < DRIVE_DEADZONE) rx = 0;

        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeft.setVelocity(fl * DRIVE_TICKS_PER_SEC);
        frontRight.setVelocity(fr * DRIVE_TICKS_PER_SEC);
        backLeft.setVelocity(bl * DRIVE_TICKS_PER_SEC);
        backRight.setVelocity(br * DRIVE_TICKS_PER_SEC);
    }

    private void handleBackup() {
        boolean x1 = gamepad1.x;
        boolean x2 = gamepad2.x;
        boolean pressedEdge = (x1 && !lastX1) || (x2 && !lastX2);

        if (pressedEdge && !isBackingUp) {
            isBackingUp = true;
            backupEndTimeMs = System.currentTimeMillis() + 150;
        }

        if (isBackingUp) {
            if (System.currentTimeMillis() < backupEndTimeMs) {
                double vel = -DRIVE_TICKS_PER_SEC * BACKUP_SPEED;
                frontLeft.setVelocity(vel);
                frontRight.setVelocity(vel);
                backLeft.setVelocity(vel);
                backRight.setVelocity(vel);
            } else {
                isBackingUp = false;
                frontLeft.setVelocity(0);
                frontRight.setVelocity(0);
                backLeft.setVelocity(0);
                backRight.setVelocity(0);
            }
        }
    }

    private void controlLauncherToggle() {
        boolean y1 = gamepad1.y;
        boolean y2 = gamepad2.y;
        boolean pressedEdge = (y1 && !lastY1) || (y2 && !lastY2);

        if (pressedEdge) launcherOn = !launcherOn;
    }

    private void updateLauncher() {
        if (launcherOn) targetRPM = DEFAULT_RPM;
        else targetRPM = 0;

        double rt = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
        double lt = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);

        if (rt > 0.5) targetRPM = HIGH_RPM;
        else if (lt > 0.5) targetRPM = LOW_RPM;

        double vel = (targetRPM / 60.0) * TICKS_PER_REV;
        launcherLeft.setVelocity(vel);
        launcherRight.setVelocity(vel);
    }

    private void controlIntake() {
        boolean bumper = combinedButton(gamepad1.right_bumper, gamepad2.right_bumper);

        if (bumper && !lastIntakeToggle) intakeToggle = !intakeToggle;
        lastIntakeToggle = bumper;

        double pA = combinedButton(gamepad1.a, gamepad2.a) ? 1.0 : 0;
        double pB = combinedButton(gamepad1.b, gamepad2.b) ? -1.0 : 0;

        double power = pA != 0 ? pA : (pB != 0 ? pB : (intakeToggle ? 1.0 : 0));
        intakeMotor.setPower(power);
    }

    private void controlFlaps() {
        boolean flapButton = combinedButton(gamepad1.left_bumper, gamepad2.left_bumper);

        if (flapButton && !lastFlapToggle) {
            flapUp = !flapUp;
            if (flapUp) flapTimer = System.currentTimeMillis();
        }
        lastFlapToggle = flapButton;

        if (flapUp && System.currentTimeMillis() - flapTimer > 300) flapUp = false;

        liftLeft.setPosition(flapUp ? FLAP_UP_POS : FLAP_DOWN_POS);
        liftRight.setPosition(flapUp ? 1 - FLAP_UP_POS : 1 - FLAP_DOWN_POS);
    }

    private void updateTelemetry() {
        telemetry.clearAll();
        telemetry.addLine("FLYWHEEL");
        telemetry.addData("Launcher", launcherOn ? "ON" : "OFF");
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Left RPM", "%.0f", launcherLeft.getVelocity() * TICKS_TO_RPM);
        telemetry.addData("Right RPM", "%.0f", launcherRight.getVelocity() * TICKS_TO_RPM);
        telemetry.addLine("FLAPS");
        telemetry.addData("Status", flapUp ? "UP" : "DOWN");
        telemetry.addLine("INTAKE");
        telemetry.addData("Power", "%.2f", intakeMotor.getPower());
        telemetry.addLine("DRIVE");
        telemetry.addData("FL Vel", "%.0f", frontLeft.getVelocity());
        telemetry.addData("FR Vel", "%.0f", frontRight.getVelocity());
        telemetry.addData("BL Vel", "%.0f", backLeft.getVelocity());
        telemetry.addData("BR Vel", "%.0f", backRight.getVelocity());
        telemetry.update();
    }
}