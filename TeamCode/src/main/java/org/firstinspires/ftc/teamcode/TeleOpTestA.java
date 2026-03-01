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

@TeleOp(name = "TeleOpTestA", group = "Test")
public class TeleOpTestA extends LinearOpMode {

    // ---------------- DASHBOARD TUNABLES ----------------
    public static double LAUNCHER_RPM = 7500;

    public static double SERVO_UP_POS = 1.0;
    public static double SERVO_DOWN_POS = 0.0;

    public static int SPINNER_STEP_TICKS = 200;
    public static double SPINNER_POWER = 0.6;

    // ---------------- CONSTANTS ----------------
    private static final double TICKS_PER_REV = 28.0;

    // ---------------- HARDWARE ----------------
    private DcMotorEx launcherLeft, launcherRight;
    private DcMotorEx spinnerMotor;
    private Servo servoLeft, servoRight;

    // ---------------- STATE ----------------
    private int spinnerTarget = 0;
    private boolean lastX = false;
    private boolean lastB = false;

    @Override
    public void runOpMode() {

        // ---------------- HARDWARE MAP ----------------
        launcherLeft  = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");
        spinnerMotor  = hardwareMap.get(DcMotorEx.class, "spinnerMotor");

        servoLeft  = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------------- SPINNER SETUP ----------------
        spinnerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinnerMotor.setTargetPosition(0);
        spinnerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // ---------------- DASHBOARD ----------------
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Launcher / Servo / Spinner STEP Test Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---------------- LAUNCHER (RPM CONTROL) ----------------
            boolean launcherPressed =
                    gamepad1.left_stick_button || gamepad1.right_stick_button;

            double ticksPerSecond =
                    (LAUNCHER_RPM / 60.0) * TICKS_PER_REV;

            if (launcherPressed) {
                launcherLeft.setVelocity(ticksPerSecond);
                launcherRight.setVelocity(ticksPerSecond);
            } else {
                launcherLeft.setVelocity(0);
                launcherRight.setVelocity(0);
            }

            // ---------------- SERVOS (RT) ----------------
            if (gamepad1.right_trigger > 0.5) {
                servoLeft.setPosition(SERVO_UP_POS);
                servoRight.setPosition(SERVO_UP_POS);
            } else {
                servoLeft.setPosition(SERVO_DOWN_POS);
                servoRight.setPosition(SERVO_DOWN_POS);
            }

            // ---------------- SPINNER STEP CONTROL ----------------
            boolean x = gamepad1.x;
            boolean b = gamepad1.b;

            if (x && !lastX) {
                spinnerTarget += SPINNER_STEP_TICKS;
            }
            if (b && !lastB) {
                spinnerTarget -= SPINNER_STEP_TICKS;
            }

            lastX = x;
            lastB = b;

            spinnerMotor.setTargetPosition(spinnerTarget);
            spinnerMotor.setPower(SPINNER_POWER);

            // ---------------- TELEMETRY ----------------
            telemetry.addLine("SPINNER");
            telemetry.addData("Target Ticks", spinnerTarget);
            telemetry.addData("Current Ticks", spinnerMotor.getCurrentPosition());
            telemetry.addData("Step Size", SPINNER_STEP_TICKS);

            telemetry.addLine("LAUNCHER");
            telemetry.addData("RPM Target", LAUNCHER_RPM);

            telemetry.update();
        }
    }
}