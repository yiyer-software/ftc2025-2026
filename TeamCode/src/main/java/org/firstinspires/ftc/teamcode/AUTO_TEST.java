package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
@Autonomous(name = "Auto_TEST", group = "Auton")
public class AUTO_TEST extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeMotor;
    private DcMotorEx launcherLeft, launcherRight;
    private Servo liftLeft, liftRight;

    private double FLAP_DOWN_POS = 0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_DRIVE_RPM = 2600;
    private static final double DRIVE_TICKS_PER_SEC = (MAX_DRIVE_RPM / 60.0) * TICKS_PER_REV;
    private double FLAP_UP_POS = 0.5;

    private static final double WHEEL_DIAMETER_IN = 4;      // FTC mecanum
    private static final double TRACK_WIDTH_IN = 15.5;         // Measure this

    private static final double TICKS_PER_INCH =
            (TICKS_PER_REV * 1.0) / (Math.PI * WHEEL_DIAMETER_IN);

    private static final double TURN_GAIN = 1.18; // tune 1.10–1.25


    @Override
    public void runOpMode() throws InterruptedException{
        initHardware();

        liftLeft.setPosition(0);
        liftRight.setPosition(1);
        waitForStart();

        while (opModeIsActive()){
            setDrivePower(-1);
            sleep(300);
            setDrivePower(0);
            launcherLeft.setVelocity(-1*(DRIVE_TICKS_PER_SEC));
            launcherRight.setVelocity(-1*(DRIVE_TICKS_PER_SEC));
            sleep(1350);
            liftLeft.setPosition(0.5);
            liftRight.setPosition(0.5);
            sleep(500);
            liftLeft.setPosition(0);
            liftRight.setPosition(1);
            intakeMotor.setPower(0.78);
            sleep(850);
            intakeMotor.setPower(0);
            liftLeft.setPosition(0.5);
            liftRight.setPosition(0.5);
            sleep(500);
            liftLeft.setPosition(0);
            liftRight.setPosition(1);
            intakeMotor.setPower(0.45);
            sleep(1850);
            intakeMotor.setPower(0);
            liftLeft.setPosition(0.5);
            liftRight.setPosition(0.5);
            sleep(500);
            liftLeft.setPosition(0);
            liftRight.setPosition(1);
            setDrivePower(-1);
            sleep(750);
            turnDegrees(-60, 0.78);
            intakeMotor.setPower(1);
            setDrivePower(1);
            sleep(900);
            setDrivePower(0);
            intakeMotor.setPower(0);
            setDrivePower(-1);
            sleep(900);
            setDrivePower(0);
            turnDegrees(60, 0.78);
            setDrivePower(1);
            sleep(750);
            setDrivePower(0);
            liftLeft.setPosition(0.5);
            liftRight.setPosition(0.5);
            sleep(500);
            liftLeft.setPosition(0);
            liftRight.setPosition(1);
            intakeMotor.setPower(0.78);
            sleep(850);
            intakeMotor.setPower(0);
            liftLeft.setPosition(0.5);
            liftRight.setPosition(0.5);
            sleep(500);
            liftLeft.setPosition(0);
            liftRight.setPosition(1);
            intakeMotor.setPower(0.45);
            sleep(1850);
            intakeMotor.setPower(0);
            liftLeft.setPosition(0.5);
            liftRight.setPosition(0.5);
            sleep(500);
            liftLeft.setPosition(0);
            liftRight.setPosition(1);
            break;




        }

        stop();
    }

    public void turnDegrees(double degrees, double power) {

        power = Math.abs(power); // RUN_TO_POSITION requires positive power

        // Calculate arc length per wheel
        double turnCircumference = Math.PI * TRACK_WIDTH_IN;
        double distance = (degrees / 360.0) * turnCircumference * TURN_GAIN;
        int ticks = (int) Math.round(distance * TICKS_PER_INCH);

        // Get current positions (NO reset)
        int fl = frontLeft.getCurrentPosition();
        int bl = backLeft.getCurrentPosition();
        int fr = frontRight.getCurrentPosition();
        int br = backRight.getCurrentPosition();

        // Set relative targets
        frontLeft.setTargetPosition(fl + ticks);
        backLeft.setTargetPosition(bl + ticks);
        frontRight.setTargetPosition(fr - ticks);
        backRight.setTargetPosition(br - ticks);

        // Brake for accuracy
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        // Wait
        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy()
                        || backLeft.isBusy() || backRight.isBusy())) {
            // optional telemetry
        }

        // Stop
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Back to normal
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    private void rotateRight(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }
    private void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void initHardware() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");

        liftLeft = hardwareMap.get(Servo.class, "liftServoLeft");
        liftRight = hardwareMap.get(Servo.class, "liftServoRight");

        // ==== Flip directions because front is now back ====
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
    }
}
