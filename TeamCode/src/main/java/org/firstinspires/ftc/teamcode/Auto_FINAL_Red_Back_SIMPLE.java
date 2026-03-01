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
@Autonomous(name = "Auto_FINAL_Red_Back_SIMPLE", group = "Auton")
public class Auto_FINAL_Red_Back_SIMPLE extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeMotor;
    private DcMotorEx launcherLeft, launcherRight;
    private Servo liftLeft, liftRight;

    private double FLAP_DOWN_POS = 0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_DRIVE_RPM = 2600;
    private static final double DRIVE_TICKS_PER_SEC = (MAX_DRIVE_RPM / 60.0) * TICKS_PER_REV;
    private double FLAP_UP_POS = 0.5;

    @Override
    public void runOpMode() throws InterruptedException{
        initHardware();

        liftLeft.setPosition(0);
        liftRight.setPosition(1);
        waitForStart();

        while (opModeIsActive()){
            rotateRight(0.78);
            sleep(700);
            rotateRight(0);
            setDrivePower(1);
            sleep(600);
            setDrivePower(0);
            stop();

        }
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

        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
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
