package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;

@Autonomous(name = "Auto_OM", group = "Auton")
public class Auto_OM extends LinearOpMode {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeMotor;
    private DcMotorEx launcherLeft, launcherRight;
    private Servo liftLeft, liftRight;
    private static final double HIGH_RPM = -500;
    private static final double LOW_RPM = -300;
    private double targetRPM = 0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_DRIVE_RPM = 2100;
    private static final double DRIVE_TICKS_PER_SEC = (MAX_DRIVE_RPM / 60.0) * TICKS_PER_REV;
    private boolean intakeToggle = false;
    private boolean lastIntakeToggle = false;
    // ticks per wheel revolution after gear reduction
    private static final double TPR = TICKS_PER_REV * 1.0;

    // distance traveled per wheel revolution, 4.0 is the wheel diameter in inches
    private static final double INCHES_PER_REV = Math.PI * 4.0;
    private boolean flapUp = false;
    private boolean lastFlapToggle = false;
    private long flapTimer = 0;
    private final double FLAP_UP_POS = 0.5;
    private final double FLAP_DOWN_POS = 0.0;


    @Override
    public void runOpMode() throws InterruptedException {
        

    }

    private void initHardware(){
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
    }
}
