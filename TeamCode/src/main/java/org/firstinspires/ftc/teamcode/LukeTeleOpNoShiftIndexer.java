package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

@Config
@TeleOp(name = "TeleOp2026FullShootingSequence", group = "Competition")
public class TeleOp2026FullShootingSequence extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx launcherLeft, launcherRight;
    private DcMotor intakeMotor, sortMotor;
    private Servo hoodLeft, hoodRight, liftLeft, liftRight;
    private ColorSensor colorSensor;

    public static double SORT_MOTOR_TICKS_PER_REV = 560.0;
    public static double SORT_GEAR_NUM = 16.0;
    public static double SORT_GEAR_DEN = 13.0;
    public static double SORT_DEGREES = 173.0;
    public static double SORT_POWER = 0.5;

    private double ticksPerOutputRev;
    private double ticksPerDegree;
    private int SORT_MOVE_TICKS;

    private final double TICKS_PER_REV = 28.0;
    private final double MAX_RPM = 7000.0;
    private double DRIVE_TICKS_PER_SEC;
    private final double DRIVE_DEADZONE = 0.03;

    private FtcDashboard dashboard;
    private MultipleTelemetry telemetryDashboard;

    private double lastTime = 0.0;
    private double hoodPosX = 0.8;

    private boolean liftActive = false;
    private double liftStartTime = 0.0;

    private int[] indexOrder = new int[]{2,2,2};
    private int[] shootOrder = new int[]{3,3,3};
    private int[] motifOrder = new int[]{0,0,0};

    private int indexerPos = 0;
    private boolean notShifting = true;

    private boolean shootingActive = false;
    private boolean lastLB = false;

    private int shootState = 0;
    private int ballsLaunched = 0;
    private int ballsIn = 0;

    private double sorterTimer = 0.0;
    private double liftTimer = 0.0;

    static final double COUNTS_PER_REV = 1378.0;
    static final double COUNTS_PER_DEGREE = COUNTS_PER_REV / 360.0;

    private boolean lastRB = false;
    private boolean intakeSequenceActive = false;
    private int intakeState = 0;
    private double intakeTimer = 0.0;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");

        launcherLeft = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        launcherRight = hardwareMap.get(DcMotorEx.class, "rightLauncher");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        sortMotor = hardwareMap.get(DcMotor.class, "sortMotor");

        hoodLeft = hardwareMap.get(Servo.class, "leftHood");
        hoodRight = hardwareMap.get(Servo.class, "rightHood");
        liftLeft = hardwareMap.get(Servo.class, "liftLeft");
        liftRight = hardwareMap.get(Servo.class, "liftRight");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

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

        sortMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DRIVE_TICKS_PER_SEC = (MAX_RPM / 60.0) * TICKS_PER_REV;

        dashboard = FtcDashboard.getInstance();
        telemetryDashboard = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        ticksPerOutputRev = SORT_MOTOR_TICKS_PER_REV * (SORT_GEAR_NUM / SORT_GEAR_DEN);
        ticksPerDegree = ticksPerOutputRev / 360.0;

        waitForStart();
        lastTime = getRuntime();

        while (opModeIsActive()) {
            handleDrive();
            shootingSequence();
            intakeSequence();
            handleLift();
            manualShift();
            handleHoodManual();
            updateTelemetry();
        }
    }

    private void handleDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.15;
        double rx = gamepad1.right_stick_x;

        if (Math.abs(y) < DRIVE_DEADZONE) y = 0;
        if (Math.abs(x) < DRIVE_DEADZONE) x = 0;
        if (Math.abs(rx) < DRIVE_DEADZONE) rx = 0;

        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                Math.max(Math.abs(bl), Math.abs(br))));

        if (max > 1) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        frontLeft.setVelocity(fl * DRIVE_TICKS_PER_SEC);
        frontRight.setVelocity(fr * DRIVE_TICKS_PER_SEC);
        backLeft.setVelocity(bl * DRIVE_TICKS_PER_SEC);
        backRight.setVelocity(br * DRIVE_TICKS_PER_SEC);
    }

    private void handleHoodManual() {
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        double speed = 0.05;

        if (gamepad1.dpad_up) hoodPosX -= speed * dt;
        if (gamepad1.dpad_down) hoodPosX += speed * dt;

        hoodPosX = Math.max(0.01, Math.min(0.7, hoodPosX)); // FIXED

        hoodLeft.setPosition(hoodPosX);
        hoodRight.setPosition(1.0 - hoodPosX);
    }

    private void manualShift(){
        if(gamepad1.a && notShifting){
            notShifting = false;

            int currentPos = sortMotor.getCurrentPosition();
            SORT_MOVE_TICKS = (int)(5.0 * COUNTS_PER_DEGREE);
            int target = currentPos + SORT_MOVE_TICKS;

            sortMotor.setTargetPosition(target);
            sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sortMotor.setPower(SORT_POWER);

            sorterTimer = getRuntime();
        }

        if(getRuntime() - sorterTimer >= 0.1){
            notShifting = true;
        }
    }

    private void shootingSequence() {

        boolean lb = gamepad1.left_bumper;

        if(lb && !lastLB && !shootingActive){
            shootingActive = true;
            shootState = 0;
        }

        lastLB = lb;
        if(!shootingActive) return;

        double vel = 3000.0 / 60.0 * TICKS_PER_REV;
        launcherLeft.setVelocity(vel);
        launcherRight.setVelocity(vel);

        switch(shootState){

            case 0:
                shootState = 1;
                break;

            case 1:
                if(!sortMotor.isBusy()){

                    int currentPos = sortMotor.getCurrentPosition();
                    SORT_MOVE_TICKS = (int)((SORT_DEGREES/2.0) * COUNTS_PER_DEGREE);
                    int target = currentPos + SORT_MOVE_TICKS;

                    sortMotor.setTargetPosition(target);
                    sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sortMotor.setPower(SORT_POWER);

                    indexerPos = (indexerPos + 1) % 3;
                    sorterTimer = getRuntime();
                    shootState = 2;
                }
                break;

            case 2:
                if(!sortMotor.isBusy() && getRuntime() - sorterTimer >= 0.35){
                    liftLeft.setPosition(0.35);
                    liftRight.setPosition(0.65);
                    liftTimer = getRuntime();
                    shootState = 3;
                }
                break;

            case 3:
                if(getRuntime() - liftTimer >= 0.75){
                    liftLeft.setPosition(0.99);
                    liftRight.setPosition(0.01);
                    ballsLaunched++;
                    ballsIn--;
                    shootState = (ballsIn == 0) ? 4 : 1;
                }
                break;

            case 4:
                shootingActive = false;
                ballsLaunched = 0;
                indexOrder = new int[]{2,2,2};
                launcherLeft.setVelocity(0);
                launcherRight.setVelocity(0);
                break;
        }
    }

    private void intakeSequence(){

        boolean rb = gamepad1.right_bumper;

        if(rb && !lastRB && !intakeSequenceActive){
            intakeSequenceActive = true;
            intakeState = 0;
            ballsIn = 0;
        }

        lastRB = rb;
        if(!intakeSequenceActive) return;

        switch(intakeState){

            case 0:
                intakeMotor.setPower(-0.8);

                boolean green = colorSensor.green() > 100;
                boolean purple = (colorSensor.red() > 55 && colorSensor.blue() > 80);

                if(green || purple){
                    intakeTimer = getRuntime();

                    if(ballsIn < 3){
                        indexOrder[ballsIn] = green ? 1 : 0;
                    }

                    intakeState = 1;
                }
                break;

            case 1:
                if(getRuntime() - intakeTimer >= 0.75 && !sortMotor.isBusy()){

                    int currentPos = sortMotor.getCurrentPosition();
                    SORT_MOVE_TICKS = (int)((SORT_DEGREES/2.0) * COUNTS_PER_DEGREE);
                    int target = currentPos + SORT_MOVE_TICKS;

                    sortMotor.setTargetPosition(target);
                    sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sortMotor.setPower(SORT_POWER);

                    indexerPos = (indexerPos + 1) % 3;
                    ballsIn++;
                    intakeState = 2;
                }
                break;

            case 2:
                if(!sortMotor.isBusy() && ballsIn < 3){
                    intakeState = 0;
                }

                if(ballsIn >= 3){
                    intakeSequenceActive = false;
                    intakeMotor.setPower(0);
                }
                break;
        }
    }

    private void handleLift(){
        if(gamepad1.x && !liftActive){
            liftActive = true;
            liftStartTime = getRuntime();
            liftLeft.setPosition(0.35);
            liftRight.setPosition(0.65);
        }

        if(liftActive && getRuntime() - liftStartTime >= 1){
            liftLeft.setPosition(0.99);
            liftRight.setPosition(0.01);
            liftActive = false;
        }
    }

    private void updateTelemetry(){

        telemetryDashboard.addData("Sorter Position", sortMotor.getCurrentPosition());
        telemetryDashboard.addData("Balls In", ballsIn);
        telemetryDashboard.addData("Balls Launched", ballsLaunched);

        telemetryDashboard.addLine("===== BALL DATA =====");
        telemetryDashboard.addData("Index Order", Arrays.toString(indexOrder));
        telemetryDashboard.addData("Shoot Order", Arrays.toString(shootOrder));
        telemetryDashboard.addData("Motif Order", Arrays.toString(motifOrder));
        telemetryDashboard.addData("Indexer Pos", indexerPos);

        telemetryDashboard.update();
    }
}
