package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@Config
@TeleOp(name = "TeleOp2026V1", group = "Competition")
public class TeleOp2026V1 extends LinearOpMode {

    // ================= HARDWARE =================
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx launcherLeft, launcherRight;
    private DcMotor intakeMotor, sortMotor;
    private Servo hoodLeft, hoodRight, liftLeft, liftRight;
    private ColorSensor colorSensor;

    // ================= DASHBOARD SORTER VARIABLES =================
    public static double SORT_MOTOR_TICKS_PER_REV = 560.0; // REV 20:1
    public static double SORT_GEAR_NUM = 16.0;
    public static double SORT_GEAR_DEN = 13.0;
    public static double SORT_DEGREES = 120.0;
    public static double SORT_POWER = 0.5;

    private double ticksPerOutputRev;
    private double ticksPerDegree;
    private int SORT_MOVE_TICKS;

    // ================= DRIVE CONSTANTS =================
    private double TICKS_PER_REV = 28.0;
    private double MAX_RPM = 7000;
    private double DRIVE_TICKS_PER_SEC;
    private double DRIVE_DEADZONE = 0.03;

    // ================= STATE =================
    private boolean shooterOn=false, intakeOn=false, sortOn=false;
    private boolean lastY=false, lastB=false, lastX=false, lastRB=false;

    private FtcDashboard dashboard;
    private MultipleTelemetry telemetryDashboard;

    private double lastTime = 0;
    private double hoodPosX = 0.5;



    // ============================================================
    @Override
    public void runOpMode() {

        // HARDWARE MAP
        frontLeft = hardwareMap.get(DcMotorEx.class,"leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class,"rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class,"leftBack");
        backRight = hardwareMap.get(DcMotorEx.class,"rightBack");

        launcherLeft = hardwareMap.get(DcMotorEx.class,"leftLauncher");
        launcherRight = hardwareMap.get(DcMotorEx.class,"rightLauncher");

        intakeMotor = hardwareMap.get(DcMotor.class,"intakeMotor");
        sortMotor = hardwareMap.get(DcMotor.class,"sortMotor");

        hoodLeft = hardwareMap.get(Servo.class,"leftHood");
        hoodRight = hardwareMap.get(Servo.class,"rightHood");
        liftLeft = hardwareMap.get(Servo.class,"liftLeft");
        liftRight = hardwareMap.get(Servo.class,"liftRight");

        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");

        // MOTOR SETUP
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

        sortMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sortMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DRIVE_TICKS_PER_SEC = (MAX_RPM/60.0)*TICKS_PER_REV;

        dashboard = FtcDashboard.getInstance();
        telemetryDashboard = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // COMPUTE SORTER MATH
        ticksPerOutputRev = SORT_MOTOR_TICKS_PER_REV * (SORT_GEAR_NUM / SORT_GEAR_DEN);
        ticksPerDegree = ticksPerOutputRev / 360.0;
        SORT_MOVE_TICKS = (int)Math.round(SORT_DEGREES * ticksPerDegree);

        waitForStart();

        while(opModeIsActive()){
            handleDrive();
            handleShooter();
            shootingSequence();
            handleLift();
            handleHoodManual();
            updateTelemetry();
        }
    }

    // ================= DRIVE =================
    private void handleDrive(){
        double y=-gamepad1.left_stick_y;
        double x=gamepad1.left_stick_x*1.15;
        double rx=-gamepad1.right_stick_x;

        if(Math.abs(y)<DRIVE_DEADZONE) y=0;
        if(Math.abs(x)<DRIVE_DEADZONE) x=0;
        if(Math.abs(rx)<DRIVE_DEADZONE) rx=0;

        double fl=y+x+rx;
        double fr=y-x-rx;
        double bl=y-x+rx;
        double br=y+x-rx;

        double max=Math.max(Math.abs(fl),Math.max(Math.abs(fr),Math.max(Math.abs(bl),Math.abs(br))));
        if(max>1){ fl/=max; fr/=max; bl/=max; br/=max; }

        frontLeft.setVelocity(fl*DRIVE_TICKS_PER_SEC);
        frontRight.setVelocity(fr*DRIVE_TICKS_PER_SEC);
        backLeft.setVelocity(bl*DRIVE_TICKS_PER_SEC);
        backRight.setVelocity(br*DRIVE_TICKS_PER_SEC);
    }

    // ================= SHOOTER =================
    private void handleShooter(){
        boolean y=gamepad1.y;
        if(y&&!lastY) shooterOn=!shooterOn;
        lastY=y;

        double targetRPM = shooterOn ? 5000 : 0;
        if(gamepad1.right_trigger>0.5) targetRPM=6500;
        if(gamepad1.left_trigger>0.5) targetRPM=3500;

        double vel=targetRPM/60.0*TICKS_PER_REV;
        launcherLeft.setVelocity(vel);
        launcherRight.setVelocity(vel);
    }
    // ================= LIFT =================
    private boolean liftActive=false;
    private double liftStartTime=0;

    private void handleLift(){
        if(gamepad1.a && !liftActive){
            liftActive=true;
            liftStartTime=getRuntime();
            liftLeft.setPosition(1);
            liftRight.setPosition(0);
        }

        if(liftActive && getRuntime()-liftStartTime>=0.5){
            liftLeft.setPosition(0);
            liftRight.setPosition(1);
            liftActive=false;
        }
    }

    // ================= HOOD =================
    private void handleHoodManual(){
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        double speed = 0.05;

        if(gamepad1.dpad_up) hoodPosX += speed * dt;
        if(gamepad1.dpad_down) hoodPosX -= speed * dt;

        hoodPosX = Math.max(0.35, Math.min(0.7, hoodPosX));

        hoodLeft.setPosition(hoodPosX);
        hoodRight.setPosition(1.0 - hoodPosX);
    }

    private boolean shootingActive = false;
    private boolean flywheelsStarted = false;
    static final double COUNTS_PER_REV = 1378.0;
    static final double COUNTS_PER_DEGREE = COUNTS_PER_REV / 360.0;
    private int shootState = 0;
    private double shootTimer = 0;
    private void shootingSequence(){
        boolean rb = gamepad1.right_bumper;
        if (rb && !lastRB && !shootingActive) {
            shootingActive = true;
            shootState = 0;
        }
        lastRB = rb;
        double vel=6000.0/60.0*TICKS_PER_REV;
        launcherLeft.setVelocity(vel);
        launcherRight.setVelocity(vel);
        flywheelsStarted = true;


        if (!shootingActive) return;

        switch (shootState) {

            case 0:

                if (!flywheelsStarted) {
                    flywheelsStarted = true;
                }

                shootState = 1;
                break;


            case 1:

                if (!sortMotor.isBusy()) {

                    int currentPos = sortMotor.getCurrentPosition();
                    SORT_MOVE_TICKS = (int)(120 * COUNTS_PER_DEGREE);
                    int target = currentPos + SORT_MOVE_TICKS;

                    sortMotor.setTargetPosition(target);
                    sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sortMotor.setPower(SORT_POWER);

                    shootState = 2;
                }
                break;

            case 2:

                if (!sortMotor.isBusy()) {
                    shootTimer = getRuntime();
                    shootState = 3;
                }
                break;

            case 3:

                if (getRuntime() - shootTimer >= 0.2) {
                    liftLeft.setPosition(1);
                    liftRight.setPosition(0);
                    shootTimer = getRuntime();
                    shootState = 4;
                }
                break;

            case 4:

                if (getRuntime() - shootTimer >= 0.1) {
                    liftLeft.setPosition(0);
                    liftRight.setPosition(1);
                    shootTimer = getRuntime();
                    shootState = 5;
                }
                break;

            case 5:

                if (getRuntime() - shootTimer >= 0.1) {
                    shootingActive = false;
                    launcherLeft.setVelocity(0);
                    launcherRight.setVelocity(0);
                }
                break;
        }

    }

    private boolean lastLB = false;
    private boolean intakeSequenceActive = false;
    private int intakeState = 0;
    private double intakeTimer = 0;
    private int[] indexOrder = new int[3];
    
    private void intakeSequence(){
        boolean lb = gamepad1.left_bumper;

        if (lb && !lastLB && !intakeSequenceActive) {
            intakeSequenceActive = true;
            intakeState = 0;
        }
        lastLB = lb;

        if (!intakeSequenceActive) return;

        switch (intakeState) {

            case 0:

                intakeMotor.setPower(0.8);

                boolean greenDetected = (colorSensor.green() > 150);
                boolean purpleDetected = (colorSensor.red() > 150 && colorSensor.blue() > 150);

                if (greenDetected || purpleDetected) {
                    intakeTimer = getRuntime();
                    intakeMotor.setPower(0);
                    intakeState = 1;
                }

                break;

            case 1:

                if (getRuntime() - intakeTimer >= 0.1) {
                    intakeState = 2;
                }
                intakeMotor.setPower(0.8);

                break;

            case 2:
                intakeMotor.setPower(0);

                if (!sortMotor.isBusy()) {

                    int currentPos = sortMotor.getCurrentPosition();
                    int moveTicks = (int)(60 * COUNTS_PER_DEGREE);
                    int target = currentPos + moveTicks;

                    sortMotor.setTargetPosition(target);
                    sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sortMotor.setPower(SORT_POWER);

                    intakeState = 3;
                }

                break;

            case 3:

                if (!sortMotor.isBusy()) {
                    intakeSequenceActive = false; // stop until next press
                }

                break;
        }


    }

    // ================= TELEMETRY =================
    private void updateTelemetry(){
        telemetryDashboard.addData("Sorter ticks/deg",ticksPerDegree);
        telemetryDashboard.addData("120deg ticks",SORT_MOVE_TICKS);
        telemetryDashboard.addData("Sorter position",sortMotor.getCurrentPosition());
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
        telemetryDashboard.update();
    }
}
