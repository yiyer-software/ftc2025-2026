package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auto_Final_Wall", group = "Auton")
public class Auto_Final_Wall extends LinearOpMode{
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeMotor;
    private DcMotorEx launcherLeft, launcherRight;
    private Servo liftLeft, liftRight;
    private static final double HIGH_RPM = -500;
    private static final double LOW_RPM = -300;
    private double targetRPM = 0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_DRIVE_RPM = 1000;
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
    public void runOpMode() throws InterruptedException{
        initHardware();
        liftLeft.setPosition(1-FLAP_DOWN_POS);
        liftRight.setPosition(FLAP_DOWN_POS);
        waitForStart();

        while (opModeIsActive()){
            strafeLeft(18, 0.8);
            driveForward(72, 0.8);
            rotateDegrees(45, 0.3, "CW");
            driveForward(18, 0.8);
            launcherLeft.setVelocity(DRIVE_TICKS_PER_SEC);
            launcherRight.setVelocity(DRIVE_TICKS_PER_SEC);

            telemetry.addData("Flywheel", "Spinning to %.0f RPM", MAX_DRIVE_RPM);
            telemetry.update();

            //No for-loop on purpose, for better tuning for the timings!!
            flapsControlUp();
            sleep(500);
            flapsControlDown();
            intakeMotor.setPower(0.45);
            sleep(1000);
            intakeMotor.setPower(0);
            flapsControlUp();
            flapsControlDown();
            intakeMotor.setPower(0.45);
            sleep(1000);
            intakeMotor.setPower(0);
            flapsControlUp();
            flapsControlDown();
        }

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

    private void flapsControlUp(){
        liftLeft.setPosition(0.5);
        liftRight.setPosition(0.5);
    }

    private void flapsControlDown(){
        liftLeft.setPosition(0);
        liftRight.setPosition(0);
    }

    private int inchesToTicks(double inches) {
        return (int) ((inches / INCHES_PER_REV) * TPR);
    }

    public void rotateDegrees(double degrees, double speed, String direction) {

        // ==== ROBOT PHYSICAL CONSTANTS (ADJUST IF NEEDED) ====
        double TRACK_WIDTH = 14.0;   // distance between left and right wheels (inches)
        double TRACK_LENGTH = 14.0;  // distance between front and back wheels (inches)
        double WHEEL_DIAMETER_INCHES = 4.0;
        double TICKS_PER_REV = 560;

        // Effective turning radius = distance from center to each wheel
        double RADIUS = Math.sqrt(Math.pow(TRACK_WIDTH / 2.0, 2) + Math.pow(TRACK_LENGTH / 2.0, 2));

        // Total distance each wheel must travel for the given rotation
        double INCHES_PER_DEGREE = (2 * Math.PI * RADIUS) / 360.0;
        double wheelTravel = degrees * INCHES_PER_DEGREE;

        double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
        double targetTicks = wheelTravel * TICKS_PER_INCH;

        // ==== RESET ENCODERS ====
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // ==== SET TARGET POSITIONS ====

        int FL, FR, BL, BR;

        if (direction.equalsIgnoreCase("CW")) {
            // Clockwise: left wheels forward, right wheels backward
            FL = (int) targetTicks;
            BL = (int) targetTicks;
            FR = (int) -targetTicks;
            BR = (int) -targetTicks;

        } else { // CCW
            // Counter-clockwise: left wheels backwards, right wheels forward
            FL = (int) -targetTicks;
            BL = (int) -targetTicks;
            FR = (int) targetTicks;
            BR = (int) targetTicks;
        }

        frontLeft.setTargetPosition(FL);
        backLeft.setTargetPosition(BL);
        frontRight.setTargetPosition(FR);
        backRight.setTargetPosition(BR);

        // ==== RUN TO POSITION ====
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() ||
                        backLeft.isBusy() || backRight.isBusy())) {}

        // ==== STOP ====
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

    public void driveForward(double inches, double speed) {

        double TICKS_PER_REV = 560;
        double WHEEL_DIAMETER_INCHES = 4.0;

        double TICKS_PER_INCH = (TICKS_PER_REV) / (Math.PI * WHEEL_DIAMETER_INCHES);

        double targetTicks = inches * TICKS_PER_INCH;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // All motors forward
        frontLeft.setTargetPosition((int)(+targetTicks));
        frontRight.setTargetPosition((int)(+targetTicks));
        backLeft.setTargetPosition((int)(+targetTicks));
        backRight.setTargetPosition((int)(+targetTicks));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() ||
                        backLeft.isBusy() || backRight.isBusy())) {}

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void strafeRight(double inches, double speed) {

        double TICKS_PER_REV = 560;
        double WHEEL_DIAMETER_INCHES = 4.0;
        double STRAFING_EFFICIENCY = 1.2;

        double TICKS_PER_INCH = (TICKS_PER_REV) / (Math.PI * WHEEL_DIAMETER_INCHES);

        double targetTicks = inches * TICKS_PER_INCH * STRAFING_EFFICIENCY;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Opposite of LEFT strafing
        frontLeft.setTargetPosition((int)(+targetTicks));
        frontRight.setTargetPosition((int)(-targetTicks));
        backLeft.setTargetPosition((int)(-targetTicks));
        backRight.setTargetPosition((int)(+targetTicks));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() ||
                        backLeft.isBusy() || backRight.isBusy())) {}

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void strafeLeft(double inches, double speed) {

        // ==== CONFIGURABLE CONSTANTS ====
        double TICKS_PER_REV = 560;      // typical FTC motor
        double WHEEL_DIAMETER_INCHES = 4.0;
        double STRAFING_EFFICIENCY = 1.2; // due to slippage, tune this

        double TICKS_PER_INCH = (TICKS_PER_REV) / (Math.PI * WHEEL_DIAMETER_INCHES);

        // Apply strafing multiplier
        double targetTicks = inches * TICKS_PER_INCH * STRAFING_EFFICIENCY;

        // ==== RESET ENCODERS ====
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // ==== SET TARGET POSITIONS ====
        // Strafing left means the wheels move in a special pattern:
        frontLeft.setTargetPosition((int)(-targetTicks));
        frontRight.setTargetPosition((int)(+targetTicks));
        backLeft.setTargetPosition((int)(+targetTicks));
        backRight.setTargetPosition((int)(-targetTicks));

        // ==== RUN TO POSITION ====
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // ==== APPLY POWER ====
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        // ==== WAIT UNTIL FINISHED ====
        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() ||
                        backLeft.isBusy() || backRight.isBusy())) {
            // optionally telemetry here
        }

        // ==== STOP MOTORS ====
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // return to normal mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}
