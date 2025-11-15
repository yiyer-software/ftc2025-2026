package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auto_Final_Wall_Basket", group = "Auton")
public class Auto_Final_Wall_Basket extends LinearOpMode{
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeMotor;
    private DcMotorEx launcherLeft, launcherRight;
    private Servo liftLeft, liftRight;
    private static final double HIGH_RPM = -500;
    private static final double LOW_RPM = -300;
    private double targetRPM = 0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_DRIVE_RPM = 6000;
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
        liftLeft.setPosition(FLAP_DOWN_POS);
        liftRight.setPosition(FLAP_DOWN_POS);
        waitForStart();

        while (opModeIsActive()){
            driveBack(5, 0.75);
            launcherLeft.setVelocity(DRIVE_TICKS_PER_SEC);
            launcherRight.setVelocity(DRIVE_TICKS_PER_SEC);

            telemetry.addData("Flywheel", "Spinning to %.0f RPM", MAX_DRIVE_RPM);
            telemetry.update();

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

    private void driveBack(double inches, double power) {
        int target = inchesToTicks(inches);

        // Reverse target because backward movement = negative encoder direction
        target = -Math.abs(target);

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target positions
        frontLeft.setTargetPosition(target);
        frontRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setTargetPosition(target);

        // RUN_TO_POSITION mode
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        // Wait until all motors reach target
        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {

            telemetry.addData("Driving Back", "Target: %d", target);
            telemetry.addData("FL", frontLeft.getCurrentPosition());
            telemetry.addData("FR", frontRight.getCurrentPosition());
            telemetry.addData("BL", backLeft.getCurrentPosition());
            telemetry.addData("BR", backRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop and switch back to regular mode
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
