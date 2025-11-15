package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoBackField_V1", group = "Auton")
public class AutoBackField_V1 extends LinearOpMode {

    // Drive motors (all four used directly)
    private DcMotorEx leftFront, rightFront, leftRear, rightRear;
    // Shooter / intake hardware
    private DcMotorEx flywheelLeft, flywheelRight;
    private DcMotor intake;
    // Two lift servos
    private Servo leftLift, rightLift;
    // --- TUNE THESE to match your hardware ---
    private static final double DRIVE_TICKS_PER_REV = 537.6; // change if different
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double DRIVE_GEAR_RATIO = 1.0;

    // Flywheel & lift tuning
    private static final double FLYWHEEL_RPM = 400.0;        // target RPM (unchanged)
    private static final double LIFT_POS_DOWN = 0.15;
    // LOWERED lift height so servos raise less than before:
    private static final double LIFT_POS_UP   = 0.40;         // lowered from 0.70

    // *** Slower / more realistic timing (adjust if needed) ***
    private static final long   LIFT_UP_MS        = 650;     // hold up to feed
    private static final long   LIFT_DOWN_MS      = 700;     // gap between cycles
    private static final long   SERVO_TRAVEL_MS   = 600;     // ms to move servo between positions
    private static final double INTAKE_POWER      = 0.50;    // intake speed when running

    // Time-based intake per ball (runs for this long around each feed)
    private static final long   INTAKE_PRESPIN_MS = 200;     // let intake spin a bit before lifting
    private static final long   INTAKE_RUN_MS     = 900;     // total intake runtime for that ball (approx)

    // Edit this variables as needed
    private static double forwardMovement = 1;
    private static double turnAngle = 40;
    private static  double horizontalMovement = -4;

    @Override
    public void runOpMode() throws InterruptedException {

        // ------------------- Hardware map -------------------
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");

        flywheelLeft  = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        intake        = hardwareMap.get(DcMotor.class,   "intake");

        leftLift  = hardwareMap.get(Servo.class, "leftLift");
        rightLift = hardwareMap.get(Servo.class, "rightLift");

        // -------------- Motor directions & modes --------------
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize lifts down
        leftLift.setPosition(1-LIFT_POS_DOWN);
        rightLift.setPosition(LIFT_POS_DOWN);
        //Mechanism ready
        telemetry.addData("Status", "Init complete - Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // --------- Start flywheels ----------
        double flyTicksPerRev = flywheelLeft.getMotorType().getTicksPerRev();
        double flyTicksPerSec = (FLYWHEEL_RPM * flyTicksPerRev) / 60.0;
        flywheelLeft.setVelocity(flyTicksPerSec);
        flywheelRight.setVelocity(flyTicksPerSec);
        telemetry.addData("Flywheel", "Spinning to %.0f RPM", FLYWHEEL_RPM);
        telemetry.update();
        sleep(900); // spin-up
        // --------- Drive FORWARD, TURN, Drive Right a precise distance (inches and degrees) using encoders ----------
        driveForwardInches(forwardMovement, 0.30);
        sleep(500);
        driveTurn(turnAngle);
        sleep(300);
        driveRightInches(horizontalMovement, 0.30);
        // --------- Fire first preloaded ball (lift once and reset) ----------
        moveLiftTo(LIFT_POS_UP, SERVO_TRAVEL_MS);
        sleep(LIFT_UP_MS);
        moveLiftTo(LIFT_POS_DOWN, SERVO_TRAVEL_MS);
        sleep(350); // small settle

        // --------- Time-based intake + feeding remaining balls ----------
        // We will feed 2 more balls. For each ball: run intake for a fixed time around the lift cycle.
        for (int i = 0; i < 2 && opModeIsActive(); i++) {
            // Start intake for this ball (time-based)
            intake.setPower(INTAKE_POWER);
            sleep(INTAKE_PRESPIN_MS); // let it grab the ball before lifting

            // Lift to feed while intake is running
            moveLiftTo(LIFT_POS_UP, SERVO_TRAVEL_MS);
            // Keep lift up while intake continues
            // Wait either INTAKE_RUN_MS (since intake started) or LIFT_UP_MS, whichever is longer.
            long waitFor = Math.max(INTAKE_RUN_MS - INTAKE_PRESPIN_MS, LIFT_UP_MS);
            sleep(waitFor);
            // Lower lift
            moveLiftTo(LIFT_POS_DOWN, SERVO_TRAVEL_MS);
            sleep(LIFT_DOWN_MS);
            // Stop intake for this ball (we used it for a fixed time window)
            intake.setPower(0);
            telemetry.addData("Shot", "%d/3", i + 2);
            telemetry.update();
        }

        // Small delay to ensure last ball clears mechanism
        sleep(350);

        telemetry.addData("Status", "Done - flywheels still spinning");
        telemetry.update();
    }

    /**
     * Smoothly moves both lift servos from their current position to targetPos over travelMs.
     * Moves LEFT and RIGHT together with the same timing so they appear synchronous.
     *
     * NOTE: right servo is mirrored relative to left (common mechanical layout). If your servos
     * are mounted the same direction, change finalRightTarget = targetPos instead of (1 - targetPos).
     */
    private void moveLiftTo(double targetPos, long travelMs) throws InterruptedException {
        if (travelMs < 20) travelMs = 20;

        // read current positions
        double startLeft = leftLift.getPosition();
        double startRight = rightLift.getPosition();

        // right servo target is mirrored
        double finalLeftTarget = 1-targetPos;

        // Use more steps for smoother, synchronous motion
        int steps = 20;
        long stepMs = Math.max(1, travelMs / steps);

        for (int s = 1; s <= steps && opModeIsActive(); s++) {
            double t = (double)s / steps;
            double posL = startLeft + (finalLeftTarget - startLeft) * t;
            double posR = startRight + (targetPos - startRight) * t;
            leftLift.setPosition(posL);
            rightLift.setPosition(posR);
            sleep(stepMs);
        }

        // Ensure final exact position
        leftLift.setPosition(finalLeftTarget);
        rightLift.setPosition(targetPos);
    }

    /**
     * Drives the robot forward the given distance in inches using RUN_TO_POSITION.
     * Positive inches -> forward relative to motor directions configured above.
     */
    private void driveForwardInches(double inches, double power) {
        if (!opModeIsActive()) return;

        double countsPerInch = (DRIVE_TICKS_PER_REV * DRIVE_GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_INCHES);
        int targetTicks = (int)Math.round(inches * countsPerInch);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(targetTicks);
        rightFront.setTargetPosition(targetTicks);
        leftRear.setTargetPosition(targetTicks);
        rightRear.setTargetPosition(targetTicks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(Math.abs(power));
        rightFront.setPower(Math.abs(power));
        leftRear.setPower(Math.abs(power));
        rightRear.setPower(Math.abs(power));

        long start = System.currentTimeMillis();
        long timeoutMs = (long)(Math.abs(inches) * 250) + 3000;

        while (opModeIsActive()
                && (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy())
                && (System.currentTimeMillis() - start) < timeoutMs) {
            telemetry.addData("Driving", "Target (in) %.2f ticks %d", inches, targetTicks);
            telemetry.addData("FL/FR", "%d / %d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
            telemetry.addData("BL/BR", "%d / %d", leftRear.getCurrentPosition(), rightRear.getCurrentPosition());
            telemetry.update();
            sleep(20);
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

private void driveTurn(double degrees) {
    if (!opModeIsActive()) return;

    // ----- Robot geometry -----
    // Estimated center-to-wheel distance (inches). Adjust if your bot diameter differs.
    final double ROBOT_DIAMETER_INCHES = 14.0;

    // ----- Encoder conversion -----
    double countsPerInch = (DRIVE_TICKS_PER_REV * DRIVE_GEAR_RATIO) /
            (Math.PI * WHEEL_DIAMETER_INCHES);

    // Arc length each wheel must travel for the turn
    double arcLength = (Math.PI * ROBOT_DIAMETER_INCHES * degrees) / 360.0;

    int turnTicks = (int) Math.round(arcLength * countsPerInch);

    // ----- Prepare motors -----
    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // CLOCKWISE TURN:
    // Left side forward  (+ticks)
    // Right side backward (-ticks)
    leftFront.setTargetPosition(turnTicks);
    leftRear.setTargetPosition(turnTicks);
    rightFront.setTargetPosition(-turnTicks);
    rightRear.setTargetPosition(-turnTicks);

    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    double power = 0.35; // safe and accurate turning power
    leftFront.setPower(power);
    rightFront.setPower(power);
    leftRear.setPower(power);
    rightRear.setPower(power);

    // Timeout based on turn magnitude
    long start = System.currentTimeMillis();
    long timeoutMs = (long)(Math.abs(degrees) * 25) + 2000;  

    // ----- Run until complete -----
    while (opModeIsActive()
            && (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy())
            && (System.currentTimeMillis() - start) < timeoutMs) {

        telemetry.addData("Turning", "Degrees %.1f  TargetTicks %d", degrees, turnTicks);
        telemetry.addData("FL/FR", "%d / %d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
        telemetry.addData("BL/BR", "%d / %d", leftRear.getCurrentPosition(), rightRear.getCurrentPosition());
        telemetry.update();

        sleep(20);
    }

    // ----- Stop -----
    leftFront.setPower(0);
    rightFront.setPower(0);
    leftRear.setPower(0);
    rightRear.setPower(0);

    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}

private void driveRightInches(double inches, double power) {
    if (!opModeIsActive()) return;

    double countsPerInch = (DRIVE_TICKS_PER_REV * DRIVE_GEAR_RATIO) /
            (Math.PI * WHEEL_DIAMETER_INCHES);
    int targetTicks = (int)Math.round(inches * countsPerInch);

    // Reset encoders
    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // --------- Mecanum RIGHT STRAFE encoder directions ---------
    // For a right strafe:
    //
    //    leftFront  → +ticks
    //    rightFront → -ticks
    //    leftRear   → -ticks
    //    rightRear  → +ticks
    //
    // These directions hold because your right motors are reversed in hardware mapping.

    leftFront.setTargetPosition( targetTicks);
    rightFront.setTargetPosition(-targetTicks);
    leftRear.setTargetPosition(-targetTicks);
    rightRear.setTargetPosition( targetTicks);

    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    power = Math.abs(power);
    leftFront.setPower(power);
    rightFront.setPower(power);
    leftRear.setPower(power);
    rightRear.setPower(power);

    // Timeout (scaled similar to forward driving)
    long start = System.currentTimeMillis();
    long timeoutMs = (long)(Math.abs(inches) * 300) + 3000;

    while (opModeIsActive()
            && (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy())
            && (System.currentTimeMillis() - start) < timeoutMs) {

        telemetry.addData("Strafing Right", "Target (in) %.2f ticks %d", inches, targetTicks);
        telemetry.addData("FL/FR", "%d / %d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
        telemetry.addData("BL/BR", "%d / %d", leftRear.getCurrentPosition(), rightRear.getCurrentPosition());
        telemetry.update();
        sleep(20);
    }

    // Stop all motors
    leftFront.setPower(0);
    rightFront.setPower(0);
    leftRear.setPower(0);
    rightRear.setPower(0);

    // Restore standard encoder mode
    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}
