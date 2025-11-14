package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@TeleOp(name = "FTCDecodeCompetitionTeleOpManualPID_SYNC", group = "Competition")
public class FTCDecodeCompetitionTeleOpManualPID_SYNC extends LinearOpMode {

    // Drive motors
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    // Intake
    private DcMotor intakeMotor;
    // Launcher
    private DcMotorEx launcherLeft, launcherRight;
    // Flaps
    private Servo liftLeft, liftRight;
    private VoltageSensor battery;

    // Launcher RPM targets
    private double highRPM = 3800;
    private double lowRPM = 3500;
    private double targetRPM = 0;

    private static final double TICKS_PER_REV = 28.0;

    // Intake toggle
    private boolean intakeToggle = false;
    private boolean lastIntakeToggle = false;

    // Flaps
    private boolean flapUp = false;
    private boolean lastFlapToggle = false;
    private long flapTimer = 0;
    private final double FLAP_UP_POS = 0.5;
    private final double FLAP_DOWN_POS = 0.0;

    // Drive constants
    private final double DRIVE_RPM = 2500;
    private final double DRIVE_TICKS_PER_SEC = (DRIVE_RPM / 60.0) * TICKS_PER_REV;

    // --- AprilTag stuff ---
    private OpenCvCamera camera;
    private OpenCvPipeline aprilTagPipeline;
    private AprilTagDetection tagOfInterest = null;
    private double yaw = 0.0;
    private double pitch = 0.0;

    // Control constants for automatic corrections
    private double yawThreshold = 2.0;
    private double pitchThreshold = 2.0;

    @Override
    public void runOpMode() {
        initHardware();
        initCamera();

        waitForStart();

        while (opModeIsActive()) {
            updateAprilTag();
            centerOnTag();

            mecanumDriveSync();
            controlIntake();
            controlFlaps();
            updateLauncherVelocitySync();
            updateTelemetry();
        }
    }

    private void initHardware() {
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

        battery = hardwareMap.voltageSensor.iterator().next();
    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Example pipeline (replace with your AprilTag pipeline)
        aprilTagPipeline = new OpenCvPipeline() {};
        camera.setPipeline(aprilTagPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT); }
            @Override
            public void onError(int errorCode) {}
        });
    }

    private void updateAprilTag() {
        // Simulate tag detection (replace with actual pipeline)
        tagOfInterest = null;
        yaw = 0.0;
        pitch = 0.0;
    }

    private void centerOnTag() {
        if (tagOfInterest == null) return;

        double x = 0.0;
        double y = 0.0;
        double rx = 0.0;

        double kYaw = 0.01;
        double kPitch = 0.01;

        if (Math.abs(yaw) > yawThreshold) rx = -yaw * kYaw;
        if (Math.abs(pitch) > pitchThreshold) y = -pitch * kPitch;

        double speedScale = 0.5;
        frontLeft.setVelocity((y + x + rx) * DRIVE_TICKS_PER_SEC * speedScale);
        frontRight.setVelocity((y - x - rx) * DRIVE_TICKS_PER_SEC * speedScale);
        backLeft.setVelocity((y - x + rx) * DRIVE_TICKS_PER_SEC * speedScale);
        backRight.setVelocity((y + x - rx) * DRIVE_TICKS_PER_SEC * speedScale);
    }

    /** Mecanum drive with synchronized RPM for straight motion and immediate stop **/
    private void mecanumDriveSync() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;

        double DEADZONE = 0.05;
        if (Math.abs(y) < DEADZONE) y = 0;
        if (Math.abs(x) < DEADZONE) x = 0;
        if (Math.abs(rx) < DEADZONE) rx = 0;

        double speedScale = gamepad1.left_trigger > 0.5 ? 0.4 : 1.0;

        double fl = (y + x + rx) * speedScale;
        double fr = (y - x - rx) * speedScale;
        double bl = (y - x + rx) * speedScale;
        double br = (y + x - rx) * speedScale;

        // Prevent overspeed and normalize
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max; fr /= max; bl /= max; br /= max;

        // Stop motors immediately if all sticks near zero
        if (Math.abs(y) < DEADZONE && Math.abs(x) < DEADZONE && Math.abs(rx) < DEADZONE) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return;
        }

        double velocityFL = fl * DRIVE_TICKS_PER_SEC;
        double velocityFR = fr * DRIVE_TICKS_PER_SEC;
        double velocityBL = bl * DRIVE_TICKS_PER_SEC;
        double velocityBR = br * DRIVE_TICKS_PER_SEC;

        frontLeft.setVelocity(velocityFL);
        frontRight.setVelocity(velocityFR);
        backLeft.setVelocity(velocityBL);
        backRight.setVelocity(velocityBR);

        telemetry.addData("Speed Mode", speedScale == 1.0 ? "FAST" : "SLOW");
    }

    /** Launcher velocity sync with voltage compensation **/
    private void updateLauncherVelocitySync() {
        if (gamepad1.right_trigger > 0.5) targetRPM = highRPM;
        else if (gamepad1.left_trigger > 0.5) targetRPM = lowRPM;
        else targetRPM = 0;

        double batteryVoltage = battery.getVoltage();
        double compensation = 12.0 / batteryVoltage; // simple voltage compensation

        double velocity = targetRPM > 50 ? (targetRPM / 60.0) * TICKS_PER_REV * compensation : 0.0;
        launcherLeft.setVelocity(velocity);
        launcherRight.setVelocity(velocity);
    }

    private void controlIntake() {
        if (gamepad1.right_bumper && !lastIntakeToggle) intakeToggle = !intakeToggle;
        lastIntakeToggle = gamepad1.right_bumper;

        double power = 0.0;
        if (gamepad1.a) power = 1.0;
        else if (gamepad1.b) power = -1.0;
        else if (intakeToggle) power = 1.0;

        intakeMotor.setPower(power);
        telemetry.addData("Intake Power", power);
    }

    private void controlFlaps() {
        if (gamepad1.left_bumper && !lastFlapToggle) {
            flapUp = !flapUp;
            if (flapUp) flapTimer = System.currentTimeMillis();
        }
        lastFlapToggle = gamepad1.left_bumper;

        if (flapUp && System.currentTimeMillis() - flapTimer > 500) flapUp = false;

        liftLeft.setPosition(flapUp ? FLAP_UP_POS : FLAP_DOWN_POS);
        liftRight.setPosition(flapUp ? 1 - FLAP_UP_POS : 1 - FLAP_DOWN_POS);

        telemetry.addData("Flap", flapUp ? "UP" : "DOWN");
    }

    private void updateTelemetry() {
        telemetry.addData("Target RPM", targetRPM);

        double leftRPM = launcherLeft.getVelocity() / TICKS_PER_REV * 60.0;
        double rightRPM = launcherRight.getVelocity() / TICKS_PER_REV * 60.0;

        telemetry.addData("Left RPM", leftRPM);
        telemetry.addData("Right RPM", rightRPM);
        telemetry.addData("Yaw", yaw);
        telemetry.addData("Pitch", pitch);
        telemetry.update();
    }
}
