package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;

@TeleOp(name="TeleOp Decode FINAL", group="Competition")
public class TeleOpDecode_FINAL extends LinearOpMode {

    // Alliance Selection
    private static final Alliance SELECTED_ALLIANCE = Alliance.BLUE;
    public enum Alliance { RED, BLUE }

    // Hardware Names
    private static final String MOTOR_FRONT_LEFT = "frontLeft", MOTOR_FRONT_RIGHT = "frontRight";
    private static final String MOTOR_BACK_LEFT = "backLeft", MOTOR_BACK_RIGHT = "backRight";
    private static final String MOTOR_LAUNCHER_LEFT = "launcherLeft", MOTOR_LAUNCHER_RIGHT = "launcherRight";
    private static final String MOTOR_INTAKE = "intakeMotor";
    private static final String SERVO_CAMERA = "cameraServo";
    private static final String SERVO_LIFT_LEFT = "liftServoLeft", SERVO_LIFT_RIGHT = "liftServoRight";
    private static final String CAMERA_NAME = "Webcam 1";

    // Camera Calibration & AprilTag Config
    private static final double CAMERA_FX = 578.272, CAMERA_FY = 578.272, CAMERA_CX = 402.145, CAMERA_CY = 221.506;
    private static final int CAMERA_WIDTH = 800, CAMERA_HEIGHT = 448;
    private static final double APRILTAG_SIZE = 0.166;
    private static final int RED_TARGET_ID = 24, BLUE_TARGET_ID = 20;
    private static final int TAG_ID_ASCENT = 21, TAG_ID_DESCENT = 22, TAG_ID_DEEP_DIVE = 23;

    // Motif Color Patterns
    private static final String[] PATTERN_ASCENT = {"GREEN", "PURPLE", "PURPLE"};
    private static final String[] PATTERN_DESCENT = {"PURPLE", "GREEN", "PURPLE"};
    private static final String[] PATTERN_DEEP_DIVE = {"PURPLE", "PURPLE", "GREEN"};

    // Gameplay & Servo Parameters
    private static final boolean ENFORCE_LAUNCH_ZONES_IN_AUTO = true;
    private static final double LAUNCH_ZONE_MAX_DISTANCE = 2.5;
    private static final double CAMERA_FORWARD_POS = 0.0, CAMERA_BACKWARD_POS = 1.0, CAMERA_CENTER_POS = 0.5;
    private static final double LIFT_DOWN_POS = 0.0, LIFT_UP_POS = 0.85;

    // Power, Speed, and Tuning
    private static final double FLYWHEEL_LAUNCH_POWER = 0.90, FLYWHEEL_IDLE_POWER = 0.30, INTAKE_POWER = 0.75;
    private static final double DRIVE_SPEED_NORMAL = 1.0, DRIVE_SPEED_SLOW = 0.5, DRIVE_DEADZONE = 0.05;
    private static final double ALIGNMENT_YAW_TOLERANCE = 2.0, ALIGNMENT_DISTANCE_OPTIMAL = 2.0;
    private static final double TURN_KP = 0.025, DRIVE_KP = 0.20, STRAFE_KP = 0.02;
    private static final double PICKUP_DRIVE_KP = 0.003, PICKUP_TURN_KP = 0.004;

    // Timing
    private static final long FLYWHEEL_SPINUP_TIME = 1000, LIFT_SERVO_MOVE_TIME = 400;
    private static final long LAUNCH_HOLD_TIME = 300, SEARCH_TIMEOUT = 7000;

    private DcMotor frontLeft, frontRight, backLeft, backRight, launcherLeft, launcherRight, intakeMotor;
    private Servo cameraServo, liftServoLeft, liftServoRight;
    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagPipeline;
    private BallDetectionPipeline ballDetectionPipeline;

    private enum ControlMode { AUTOMATIC, MANUAL }
    private ControlMode currentMode = ControlMode.AUTOMATIC;

    private enum AutoState { IDLE, SEARCHING, MOVING_TO_LAUNCH_ZONE, ALIGNING, LAUNCHING, COMPLETE,
                           START_PICKUP, SEARCHING_FOR_BALL, DRIVING_TO_BALL, INTAKING }
    private AutoState autoLaunchState = AutoState.IDLE;
    private AutoState autoPickupState = AutoState.IDLE;

    private boolean lastA, lastB, lastX, lastY, lastLeftBumper, lastRightBumper;

    private AprilTagDetection targetTag = null;
    private String[] currentPattern = {"-", "-", "-"};
    private int patternIndex = 0;
    private ElapsedTime runtime = new ElapsedTime(), autoTimer = new ElapsedTime(), searchTimer = new ElapsedTime();
    private boolean liftIsUp = false, precisionMode = false, autoSequenceStarted = false, inLaunchZone = false, motifDetected = false;
    private int ballsLaunched = 0, searchPhase = 0;

    @Override
    public void runOpMode() {
        initializeHardware();
        initializeCamera();
        displayStartupTelemetry();
        waitForStart();
        runtime.reset();
        if (!autoSequenceStarted) {
            executeStartingSequence();
            autoSequenceStarted = true;
        }
        while (opModeIsActive()) {
            updateButtonStates();
            checkLaunchZone(); // Must run before processAutoLaunch
            if (currentMode == ControlMode.MANUAL) {
                runManualMode();
            } else {
                runAutomaticMode();
            }
            updateTelemetry();
            sleep(20);
        }
        cleanup();
    }

    private void initializeHardware() { /* ... unchanged ... */ }

    private void initializeCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, CAMERA_NAME), cameraMonitorViewId);
        aprilTagPipeline = new AprilTagDetectionPipeline(APRILTAG_SIZE, CAMERA_FX, CAMERA_FY, CAMERA_CX, CAMERA_CY);
        ballDetectionPipeline = new BallDetectionPipeline(CAMERA_WIDTH);
        switchToAprilTagPipeline();
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() { camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT); }
            @Override public void onError(int errorCode) {}
        });
    }

    private void switchToAprilTagPipeline() { camera.setPipeline(aprilTagPipeline); }
    private void switchToBallDetectionPipeline() { camera.setPipeline(ballDetectionPipeline); }

    private void displayStartupTelemetry() { /* ... unchanged ... */ }

    private void executeStartingSequence() {
        setMecanumDrive(0.4, 0, 0); sleep(500);
        setMecanumDrive(0, 0, 0); sleep(200);
        inLaunchZone = true;
        for (int i = 1; i <= 3; i++) {
            launchSequence(); ballsLaunched++; sleep(500);
        }
        setCameraPosition(CAMERA_CENTER_POS); sleep(500);
        detectMotifAndSetPattern(); sleep(1000);
        currentMode = ControlMode.AUTOMATIC;
    }

    private void checkLaunchZone() {
        if (camera.getPipeline() == aprilTagPipeline) {
            targetTag = detectAllianceTarget();
            inLaunchZone = (targetTag != null && targetTag.pose.z <= LAUNCH_ZONE_MAX_DISTANCE);
        }
    }

    private void detectMotifAndSetPattern() { /* ... unchanged ... */ }
    
    private void updateButtonStates() { /* ... unchanged ... */ }
    
    private void onButtonA() { currentMode = ControlMode.AUTOMATIC; resetAutoStates(); }
    private void onButtonB() { currentMode = ControlMode.MANUAL; resetAutoStates(); stopAllMotors(); }
    
    private void onButtonX() {
        if (autoLaunchState == AutoState.IDLE) {
            autoLaunchState = AutoState.SEARCHING; autoPickupState = AutoState.IDLE; autoTimer.reset();
            setCameraPosition(CAMERA_BACKWARD_POS);
            switchToAprilTagPipeline();
        } else {
            resetAutoStates(); stopAllMotors();
        }
    }

    private void onButtonY() {
        if (autoPickupState == AutoState.IDLE) {
            autoPickupState = AutoState.START_PICKUP; autoLaunchState = AutoState.IDLE; autoTimer.reset();
            setCameraPosition(CAMERA_FORWARD_POS);
            switchToBallDetectionPipeline();
        } else {
            resetAutoStates(); stopAllMotors();
        }
    }

    private void onLeftBumper() { toggleLift(); }
    private void onRightBumper() { precisionMode = !precisionMode; }
    
    private void runManualMode() {
        if (autoLaunchState != AutoState.IDLE) { processAutoLaunch(); return; }
        if (autoPickupState != AutoState.IDLE) { processAutoPickup(); return; }
        handleDriveControl();
        handleManualFlywheelControl();
        handleIntakeControl();
    }
    
    private void handleDriveControl() { /* ... unchanged ... */ }
    private void handleManualFlywheelControl() { /* ... unchanged ... */ }
    private void handleIntakeControl() { /* ... unchanged ... */ }

    private void runAutomaticMode() {
        if (autoLaunchState != AutoState.IDLE) { processAutoLaunch(); return; }
        if (autoPickupState != AutoState.IDLE) { processAutoPickup(); return; }
        switchToAprilTagPipeline(); // Ensure we are looking for AprilTags in general auto mode
        if (targetTag != null) {
            searchPhase = 0;
            if (!inLaunchZone && ENFORCE_LAUNCH_ZONES_IN_AUTO) {
                double drivePower = (targetTag.pose.z - (LAUNCH_ZONE_MAX_DISTANCE - 0.2)) * DRIVE_KP;
                setMecanumDrive(Range.clip(drivePower, -0.6, 0.6), 0, 0);
            } else {
                alignAndLaunch();
            }
        } else {
            enhancedSearchForTarget();
        }
        if (hasManualInput()) { handleDriveControl(); }
    }
    
    private void alignAndLaunch() { /* ... unchanged ... */ }
    private void processAutoLaunch() { /* ... unchanged ... */ }
    private void launchSequence() { /* ... unchanged ... */ }

    private void processAutoPickup() {
        String targetColor = (patternIndex < currentPattern.length) ? currentPattern[patternIndex] : "NONE";
        ballDetectionPipeline.setTargetColor(targetColor);

        switch (autoPickupState) {
            case START_PICKUP:
                patternIndex = 0;
                autoPickupState = AutoState.SEARCHING_FOR_BALL;
                autoTimer.reset();
                break;
            case SEARCHING_FOR_BALL:
                setMecanumDrive(0, 0, 0.3); // Slowly turn
                if (ballDetectionPipeline.isBallFound()) {
                    stopAllMotors();
                    autoPickupState = AutoState.DRIVING_TO_BALL;
                }
                if (autoTimer.milliseconds() > SEARCH_TIMEOUT) autoPickupState = AutoState.COMPLETE;
                break;
            case DRIVING_TO_BALL:
                intakeMotor.setPower(INTAKE_POWER);
                if (!ballDetectionPipeline.isBallFound()) {
                    autoPickupState = AutoState.SEARCHING_FOR_BALL; autoTimer.reset(); break;
                }
                double drivePower = (CAMERA_HEIGHT - ballDetectionPipeline.getBallY()) * PICKUP_DRIVE_KP;
                double turnPower = ballDetectionPipeline.getBallXError() * PICKUP_TURN_KP;
                setMecanumDrive(Range.clip(drivePower, -0.4, 0.4), 0, -turnPower);
                if (ballDetectionPipeline.getBallY() > CAMERA_HEIGHT * 0.9) {
                    autoPickupState = AutoState.INTAKING; autoTimer.reset();
                }
                break;
            case INTAKING:
                setMecanumDrive(0.3, 0, 0);
                if (autoTimer.milliseconds() > 750) {
                    patternIndex++;
                    if (patternIndex >= currentPattern.length) {
                        autoPickupState = AutoState.COMPLETE;
                    } else {
                        autoPickupState = AutoState.SEARCHING_FOR_BALL; autoTimer.reset();
                    }
                }
                break;
            case COMPLETE:
            default:
                resetAutoStates(); stopAllMotors(); break;
        }
    }
    
    private AprilTagDetection detectAllianceTarget() { /* ... unchanged ... */ }
    private void alignToTarget(AprilTagDetection tag) { /* ... unchanged ... */ }
    private boolean isAlignedWithTarget(AprilTagDetection tag) { /* ... unchanged ... */ }
    private void enhancedSearchForTarget() { /* ... unchanged ... */ }
    
    private void resetAutoStates() {
        autoLaunchState = AutoState.IDLE;
        autoPickupState = AutoState.IDLE;
        searchPhase = 0;
        patternIndex = 0;
        switchToAprilTagPipeline();
    }
    
    private void setMecanumDrive(double d, double s, double t) { /* ... unchanged ... */ }
    private void setFlywheelPower(double p) { /* ... unchanged ... */ }
    private void setLiftPosition(boolean u) { /* ... unchanged ... */ }
    private void toggleLift() { setLiftPosition(!liftIsUp); }
    private void setCameraPosition(double p) { cameraServo.setPosition(p); }
    private void stopAllMotors() { /* ... unchanged ... */ }
    private void setDriveBehavior(DcMotor.ZeroPowerBehavior b) { /* ... unchanged ... */ }
    private double applyDeadzone(double v) { return Math.abs(v) < DRIVE_DEADZONE ? 0 : v; }
    private boolean hasManualInput() { return Math.abs(gamepad1.left_stick_y) > DRIVE_DEADZONE || Math.abs(gamepad1.left_stick_x) > DRIVE_DEADZONE || Math.abs(gamepad1.right_stick_x) > DRIVE_DEADZONE; }
    
    private void updateTelemetry() {
        telemetry.addData("PATTERN", String.format("[%s] [%s] [%s]", currentPattern[0], currentPattern[1], currentPattern[2]));
        telemetry.addData("Mode", currentMode);
        if(autoPickupState != AutoState.IDLE) {
            telemetry.addData("Auto-Pickup", autoPickupState);
            if (patternIndex < currentPattern.length) telemetry.addData("--> Seeking", "[%s]", currentPattern[patternIndex]);
        }
        if (camera.getPipeline() == aprilTagPipeline && targetTag != null) {
            telemetry.addData("Tag", "%d | Dist: %.2f", targetTag.id, targetTag.pose.z);
        } else if (camera.getPipeline() == ballDetectionPipeline && ballDetectionPipeline.isBallFound()) {
            telemetry.addData("Ball", "X: %.0f, Y: %.0f", ballDetectionPipeline.getBallX(), ballDetectionPipeline.getBallY());
        } else {
            telemetry.addLine("No Target Detected");
        }
        telemetry.update();
    }
    
    private void cleanup() { if (camera != null) { camera.stopStreaming(); } stopAllMotors(); }
}
