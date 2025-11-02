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

@TeleOp(name="TeleOp Decode V25N05 FINAL", group="Competition")
public class TeleOpDecodeV25N05 extends LinearOpMode {

    // Alliance Selection
    private static final Alliance SELECTED_ALLIANCE = Alliance.BLUE;
    public enum Alliance { RED, BLUE }

    // Hardware Names
    private static final String MOTOR_FRONT_LEFT = "frontLeft";
    private static final String MOTOR_FRONT_RIGHT = "frontRight";
    private static final String MOTOR_BACK_LEFT = "backLeft";
    private static final String MOTOR_BACK_RIGHT = "backRight";
    private static final String MOTOR_LAUNCHER_LEFT = "launcherLeft";
    private static final String MOTOR_LAUNCHER_RIGHT = "launcherRight";
    private static final String MOTOR_INTAKE = "intakeMotor";
    private static final String SERVO_CAMERA = "cameraServo";
    private static final String SERVO_LIFT_LEFT = "liftServoLeft";
    private static final String SERVO_LIFT_RIGHT = "liftServoRight";
    private static final String CAMERA_NAME = "Webcam 1";

    // Camera Calibration
    private static final double CAMERA_FX = 578.272;
    private static final double CAMERA_FY = 578.272;
    private static final double CAMERA_CX = 402.145;
    private static final double CAMERA_CY = 221.506;
    private static final int CAMERA_WIDTH = 800;
    private static final int CAMERA_HEIGHT = 448;
    private static final double APRILTAG_SIZE = 0.166;

    // AprilTag IDs
    private static final int RED_TARGET_ID = 5;      // Single Red Alliance Goal
    private static final int BLUE_TARGET_ID = 2;     // Single Blue Alliance Goal
    private static final int TAG_ID_ASCENT = 11;
    private static final int TAG_ID_DESCENT = 12;
    private static final int TAG_ID_DEEP_DIVE = 13;

    // Motif Color Patterns
    private static final String[] PATTERN_ASCENT = {"RED", "GREEN", "BLUE"};
    private static final String[] PATTERN_DESCENT = {"GREEN", "BLUE", "RED"};
    private static final String[] PATTERN_DEEP_DIVE = {"BLUE", "RED", "GREEN"};

    // Gameplay Parameters
    private static final boolean ENFORCE_LAUNCH_ZONES_IN_AUTO = true;
    private static final double LAUNCH_ZONE_MAX_DISTANCE = 2.5;

    // Servo Positions
    private static final double CAMERA_FORWARD_POS = 0.0;
    private static final double CAMERA_BACKWARD_POS = 1.0;
    private static final double CAMERA_CENTER_POS = 0.5;
    private static final double LIFT_DOWN_POS = 0.0;
    private static final double LIFT_UP_POS = 0.85;

    // Motor Power Levels
    private static final double FLYWHEEL_LAUNCH_POWER = 0.90;
    private static final double FLYWHEEL_IDLE_POWER = 0.30;
    private static final double INTAKE_POWER = 0.75;

    // Drive Parameters
    private static final double DRIVE_SPEED_NORMAL = 1.0;
    private static final double DRIVE_SPEED_SLOW = 0.5;
    private static final double DRIVE_DEADZONE = 0.05;

    // Auto-Alignment Parameters
    private static final double ALIGNMENT_YAW_TOLERANCE = 2.0;
    private static final double ALIGNMENT_DISTANCE_OPTIMAL = 2.0;
    private static final double TURN_KP = 0.025;
    private static final double DRIVE_KP = 0.20;
    private static final double STRAFE_KP = 0.02;

    // Timing Parameters
    private static final long FLYWHEEL_SPINUP_TIME = 1000;
    private static final long LIFT_SERVO_MOVE_TIME = 400;
    private static final long LAUNCH_HOLD_TIME = 300;
    private static final long SEARCH_TIMEOUT = 7000;

    // Motor Directions
    private static final boolean REVERSE_FRONT_LEFT = true;
    private static final boolean REVERSE_FRONT_RIGHT = false;
    private static final boolean REVERSE_BACK_LEFT = true;
    private static final boolean REVERSE_BACK_RIGHT = false;
    private static final boolean REVERSE_LAUNCHER_LEFT = false;
    private static final boolean REVERSE_LAUNCHER_RIGHT = true;

    // Behavior Flags
    private static final boolean ENABLE_FLYWHEEL_IDLE = true;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor launcherLeft, launcherRight, intakeMotor;
    private Servo cameraServo, liftServoLeft, liftServoRight;
    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagPipeline;

    private enum ControlMode { AUTOMATIC, MANUAL }
    private ControlMode currentMode = ControlMode.AUTOMATIC;

    private enum AutoState { IDLE, SEARCHING, MOVING_TO_LAUNCH_ZONE, ALIGNING, LAUNCHING, PICKING_UP, COMPLETE }
    private AutoState autoLaunchState = AutoState.IDLE;
    private AutoState autoPickupState = AutoState.IDLE;

    private boolean lastA, lastB, lastX, lastY;
    private boolean lastLeftBumper, lastRightBumper;

    private AprilTagDetection targetTag = null;
    private String[] currentPattern = {"-", "-", "-"};
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime autoTimer = new ElapsedTime();
    private boolean liftIsUp = false;
    private boolean precisionMode = false;
    private int ballsLaunched = 0;
    private boolean autoSequenceStarted = false;
    private boolean inLaunchZone = false;
    private boolean motifDetected = false;
    private int searchPhase = 0;
    private ElapsedTime searchTimer = new ElapsedTime();

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
            handleModeSelection();
            checkLaunchZone();

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

    private void initializeHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, MOTOR_FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, MOTOR_FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, MOTOR_BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, MOTOR_BACK_RIGHT);
        launcherLeft = hardwareMap.get(DcMotor.class, MOTOR_LAUNCHER_LEFT);
        launcherRight = hardwareMap.get(DcMotor.class, MOTOR_LAUNCHER_RIGHT);
        intakeMotor = hardwareMap.get(DcMotor.class, MOTOR_INTAKE);
        cameraServo = hardwareMap.get(Servo.class, SERVO_CAMERA);
        liftServoLeft = hardwareMap.get(Servo.class, SERVO_LIFT_LEFT);
        liftServoRight = hardwareMap.get(Servo.class, SERVO_LIFT_RIGHT);

        frontLeft.setDirection(REVERSE_FRONT_LEFT ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        frontRight.setDirection(REVERSE_FRONT_RIGHT ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        backLeft.setDirection(REVERSE_BACK_LEFT ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        backRight.setDirection(REVERSE_BACK_RIGHT ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(REVERSE_LAUNCHER_LEFT ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        launcherRight.setDirection(REVERSE_LAUNCHER_RIGHT ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        setDriveBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        cameraServo.setPosition(CAMERA_BACKWARD_POS);
        setLiftPosition(false);
    }

    private void initializeCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, CAMERA_NAME), cameraMonitorViewId);
        aprilTagPipeline = new AprilTagDetectionPipeline(APRILTAG_SIZE, CAMERA_FX, CAMERA_FY, CAMERA_CX, CAMERA_CY);
        camera.setPipeline(aprilTagPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT); }
            @Override
            public void onError(int errorCode) { }
        });
    }

    private void displayStartupTelemetry() {
        telemetry.addLine("FTC DECODE TeleOp FINAL");
        telemetry.addData("Alliance", SELECTED_ALLIANCE);
        telemetry.addData("Default Mode", "AUTOMATIC");
        telemetry.addLine("Ready to Start!");
        telemetry.update();
    }

    private void executeStartingSequence() {
        setMecanumDrive(0.4, 0, 0);
        sleep(500);
        setMecanumDrive(0, 0, 0);
        sleep(200);

        inLaunchZone = true;
        for (int i = 1; i <= 3; i++) {
            launchSequence();
            ballsLaunched++;
            sleep(500);
        }

        setCameraPosition(CAMERA_CENTER_POS);
        sleep(500);
        detectMotifAndSetPattern();
        sleep(1000);

        currentMode = ControlMode.AUTOMATIC;
        autoLaunchState = AutoState.IDLE;
    }

    private void checkLaunchZone() {
        targetTag = detectAllianceTarget();
        inLaunchZone = (targetTag != null && targetTag.pose.z <= LAUNCH_ZONE_MAX_DISTANCE);
    }

    private void detectMotifAndSetPattern() {
        if (motifDetected) return;
        ArrayList<AprilTagDetection> detections = aprilTagPipeline.getLatestDetections();
        if (detections == null || detections.isEmpty()) return;
        for (AprilTagDetection detection : detections) {
            if (detection.id == TAG_ID_ASCENT) { currentPattern = PATTERN_ASCENT; motifDetected = true; return; }
            if (detection.id == TAG_ID_DESCENT) { currentPattern = PATTERN_DESCENT; motifDetected = true; return; }
            if (detection.id == TAG_ID_DEEP_DIVE) { currentPattern = PATTERN_DEEP_DIVE; motifDetected = true; return; }
        }
    }

    private void updateButtonStates() {
        boolean currentA = gamepad1.a, currentB = gamepad1.b, currentX = gamepad1.x, currentY = gamepad1.y;
        boolean currentLB = gamepad1.left_bumper, currentRB = gamepad1.right_bumper;
        if (currentA && !lastA) onButtonA();
        if (currentB && !lastB) onButtonB();
        if (currentX && !lastX) onButtonX();
        if (currentY && !lastY) onButtonY();
        if (currentLB && !lastLeftBumper) onLeftBumper();
        if (currentRB && !lastRightBumper) onRightBumper();
        lastA = currentA; lastB = currentB; lastX = currentX; lastY = currentY;
        lastLeftBumper = currentLB; lastRightBumper = currentRB;
    }

    private void handleModeSelection() {}

    private void onButtonA() { currentMode = ControlMode.AUTOMATIC; resetAutoStates(); gamepad1.rumble(100); }
    private void onButtonB() { currentMode = ControlMode.MANUAL; resetAutoStates(); stopAllMotors(); gamepad1.rumble(100); }
    private void onButtonX() {
        if (autoLaunchState == AutoState.IDLE) {
            autoLaunchState = AutoState.SEARCHING;
            autoPickupState = AutoState.IDLE;
            autoTimer.reset();
            setCameraPosition(CAMERA_BACKWARD_POS);
            gamepad1.rumble(200);
        } else {
            resetAutoStates();
            stopAllMotors();
            gamepad1.rumble(50);
        }
    }
    private void onButtonY() {
        if (autoPickupState == AutoState.IDLE) {
            autoPickupState = AutoState.SEARCHING;
            autoLaunchState = AutoState.IDLE;
            autoTimer.reset();
            setCameraPosition(CAMERA_FORWARD_POS);
            gamepad1.rumble(200);
        } else {
            autoPickupState = AutoState.IDLE;
            stopAllMotors();
            gamepad1.rumble(50);
        }
    }
    private void onLeftBumper() { toggleLift(); gamepad1.rumble(50); }
    private void onRightBumper() { precisionMode = !precisionMode; gamepad1.rumble(50); }

    private void runManualMode() {
        if (autoLaunchState != AutoState.IDLE) { processAutoLaunch(); return; }
        if (autoPickupState != AutoState.IDLE) { processAutoPickup(); return; }
        handleDriveControl();
        handleManualFlywheelControl();
        handleIntakeControl();
    }

    private void handleDriveControl() {
        double drive = -gamepad1.left_stick_y, strafe = gamepad1.left_stick_x, turn = gamepad1.right_stick_x;
        drive = applyDeadzone(drive); strafe = applyDeadzone(strafe); turn = applyDeadzone(turn);
        double speed = precisionMode ? DRIVE_SPEED_SLOW : DRIVE_SPEED_NORMAL;
        setMecanumDrive(drive * speed, strafe * speed, turn * speed);
    }

    private void handleManualFlywheelControl() {
        double trigger = gamepad1.left_trigger;
        if (trigger > 0.1) {
            double power = Range.scale(trigger, 0, 1, FLYWHEEL_IDLE_POWER, FLYWHEEL_LAUNCH_POWER);
            setFlywheelPower(power);
        } else {
            setFlywheelPower(ENABLE_FLYWHEEL_IDLE ? FLYWHEEL_IDLE_POWER : 0);
        }
    }

    private void handleIntakeControl() {
        double trigger = gamepad1.right_trigger;
        if (trigger > 0.1) { intakeMotor.setPower(trigger * INTAKE_POWER); }
        else { intakeMotor.setPower(0); }
    }

    private void runAutomaticMode() {
        if (autoLaunchState != AutoState.IDLE) { processAutoLaunch(); return; }
        targetTag = detectAllianceTarget();
        if (targetTag != null) {
            searchPhase = 0;
            checkLaunchZone();
            if (!inLaunchZone && ENFORCE_LAUNCH_ZONES_IN_AUTO) {
                double distance = targetTag.pose.z;
                double drivePower = (distance - (LAUNCH_ZONE_MAX_DISTANCE - 0.2)) * DRIVE_KP;
                setMecanumDrive(Range.clip(drivePower, -0.6, 0.6), 0, 0);
            } else {
                alignAndLaunch();
            }
        } else {
            enhancedSearchForTarget();
        }
        if (hasManualInput()) { handleDriveControl(); }
    }

    private void alignAndLaunch() {
        if (!isAlignedWithTarget(targetTag)) {
            alignToTarget(targetTag);
        } else {
            launchSequence();
            ballsLaunched++;
            stopAllMotors();
            sleep(500);
        }
    }
    
    private void processAutoLaunch() {
        switch (autoLaunchState) {
            case SEARCHING:
                targetTag = detectAllianceTarget();
                if (targetTag != null) {
                    checkLaunchZone();
                    autoLaunchState = (inLaunchZone || !ENFORCE_LAUNCH_ZONES_IN_AUTO) ? AutoState.ALIGNING : AutoState.MOVING_TO_LAUNCH_ZONE;
                    autoTimer.reset();
                } else {
                    enhancedSearchForTarget();
                    if (autoTimer.milliseconds() > SEARCH_TIMEOUT) { autoLaunchState = AutoState.COMPLETE; gamepad1.rumble(500); }
                }
                break;
            case MOVING_TO_LAUNCH_ZONE:
                targetTag = detectAllianceTarget();
                if (targetTag != null) {
                    double distance = targetTag.pose.z;
                    if (distance > LAUNCH_ZONE_MAX_DISTANCE) {
                        double drivePower = (distance - (LAUNCH_ZONE_MAX_DISTANCE - 0.2)) * DRIVE_KP;
                        setMecanumDrive(Range.clip(drivePower, -0.6, 0.6), 0, 0);
                    } else {
                        stopAllMotors();
                        autoLaunchState = AutoState.ALIGNING;
                    }
                } else { autoLaunchState = AutoState.SEARCHING; }
                break;
            case ALIGNING:
                targetTag = detectAllianceTarget();
                if (targetTag != null) {
                    alignToTarget(targetTag);
                    if (isAlignedWithTarget(targetTag)) { autoLaunchState = AutoState.LAUNCHING; }
                } else { autoLaunchState = AutoState.SEARCHING; }
                break;
            case LAUNCHING:
                launchSequence(); ballsLaunched++; autoLaunchState = AutoState.COMPLETE; gamepad1.rumble(1000);
                break;
            case COMPLETE:
            default:
                resetAutoStates(); stopAllMotors(); break;
        }
    }

    private void launchSequence() {
        checkLaunchZone();
        if (!inLaunchZone && ENFORCE_LAUNCH_ZONES_IN_AUTO && currentMode == ControlMode.AUTOMATIC) { return; }
        setFlywheelPower(FLYWHEEL_LAUNCH_POWER);
        sleep(FLYWHEEL_SPINUP_TIME);
        setLiftPosition(true);
        sleep(LIFT_SERVO_MOVE_TIME);
        sleep(LAUNCH_HOLD_TIME);
        setLiftPosition(false);
        sleep(LIFT_SERVO_MOVE_TIME);
        setFlywheelPower(ENABLE_FLYWHEEL_IDLE ? FLYWHEEL_IDLE_POWER : 0);
    }

    private void processAutoPickup() {
        switch (autoPickupState) {
            case SEARCHING:
                intakeMotor.setPower(INTAKE_POWER); setMecanumDrive(0.35, 0, 0);
                autoPickupState = AutoState.PICKING_UP; autoTimer.reset();
                break;
            case PICKING_UP:
                if (autoTimer.milliseconds() > 5000) { autoPickupState = AutoState.COMPLETE; }
                break;
            case COMPLETE:
            default:
                stopAllMotors(); autoPickupState = AutoState.IDLE; gamepad1.rumble(500); break;
        }
    }

    private AprilTagDetection detectAllianceTarget() {
        ArrayList<AprilTagDetection> detections = aprilTagPipeline.getLatestDetections();
        if (detections == null || detections.isEmpty()) return null;
        int targetId = (SELECTED_ALLIANCE == Alliance.BLUE) ? BLUE_TARGET_ID : RED_TARGET_ID;
        for (AprilTagDetection detection : detections) {
            if (detection.id == targetId) {
                return detection;
            }
        }
        return null;
    }

    private void alignToTarget(AprilTagDetection tag) {
        Orientation rot = Orientation.getOrientation(tag.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        double yawError = rot.firstAngle;
        double distError = tag.pose.z - ALIGNMENT_DISTANCE_OPTIMAL;
        double strafeError = tag.pose.x;
        double turnPower = -yawError * TURN_KP;
        double drivePower = -distError * DRIVE_KP;
        double strafePower = -strafeError * STRAFE_KP;
        setMecanumDrive(drivePower, strafePower, turnPower);
    }

    private boolean isAlignedWithTarget(AprilTagDetection tag) {
        Orientation rot = Orientation.getOrientation(tag.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        return Math.abs(rot.firstAngle) < ALIGNMENT_YAW_TOLERANCE && Math.abs(tag.pose.z - ALIGNMENT_DISTANCE_OPTIMAL) < 0.1;
    }

    private void enhancedSearchForTarget() {
        switch (searchPhase) {
            case 0: setMecanumDrive(0, 0, -0.4); searchTimer.reset(); searchPhase = 1; break;
            case 1: if (searchTimer.milliseconds() > 400) { setMecanumDrive(0, 0, 0.4); searchTimer.reset(); searchPhase = 2; } break;
            case 2: if (searchTimer.milliseconds() > 800) { setMecanumDrive(0, 0, -0.4); searchTimer.reset(); searchPhase = 3; } break;
            case 3: if (searchTimer.milliseconds() > 400) { stopAllMotors(); searchPhase = 4; } break;
            case 4: setMecanumDrive(0, 0, 0.25); break;
        }
    }
    
    private void resetAutoStates() { autoLaunchState = AutoState.IDLE; autoPickupState = AutoState.IDLE; searchPhase = 0; }
    private void setMecanumDrive(double drive, double strafe, double turn) {
        double fL = drive + strafe + turn, fR = drive - strafe - turn;
        double bL = drive - strafe + turn, bR = drive + strafe - turn;
        double max = Math.max(1.0, Math.max(Math.abs(fL), Math.max(Math.abs(fR), Math.max(Math.abs(bL), Math.abs(bR)))));
        frontLeft.setPower(fL / max); frontRight.setPower(fR / max);
        backLeft.setPower(bL / max); backRight.setPower(bR / max);
    }
    private void setFlywheelPower(double power) { launcherLeft.setPower(power); launcherRight.setPower(power); }
    private void setLiftPosition(boolean up) { double pos = up ? LIFT_UP_POS : LIFT_DOWN_POS; liftServoLeft.setPosition(pos); liftServoRight.setPosition(pos); liftIsUp = up; }
    private void toggleLift() { setLiftPosition(!liftIsUp); }
    private void setCameraPosition(double position) { cameraServo.setPosition(position); }
    private void stopAllMotors() { setMecanumDrive(0, 0, 0); intakeMotor.setPower(0); if (!ENABLE_FLYWHEEL_IDLE) { setFlywheelPower(0); } }
    private void setDriveBehavior(DcMotor.ZeroPowerBehavior behavior) { frontLeft.setZeroPowerBehavior(behavior); frontRight.setZeroPowerBehavior(behavior); backLeft.setZeroPowerBehavior(behavior); backRight.setZeroPowerBehavior(behavior); }
    private double applyDeadzone(double value) { return Math.abs(value) < DRIVE_DEADZONE ? 0.0 : value; }
    private boolean hasManualInput() { return Math.abs(gamepad1.left_stick_y) > DRIVE_DEADZONE || Math.abs(gamepad1.left_stick_x) > DRIVE_DEADZONE || Math.abs(gamepad1.right_stick_x) > DRIVE_DEADZONE; }
    
    private void updateTelemetry() {
        telemetry.addData("PATTERN", String.format("[%s] [%s] [%s]", currentPattern[0], currentPattern[1], currentPattern[2]));
        telemetry.addData("Mode", currentMode);
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());
        telemetry.addData("Balls Launched", ballsLaunched);
        telemetry.addData("Launch Zone", inLaunchZone ? "LEGAL" : "ILLEGAL");
        if (targetTag != null) {
            Orientation rot = Orientation.getOrientation(targetTag.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
            telemetry.addData("Tag", "%d | Yaw: %.1f | Dist: %.2f", targetTag.id, rot.firstAngle, targetTag.pose.z);
        } else { telemetry.addLine("No Target Detected"); }
        telemetry.update();
    }
    
    private void cleanup() { if (camera != null) { camera.stopStreaming(); } stopAllMotors(); }
}
