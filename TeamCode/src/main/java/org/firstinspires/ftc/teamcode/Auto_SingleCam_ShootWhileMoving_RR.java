package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Mat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

@Autonomous(name = "Auto_SingleCam_ShootWhileMoving_RR", group = "AUTO")
public class Auto_SingleCam_ShootWhileMoving_RR extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftRear, rightRear;
    private Servo servo1 = null;
    private Servo servo2 = null;
    private DcMotorEx flywheel1 = null;
    private DcMotorEx flywheel2 = null;
    private DcMotor Intake = null;
    private SampleMecanumDrive drive;
    private OpenCvCamera camera;
    private CombinedDetectionPipeline combinedPipeline; // combined pipeline
    private AprilTagDetection lastTagSeen = null;

    // Camera watchdog flag: indicates whether streaming is currently active
    private volatile boolean cameraStreamingActive = false;

    private static final double FX = 578.272, FY = 578.272, CX = 402.145, CY = 221.506;
    private static final double TAG_SIZE_METERS = 0.166;

    private final Map<Integer, double[]> FIELD_TAG_POSES = new HashMap<Integer, double[]>() {{
        put(20, new double[]{24.0, 24.0, Math.toRadians(0)});
        put(21, new double[]{48.0, 24.0, Math.toRadians(0)});
        put(24, new double[]{72.0, 24.0, Math.toRadians(0)});
    }};
    private enum Alliance { BLUE, RED, UNKNOWN }
    private Alliance alliance = Alliance.UNKNOWN;

    private static final double TICKS_PER_REV = 28.0;
    private static final double BASE_FLY_RPM = 4200.0;
    private static final double MAX_FLY_RPM  = 6000.0;
    private static final int BALL_CAM_WIDTH = 640;
    private static final double BALL_APPROACH_SPEED = 0.5;

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime candidateTimer = new ElapsedTime();
    private final ElapsedTime shooterTimer = new ElapsedTime();

    // new timer for how long vision has been lost while approaching
    private final ElapsedTime ballLostTimer = new ElapsedTime();

    private enum State { SEARCHING, APPROACHING, INTAKING, PREPARE_SHOOT, SHOOTING, TRANSIT }
    private State state = State.SEARCHING;

    private double currentTargetRPM = BASE_FLY_RPM;
    private final int intakeSide = -1; // camera and intake opposite: negative flips forward
    private double bestCandidateY = -1.0;
    private double bestCandidateX = 0.0;
    private final double CANDIDATE_WINDOW_SEC = 0.18;
    private int shooterPhase = 0;
    private final double SHOOTER_SPINUP_SEC = 0.35;
    private final long FEED_MS = 220;
    private final double POST_FEED_SEC = 0.18;

    // persisted ball target (last-known)
    private volatile boolean hasBallTarget = false;
    private volatile double lastKnownBallX = 0.0; // pixel center x
    private volatile double lastKnownBallY = 0.0; // pixel center y

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        drive = new SampleMecanumDrive(hardwareMap, new Pose2d(0,0,0));
        drive.setPoseEstimate(new Pose2d(0,0,0));

        // Start camera immediately during INIT and ensure it persists via a watchdog
        initSingleCameraPersistent();

        // --- INIT LOOP (camera is started in initSingleCameraPersistent)
        while (!isStarted() && !isStopRequested()) {
            // show camera status and detections while waiting for START
            telemetry.addData("Camera streaming", cameraStreamingActive ? "YES" : "NO");
            ArrayList<AprilTagDetection> tags = combinedPipeline != null ? combinedPipeline.getLatestTags() : null;
            if (tags != null && tags.size() > 0) {
                for (AprilTagDetection t : tags) {
                    if (FIELD_TAG_POSES.containsKey(t.id)) {
                        lastTagSeen = t;
                        if (t.id == 20) alliance = Alliance.BLUE;
                        else if (t.id == 24) alliance = Alliance.RED;
                        else alliance = Alliance.UNKNOWN;
                        break;
                    }
                }
            }

            if (combinedPipeline != null && combinedPipeline.isBallFound())
                telemetry.addData("Ball", "xErr=%.1f y=%.1f", combinedPipeline.getBallXError(), combinedPipeline.getBallY());
            else telemetry.addLine("Ball: none");

            if (lastTagSeen != null)
                telemetry.addData("Tag", "id=%d z=%.2fm", lastTagSeen.id, lastTagSeen.pose.z);
            else telemetry.addLine("No tag seen yet");

            telemetry.update();
            // small sleep to reduce busy-wait load and allow camera thread to run
            sleep(50);
        }

        runtime.reset();
        if (lastTagSeen != null) {
            double[] pose = computeRobotPoseFromTag(lastTagSeen);
            if (pose != null) drive.setPoseEstimate(new Pose2d(pose[0], pose[1], pose[2]));
        }

        setFlywheelRPM(currentTargetRPM);

        // --- MAIN LOOP
        while (opModeIsActive() && runtime.seconds() < 29.5) {
            drive.update();
            ArrayList<AprilTagDetection> tags = combinedPipeline != null ? combinedPipeline.getLatestTags() : null;
            if (tags != null && tags.size() > 0) lastTagSeen = tags.get(0);

            switch (state) {
                case SEARCHING:
                    setFlywheelRPM(currentTargetRPM);

                    // If pipeline says ball present, collect candidates for a short stable window
                    if (combinedPipeline != null && combinedPipeline.isBallFound()) {
                        double bx = combinedPipeline.getBallX();
                        double by = combinedPipeline.getBallY();

                        if (bestCandidateY < 0) {
                            bestCandidateY = by;
                            bestCandidateX = bx;
                            candidateTimer.reset();
                        } else if (by > bestCandidateY) {
                            bestCandidateY = by;
                            bestCandidateX = bx;
                            candidateTimer.reset();
                        }

                        // after a short stable window, commit to approach and store target
                        if (candidateTimer.seconds() > CANDIDATE_WINDOW_SEC && bestCandidateY > 0) {
                            // lock target
                            hasBallTarget = true;
                            lastKnownBallX = bestCandidateX;
                            lastKnownBallY = bestCandidateY;
                            ballLostTimer.reset();
                            // turn on intake and keep it on for the remainder of the run
                            Intake.setPower(1.0 * (intakeSide >= 0 ? 1.0 : -1.0));
                            state = State.APPROACHING;
                            loopTimer.reset();
                        } else {
                            // stay still while we're waiting for a stable candidate (no rotation)
                            drive.setWeightedDrivePower(new Pose2d(0.0, 0.0, 0.0));
                        }
                    } else {
                        // no candidate: clear accumulation after a short time and remain stopped
                        if (candidateTimer.seconds() > 0.4) {
                            bestCandidateY = -1.0;
                            bestCandidateX = 0.0;
                        }
                        // keep robot still while searching (camera doing detection)
                        drive.setWeightedDrivePower(new Pose2d(0.0, 0.0, 0.0));
                    }
                    break;

                case APPROACHING:
                    // If we've already locked a target, keep going to that position even if ball is momentarily lost
                    if (hasBallTarget) {
                        // update last-known when vision returns
                        boolean live = (combinedPipeline != null && combinedPipeline.isBallFound());
                        if (live) {
                            lastKnownBallX = combinedPipeline.getBallX();
                            lastKnownBallY = combinedPipeline.getBallY();
                            ballLostTimer.reset();
                        }

                        // Compute horizontal angle offset (radians) using pinhole model:
                        // angle ≈ atan((pixelX - CX) / FX)
                        double pixelX = lastKnownBallX;
                        double pixelY = lastKnownBallY;
                        double angleOffset = Math.atan((pixelX - CX) / FX); // camera frame angle to ball

                        // If camera is mounted on opposite side physically you may need to invert sign;
                        // we keep your existing intakeSide handling only for forward direction.

                        // TURN in place first using RoadRunner, then drive straight forward using a single trajectory.
                        // Only initiate a turn/trajectory when the drive is not busy.
                        if (!drive.isBusy()) {
                            // 1) Turn to face the ball (angleOffset)
                            drive.turnAsync(angleOffset);
                            // wait for the turn to complete while maintaining shooter spin
                            while (opModeIsActive() && drive.isBusy()) {
                                maintainFlywheelVelocity();
                                drive.update();
                                idle();
                            }

                            // 2) After turning, plan a straight trajectory forward of an estimated distance.
                            // Estimate distance (inches) from the ball's vertical pixel position.
                            // This is a heuristic: when the ball is lower (larger pixelY) it's closer.
                            double frac = 1.0 - (pixelY / (double) BALL_CAM_WIDTH); // 0..1
                            frac = Range.clip(frac, 0.05, 1.0);
                            // scale to an approximate forward distance (inches). Tunable.
                            double forwardInches = Range.clip(frac * 36.0, 6.0, 48.0);
                            // flip sign if intake is on the opposite side (preserves your existing convention)
                            if (intakeSide < 0) forwardInches = -forwardInches;

                            // build and follow a single straight trajectory toward the estimated location
                            Pose2d start = drive.getPoseEstimate();
                            drive.followTrajectoryAsync(drive.trajectoryBuilder(start).forward(forwardInches).build());

                            // while driving the trajectory, keep updating and allow vision to update lastKnown (but do not interrupt)
                            while (opModeIsActive() && drive.isBusy()) {
                                // refresh last-known if live vision returns
                                if (combinedPipeline != null && combinedPipeline.isBallFound()) {
                                    lastKnownBallX = combinedPipeline.getBallX();
                                    lastKnownBallY = combinedPipeline.getBallY();
                                    ballLostTimer.reset();
                                }
                                maintainFlywheelVelocity();
                                drive.update();
                                idle();
                            }

                            // after finishing the straight trajectory, treat as we are in intake zone
                            state = State.INTAKING;
                            loopTimer.reset();
                        } // if drive busy: let it finish (we keep the locked target in memory)
                    } else {
                        // If we lost our stored target entirely, go back to searching
                        state = State.SEARCHING;
                    }
                    break;


                case INTAKING:
                    // Intake is already turned ON when we locked the target; keep it running here (forever until stopAll)
                    Intake.setPower(1.0 * (intakeSide >= 0 ? 1.0 : -1.0));
                    setFlywheelRPM(currentTargetRPM);

                    // We will assume that after we've driven into the ball (close pixel Y) and a short dwell, the ball is collected.
                    // Keep intake on; after a short time proceed to shooting preparation.
                    if (loopTimer.seconds() > 0.5 || (combinedPipeline != null && combinedPipeline.getBallY() > BALL_CAM_WIDTH * 0.86)) {
                        // do NOT stop intake (user asked "forever"); proceed to prepare shoot
                        // mark we no longer need the locked ball target for approach
                        hasBallTarget = false;
                        bestCandidateY = -1.0;
                        bestCandidateX = 0.0;
                        shooterTimer.reset();
                        shooterPhase = 0;
                        state = State.PREPARE_SHOOT;
                    }
                    break;

                case PREPARE_SHOOT:
                    // Before shooting, align with the last-seen AprilTag (if any)
                    setFlywheelRPM(currentTargetRPM);

                    if (lastTagSeen != null) {
                        // compute a desired heading to face the tag using current estimated robot pose
                        double[] tagPose = FIELD_TAG_POSES.getOrDefault(lastTagSeen.id, null);
                        if (tagPose != null) {
                            Pose2d robotPose = drive.getPoseEstimate();
                            double robotX = robotPose.getX();
                            double robotY = robotPose.getY();
                            double robotHeading = robotPose.getHeading();

                            double tagX = tagPose[0];
                            double tagY = tagPose[1];
                            double desiredHeading = Math.atan2(tagY - robotY, tagX - robotX);

                            double angleDiff = wrapAngle(desiredHeading - robotHeading);

                            // Use RoadRunner to turn toward the tag
                            drive.turnAsync(angleDiff);
                            // wait for turn to finish while maintaining flywheel spin
                            while (opModeIsActive() && drive.isBusy()) {
                                maintainFlywheelVelocity();
                                drive.update();
                                idle();
                            }
                        }
                    }

                    // After aligning, spin up shooter for the configured spinup time then go to SHOOTING
                    if (shooterTimer.seconds() >= SHOOTER_SPINUP_SEC) {
                        shooterTimer.reset();
                        shooterPhase = 1;
                        state = State.SHOOTING;
                    } else {
                        // small nudge forward if you want to tuck into shooting position
                        drive.setWeightedDrivePower(new Pose2d(0.04 * (intakeSide < 0 ? -1 : 1), 0, 0));
                    }
                    break;

                case SHOOTING:
                    setFlywheelRPM(currentTargetRPM);
                    if (shooterPhase == 1) {
                        // actuate feeder servos while intake remains on
                        servo1.setPosition(0.95);
                        servo2.setPosition(0.95);
                        sleep(FEED_MS);
                        servo1.setPosition(0.0);
                        servo2.setPosition(0.0);
                        shooterTimer.reset();
                        shooterPhase = 2;
                    } else if (shooterPhase == 2) {
                        if (shooterTimer.seconds() > POST_FEED_SEC) {
                            shooterPhase = 0;
                            state = State.TRANSIT;
                            loopTimer.reset();
                        }
                    } else shooterPhase = 1;

                    drive.setWeightedDrivePower(new Pose2d(0.02 * (intakeSide < 0 ? -1 : 1), 0, 0));
                    break;

                case TRANSIT:
                    // small transit move using RoadRunner, then go back to SEARCHING
                    if (!drive.isBusy()) {
                        Pose2d start = drive.getPoseEstimate();
                        double forward = 8.0 * (intakeSide < 0 ? -1 : 1);
                        drive.followTrajectoryAsync(drive.trajectoryBuilder(start).forward(forward).build());
                    } else {
                        drive.update();
                    }
                    if (loopTimer.seconds() > 0.65 && !drive.isBusy()) {
                        state = State.SEARCHING;
                        bestCandidateY = -1.0;
                        hasBallTarget = false;
                    }
                    break;
            }

            publishTelemetry();
            maintainFlywheelVelocity();
            idle();
        }

        stopAll();
    }

    // helper: wrap angle to [-PI, PI]
    private double wrapAngle(double ang) {
        while (ang <= -Math.PI) ang += 2.0 * Math.PI;
        while (ang > Math.PI) ang -= 2.0 * Math.PI;
        return ang;
    }

    // --- IMPORTANT: persistent camera setup with watchdog retries
    private void initSingleCameraPersistent() {
        final int camViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId","id", hardwareMap.appContext.getPackageName());

        // create camera instance
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class,"Webcam 1"), camViewId);

        // prepare combined pipeline
        combinedPipeline = new CombinedDetectionPipeline(TAG_SIZE_METERS, FX, FY, CX, CY, BALL_CAM_WIDTH);
        // set pipeline BEFORE opening camera
        camera.setPipeline(combinedPipeline);

        // attempt to open camera (async) and start streaming
        openAndStartCameraAsync();

        // start a background watcher thread that ensures camera stays streaming (with delay to avoid busy retries)
        Thread watcher = new Thread(() -> {
            while (!isStopRequested() && !isStarted()) {
                try {
                    // If camera isn't streaming, attempt to (re)open it, but rate-limit retries
                    if (!cameraStreamingActive) {
                        telemetry.addData("CameraWatch", "Camera not streaming — attempting reopen");
                        telemetry.update();
                        openAndStartCameraAsync();
                    }
                    // sleep to avoid tight loop and to give the camera driver time to stabilize
                    Thread.sleep(500);
                } catch (InterruptedException ie) {
                    // ignore interruption, keep loop running until opMode stops
                } catch (Exception e) {
                    telemetry.addData("CameraWatchError", e.getMessage());
                    telemetry.update();
                    try { Thread.sleep(500); } catch (InterruptedException ignored) {}
                }
            }

            // Once opmode started, keep checking until stop requested (less frequently)
            while (!isStopRequested()) {
                try {
                    if (!cameraStreamingActive) {
                        telemetry.addData("CameraWatch", "Camera streaming lost — attempting reopen");
                        telemetry.update();
                        openAndStartCameraAsync();
                    }
                    Thread.sleep(1000);
                } catch (InterruptedException ignored) {}
            }
        });
        watcher.setDaemon(true);
        watcher.start();
    }

    // Helper that calls openCameraDeviceAsync and sets cameraStreamingActive appropriately
    private void openAndStartCameraAsync() {
        if (camera == null) return;
        try {
            // avoid multiple simultaneous open attempts
            // mark false now and only set true on successful start
            cameraStreamingActive = false;
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    try {
                        // ensure pipeline is set (idempotent)
                        try { camera.setPipeline(combinedPipeline); } catch (Exception ignored) {}
                        camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                        cameraStreamingActive = true;
                        telemetry.addData("Camera", "Opened & streaming");
                        telemetry.update();
                    } catch (Exception e) {
                        cameraStreamingActive = false;
                        telemetry.addData("Camera start error", e.getMessage());
                        telemetry.update();
                    }
                }

                @Override
                public void onError(int errorCode) {
                    cameraStreamingActive = false;
                    telemetry.addData("Camera open error", errorCode);
                    telemetry.update();
                }
            });
        } catch (Exception e) {
            cameraStreamingActive = false;
            telemetry.addData("Camera async open exception", e.getMessage());
            telemetry.update();
        }
    }

    // Combined pipeline class: runs both AprilTag and Ball detection
    public static class CombinedDetectionPipeline extends org.openftc.easyopencv.OpenCvPipeline {
        private final AprilTagDetectionPipeline tagPipeline;
        private final BallDetectionPipeline ballPipeline;

        public CombinedDetectionPipeline(double tagSize, double fx, double fy, double cx, double cy, int ballWidth) {
            tagPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);
            ballPipeline = new BallDetectionPipeline(ballWidth);
        }

        @Override
        public Mat processFrame(Mat input) {
            // run both pipelines on same frame
            tagPipeline.processFrame(input);
            ballPipeline.processFrame(input);
            return input;
        }

        public ArrayList<AprilTagDetection> getLatestTags() { return tagPipeline.getLatestDetections(); }
        public boolean isBallFound() { return ballPipeline.isBallFound(); }
        public double getBallX() { return ballPipeline.getBallX(); }
        public double getBallY() { return ballPipeline.getBallY(); }
        public double getBallXError() { return ballPipeline.getBallXError(); }
    }

    // Remaining helper functions (hardware, pose, telemetry, etc.)
    private void initHardware() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servo1.setPosition(0.0);
        servo2.setPosition(0.0);
    }

    private double rpmToTicksPerSec(double rpm) { return (rpm / 60.0) * TICKS_PER_REV; }
    private void setFlywheelRPM(double rpm) {
        rpm = Range.clip(rpm, 0, MAX_FLY_RPM);
        double ticksPerSec = rpmToTicksPerSec(rpm);
        flywheel1.setVelocity(ticksPerSec);
        flywheel2.setVelocity(ticksPerSec);
    }
    private void maintainFlywheelVelocity() { setFlywheelRPM(currentTargetRPM); }

    private double[] computeRobotPoseFromTag(AprilTagDetection detection) {
        if (detection == null) return null;
        int id = detection.id;
        double[] tagPose = FIELD_TAG_POSES.get(id);
        if (tagPose == null) return null;
        double tagX = tagPose[0], tagY = tagPose[1], tagHeading = tagPose[2];
        double relX_m = detection.pose.x, relZ_m = detection.pose.z;
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        double relYaw = Math.toRadians(rot.firstAngle);
        double relX = relX_m * 39.3701, relZ = relZ_m * 39.3701;
        double robotX = tagX - relZ * Math.cos(tagHeading) + relX * Math.sin(tagHeading);
        double robotY = tagY - relZ * Math.sin(tagHeading) - relX * Math.cos(tagHeading);
        double robotHeading = tagHeading - relYaw;
        return new double[]{robotX, robotY, robotHeading};
    }

    private void publishTelemetry() {
        Pose2d p = drive.getPoseEstimate();
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1fdeg", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.addData("State", state.toString());
        telemetry.addData("RPM target", currentTargetRPM);
        telemetry.addData("Alliance", alliance.toString());
        telemetry.addData("BallFound (live)", combinedPipeline != null && combinedPipeline.isBallFound());
        telemetry.addData("HasLockedTarget", hasBallTarget);
        telemetry.addData("LastKnownBallXY", "%.1f, %.1f", lastKnownBallX, lastKnownBallY);
        if (combinedPipeline != null && combinedPipeline.isBallFound())
            telemetry.addData("BallXY (live)", "%.1f, %.1f err=%.1f", combinedPipeline.getBallX(), combinedPipeline.getBallY(), combinedPipeline.getBallXError());
        telemetry.addData("Camera streaming", cameraStreamingActive ? "YES" : "NO");
        telemetry.update();
    }

    private void stopAll() {
        // stop motors/servos
        Intake.setPower(0.0);
        if (flywheel1 != null) flywheel1.setPower(0.0);
        if (flywheel2 != null) flywheel2.setPower(0.0);
        drive.setWeightedDrivePower(new Pose2d(0,0,0));

        // stop camera streaming and close device cleanly
        cameraStreamingActive = false;
        if (camera != null) {
            try {
                camera.stopStreaming();
            } catch (Exception ignored) {}
            try {
                camera.closeCameraDevice();
            } catch (Exception ignored) {}
        }
    }

}
