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
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

@Autonomous(name = "Auto_ShootWhileMoving_RR", group = "AUTO")
public class Auto_ShootWhileMoving_RR extends LinearOpMode {

    private DcMotorEx leftFrontEx, rightFrontEx, leftBackEx, rightBackEx;
    private Servo servo1 = null;
    private Servo servo2 = null;
    private DcMotorEx flywheel1 = null;
    private DcMotorEx flywheel2 = null;
    private DcMotor Intake = null;
    private SampleMecanumDrive drive;
    private OpenCvCamera camTags;
    private OpenCvCamera camBalls;
    private AprilTagDetectionPipeline tagPipeline;
    private BallDetectionPipeline ballPipeline;

    private static final double FX = 578.272, FY = 578.272, CX = 402.145, CY = 221.506;
    private static final double TAG_SIZE_METERS = 0.166;

    private final Map<Integer, double[]> FIELD_TAG_POSES = new HashMap<Integer, double[]>() {{
        put(20, new double[]{24.0, 24.0, Math.toRadians(0)});
        put(21, new double[]{48.0, 24.0, Math.toRadians(0)});
        put(24, new double[]{72.0, 24.0, Math.toRadians(0)});
    }};
    private AprilTagDetection lastTagSeen = null;
    private enum Alliance { BLUE, RED, UNKNOWN }
    private Alliance alliance = Alliance.UNKNOWN;

    private static final double TICKS_PER_REV = 28.0;
    private static final double BASE_FLY_RPM = 6000.0;
    private static final double MAX_FLY_RPM  = 6000.0;
    private static final int BALL_CAM_WIDTH = 640;
    private static final double BALL_APPROACH_SPEED = 0.36;

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime candidateTimer = new ElapsedTime();
    private final ElapsedTime shooterTimer = new ElapsedTime();

    private enum State { SEARCHING, APPROACHING, INTAKING, PREPARE_SHOOT, SHOOTING, TRANSIT }
    private State state = State.SEARCHING;

    private double currentTargetRPM = BASE_FLY_RPM;
    private final int intakeSide = -1;
    private double bestCandidateY = -1.0;
    private double bestCandidateX = 0.0;
    private final double CANDIDATE_WINDOW_SEC = 0.18;
    private int shooterPhase = 0;
    private final double SHOOTER_SPINUP_SEC = 0.35;
    private final long FEED_MS = 220;
    private final double POST_FEED_SEC = 0.18;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        drive = new SampleMecanumDrive(hardwareMap, new Pose2d(0,0,0));
        drive.setPoseEstimate(new Pose2d(0,0,0));
        initCameras();
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> tags = tagPipeline.getLatestDetections();
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
            if (ballPipeline != null && ballPipeline.isBallFound()) telemetry.addData("Ball", "xerr=%.1f y=%.1f", ballPipeline.getBallXError(), ballPipeline.getBallY());
            else telemetry.addLine("Ball: none");
            if (lastTagSeen != null) telemetry.addData("Tag", "id=%d z=%.2fm", lastTagSeen.id, lastTagSeen.pose.z);
            else telemetry.addLine("No tag seen yet");
            telemetry.update();
            sleep(50);
        }
        runtime.reset();
        if (lastTagSeen != null) {
            double[] pose = computeRobotPoseFromTag(lastTagSeen);
            if (pose != null) drive.setPoseEstimate(new Pose2d(pose[0], pose[1], pose[2]));
        }
        setFlywheelRPM(currentTargetRPM);
        while (opModeIsActive() && runtime.seconds() < 29.5) {
            drive.update();
            ArrayList<AprilTagDetection> tagsNow = tagPipeline.getLatestDetections();
            if (tagsNow != null && tagsNow.size() > 0) {
                for (AprilTagDetection t : tagsNow) {
                    if (FIELD_TAG_POSES.containsKey(t.id)) {
                        double[] poseFromTag = computeRobotPoseFromTag(t);
                        if (poseFromTag != null) {
                            drive.setPoseEstimate(new Pose2d(poseFromTag[0], poseFromTag[1], poseFromTag[2]));
                            lastTagSeen = t;
                            if (t.id == 20) alliance = Alliance.BLUE;
                            else if (t.id == 24) alliance = Alliance.RED;
                            break;
                        }
                    }
                }
            }

            switch (state) {
                case SEARCHING:
                    setFlywheelRPM(currentTargetRPM);
                    if (ballPipeline != null && ballPipeline.isBallFound()) {
                        double bx = ballPipeline.getBallX();
                        double by = ballPipeline.getBallY();
                        if (bestCandidateY < 0) {
                            bestCandidateY = by; bestCandidateX = bx; candidateTimer.reset();
                        } else {
                            if (by > bestCandidateY) { bestCandidateY = by; bestCandidateX = bx; candidateTimer.reset(); }
                        }
                        if (candidateTimer.seconds() > CANDIDATE_WINDOW_SEC && bestCandidateY > 0) {
                            state = State.APPROACHING; loopTimer.reset();
                        } else {
                            drive.setWeightedDrivePower(new Pose2d(0,0,0.20));
                        }
                    } else {
                        if (candidateTimer.seconds() > 0.4) { bestCandidateY = -1.0; bestCandidateX = 0.0; }
                        drive.setWeightedDrivePower(new Pose2d(0,0,0.28));
                    }
                    break;

                case APPROACHING:
                    if (ballPipeline == null || !ballPipeline.isBallFound()) { bestCandidateY = -1.0; bestCandidateX = 0.0; state = State.SEARCHING; break; }
                    double xErr = ballPipeline.getBallXError();
                    double y = ballPipeline.getBallY();
                    double turnCmd = Range.clip(-xErr / 300.0, -0.45, 0.45);
                    double forwardCmd = Range.clip((BALL_CAM_WIDTH - y) / (BALL_CAM_WIDTH * 2.0), 0, BALL_APPROACH_SPEED);
                    if (intakeSide < 0) forwardCmd = -forwardCmd;
                    drive.setWeightedDrivePower(new Pose2d(forwardCmd, 0, turnCmd));
                    if (y > BALL_CAM_WIDTH * 0.72) { state = State.INTAKING; loopTimer.reset(); }
                    break;

                case INTAKING:
                    Intake.setPower(1.0 * (intakeSide >= 0 ? 1.0 : -1.0));
                    setFlywheelRPM(currentTargetRPM);
                    if (!ballPipeline.isBallFound()) {
                        if (loopTimer.seconds() > 0.35) { Intake.setPower(0.0); state = State.PREPARE_SHOOT; shooterTimer.reset(); shooterPhase = 0; }
                    } else {
                        if (ballPipeline.getBallY() > BALL_CAM_WIDTH * 0.86 || loopTimer.seconds() > 1.2) { Intake.setPower(0.0); state = State.PREPARE_SHOOT; shooterTimer.reset(); shooterPhase = 0; }
                    }
                    break;

                case PREPARE_SHOOT:
                    setFlywheelRPM(currentTargetRPM);
                    if (shooterTimer.seconds() >= SHOOTER_SPINUP_SEC) { shooterTimer.reset(); shooterPhase = 1; state = State.SHOOTING; }
                    else drive.setWeightedDrivePower(new Pose2d(0.06 * (intakeSide < 0 ? -1 : 1), 0, 0));
                    break;

                case SHOOTING:
                    setFlywheelRPM(currentTargetRPM);
                    if (shooterPhase == 1) {
                        servo1.setPosition(0.95);
                        sleep(FEED_MS);
                        servo1.setPosition(0.0);
                        shooterTimer.reset();
                        shooterPhase = 2;
                    } else if (shooterPhase == 2) {
                        if (shooterTimer.seconds() > POST_FEED_SEC) { shooterPhase = 0; state = State.TRANSIT; loopTimer.reset(); }
                    } else {
                        shooterPhase = 1;
                    }
                    drive.setWeightedDrivePower(new Pose2d(0.05 * (intakeSide < 0 ? -1 : 1), 0, 0));
                    break;

                case TRANSIT:
                    if (!drive.isBusy()) {
                        Pose2d start = drive.getPoseEstimate();
                        double forward = 8.0 * (intakeSide < 0 ? -1 : 1);
                        drive.followTrajectoryAsync(drive.trajectoryBuilder(start).forward(forward).build());
                    } else drive.update();
                    if (loopTimer.seconds() > 0.65 && !drive.isBusy()) { state = State.SEARCHING; bestCandidateY = -1.0; }
                    break;
            }

            drive.update();
            publishTelemetry();
            maintainFlywheelVelocity();
            idle();
        }

        stopAll();
        telemetry.addLine("Autonomous finished.");
        telemetry.update();
        sleep(300);
    }

    private void initHardware() {
        leftFrontEx = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFrontEx = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBackEx = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBackEx = hardwareMap.get(DcMotorEx.class, "rightBack");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        leftFrontEx.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackEx.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontEx.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackEx.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servo1.setPosition(0.0);
        servo2.setPosition(0.0);
    }

    private void initCameras() {
        int camViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id", hardwareMap.appContext.getPackageName());
        camTags = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), camViewId);
        tagPipeline = new AprilTagDetectionPipeline(TAG_SIZE_METERS, FX, FY, CX, CY);
        camTags.setPipeline(tagPipeline);
        camTags.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() { camTags.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT); }
            @Override public void onError(int errorCode) {}
        });

        camBalls = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 2"), camViewId);
        ballPipeline = new BallDetectionPipeline(BALL_CAM_WIDTH);
        camBalls.setPipeline(ballPipeline);
        camBalls.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() { camBalls.startStreaming(BALL_CAM_WIDTH, BALL_CAM_WIDTH * 9 / 16, OpenCvCameraRotation.UPRIGHT); }
            @Override public void onError(int errorCode) {}
        });
    }

    private double rpmToTicksPerSec(double rpm) { return (rpm / 60.0) * TICKS_PER_REV; }
    private void setFlywheelRPM(double rpm) {
        rpm = Range.clip(rpm, 0, MAX_FLY_RPM);
        double ticksPerSec = rpmToTicksPerSec(rpm);
        flywheel1.setVelocity(ticksPerSec);
        flywheel2.setVelocity(ticksPerSec);
    }
    private void maintainFlywheelVelocity() { setFlywheelRPM(currentTargetRPM); }

    private void pulseIndexerOnce() {
        servo1.setPosition(0.95);
        sleep(FEED_MS);
        servo1.setPosition(0.0);
        sleep(120);
    }

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
        telemetry.addData("BallFound", ballPipeline != null && ballPipeline.isBallFound());
        if (ballPipeline != null && ballPipeline.isBallFound()) telemetry.addData("BallXY", "%.1f, %.1f err=%.1f", ballPipeline.getBallX(), ballPipeline.getBallY(), ballPipeline.getBallXError());
        if (lastTagSeen != null) telemetry.addData("TagLastSeen", lastTagSeen.id);
        telemetry.update();
    }

    private void stopAll() {
        Intake.setPower(0.0);
        flywheel1.setPower(0.0);
        flywheel2.setPower(0.0);
        drive.setWeightedDrivePower(new Pose2d(0,0,0));
        if (camBalls != null) camBalls.stopStreaming();
        if (camTags != null) camTags.stopStreaming();
    }
}
