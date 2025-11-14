package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Mat;

import org.openftc.apriltag.AprilTagDetection;
import java.util.ArrayList;

@TeleOp(name = "V3", group = "Competition")
public class TeleOpV3 extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeMotor;
    private DcMotorEx launcherLeft, launcherRight;
    private Servo liftLeft, liftRight;

    private static final double HIGH_RPM = 3150;
    private static final double LOW_RPM = 2500;
    private double targetRPM = 0;

    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_DRIVE_RPM = 6000;
    private static final double DRIVE_TICKS_PER_SEC = (MAX_DRIVE_RPM / 60.0) * TICKS_PER_REV;

    private boolean intakeToggle = false;
    private boolean lastIntakeToggle = false;
    private boolean flapUp = false;
    private boolean lastFlapToggle = false;
    private long flapTimer = 0;
    private final double FLAP_UP_POS = 0.5;
    private final double FLAP_DOWN_POS = 0.0;

    // ====== AprilTag fields ======
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagPipeline;
    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagSize = 0.166; // meters

    @Override
    public void runOpMode() {
        initHardware();
        initCamera();

        waitForStart();

        while (opModeIsActive()) {
            mecanumDrive();
            controlIntake();
            controlFlaps();
            updateLauncherVelocity();
            updateTelemetry();
            updateAprilTagTelemetry();
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
    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        aprilTagPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming once the camera is opened
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Print any errors to telemetry
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
    }

    /** 🚗 Perfect mecanum drive with normalized diagonal/strafe **/
    private void mecanumDrive() {
        double y = -gamepad1.left_stick_y; // forward/back
        double x = gamepad1.left_stick_x;  // strafe
        double rx = -gamepad1.right_stick_x; // rotation

        double DEADZONE = 0.05;
        if (Math.abs(y) < DEADZONE) y = 0;
        if (Math.abs(x) < DEADZONE) x = 0;
        if (Math.abs(rx) < DEADZONE) rx = 0;

        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeft.setVelocity(fl * DRIVE_TICKS_PER_SEC);
        frontRight.setVelocity(fr * DRIVE_TICKS_PER_SEC);
        backLeft.setVelocity(bl * DRIVE_TICKS_PER_SEC);
        backRight.setVelocity(br * DRIVE_TICKS_PER_SEC);
    }

    private void updateLauncherVelocity() {
        if (gamepad1.right_trigger > 0.5) targetRPM = HIGH_RPM;
        else if (gamepad1.left_trigger > 0.5) targetRPM = LOW_RPM;
        else targetRPM = 0;

        double velocityTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REV;
        launcherLeft.setVelocity(velocityTicksPerSec);
        launcherRight.setVelocity(velocityTicksPerSec);
    }

    private void controlIntake() {
        if (gamepad1.right_bumper && !lastIntakeToggle) intakeToggle = !intakeToggle;
        lastIntakeToggle = gamepad1.right_bumper;

        double intakePower = 0.0;
        if (gamepad1.a) intakePower = 1.0;
        else if (gamepad1.b) intakePower = -1.0;
        else if (intakeToggle) intakePower = 1.0;

        intakeMotor.setPower(intakePower);
    }

    private void controlFlaps() {
        if (gamepad1.left_bumper && !lastFlapToggle) {
            flapUp = !flapUp;
            if (flapUp) flapTimer = System.currentTimeMillis();
        }
        lastFlapToggle = gamepad1.left_bumper;

        if (flapUp && System.currentTimeMillis() - flapTimer > 300) flapUp = false;

        liftLeft.setPosition(flapUp ? FLAP_UP_POS : FLAP_DOWN_POS);
        liftRight.setPosition(flapUp ? 1 - FLAP_UP_POS : 1 - FLAP_DOWN_POS);
    }

    private void updateTelemetry() {
        telemetry.addData("Drive FL RPM", String.format("%.2f", frontLeft.getVelocity() / TICKS_PER_REV * 60.0));
        telemetry.addData("Drive FR RPM", String.format("%.2f", frontRight.getVelocity() / TICKS_PER_REV * 60.0));
        telemetry.addData("Drive BL RPM", String.format("%.2f", backLeft.getVelocity() / TICKS_PER_REV * 60.0));
        telemetry.addData("Drive BR RPM", String.format("%.2f", backRight.getVelocity() / TICKS_PER_REV * 60.0));
        telemetry.addData("Launcher Target RPM", targetRPM);
        telemetry.addData("Launcher Left RPM", String.format("%.2f", launcherLeft.getVelocity() / TICKS_PER_REV * 60.0));
        telemetry.addData("Launcher Right RPM", String.format("%.2f", launcherRight.getVelocity() / TICKS_PER_REV * 60.0));

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        String mode;
        if (Math.abs(y) > 0.1 && Math.abs(x) < 0.1 && Math.abs(rx) < 0.1) mode = "FORWARD/BACK";
        else if (Math.abs(x) > 0.1 && Math.abs(y) < 0.1 && Math.abs(rx) < 0.1) mode = "STRAFE";
        else if (Math.abs(rx) > 0.1 && Math.abs(x) < 0.1 && Math.abs(y) < 0.1) mode = "ROTATE";
        else mode = "DIAGONAL/COMBO";
        telemetry.addData("Movement Mode", mode);
    }

    // ====== AprilTag Telemetry ======
    private void updateAprilTagTelemetry() {
        ArrayList<AprilTagDetection> detections = aprilTagPipeline.getLatestDetections();
        if (detections.size() > 0) {
            for (AprilTagDetection tag : detections) {
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("X (ft)", String.format("%.2f", tag.pose.x * FEET_PER_METER));
                telemetry.addData("Y (ft)", String.format("%.2f", tag.pose.y * FEET_PER_METER));
                telemetry.addData("Z (ft)", String.format("%.2f", tag.pose.z * FEET_PER_METER));
            }
        } else {
            telemetry.addData("Tag", "None detected");
        }
        telemetry.update();
    }

    // ====== Inner AprilTag Pipeline Class ======
    public class AprilTagDetectionPipeline extends org.openftc.easyopencv.OpenCvPipeline {
        private double fx, fy, cx, cy, tagsize;
        private ArrayList<AprilTagDetection> detections = new ArrayList<>();

        public AprilTagDetectionPipeline(double tagsize, double fx, double fy, double cx, double cy) {
            this.tagsize = tagsize;
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
        }

        @Override
        public Mat processFrame(Mat input) {
            // Return the original frame; real detection happens via OpenFTC's API
            // In your real setup, you can call AprilTagDetectorJNI or FTC SDK AprilTag functions
            return input;
        }

        public ArrayList<AprilTagDetection> getLatestDetections() {
            // For now, just return an empty list (replace with actual detection logic)
            return detections;
        }
    }
}
