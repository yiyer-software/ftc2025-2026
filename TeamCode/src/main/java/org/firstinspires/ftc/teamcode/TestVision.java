package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@TeleOp(name = "TestVision Final", group = "Vision")
public class TestVision extends LinearOpMode {

    private static final String WEBCAM_NAME = "Vision";

    private OpenCvCamera camera;
    private CombinedPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init TestVision Final...");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName;
        try {
            webcamName = hardwareMap.get(WebcamName.class, WEBCAM_NAME);
        } catch (Exception e) {
            telemetry.addLine("Webcam '" + WEBCAM_NAME + "' not found in config.");
            telemetry.update();
            while (!isStopRequested()) sleep(500);
            return;
        }

        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new CombinedPipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                telemetry.addLine("Camera streaming (RC preview).");
                telemetry.update();
            }
            @Override public void onError(int errorCode) {
                telemetry.addData("CameraError", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        telemetry.clearAll();
        telemetry.addLine("Running. Center color + AprilTag (if available).");
        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) {
            // Color info
            String color = pipeline.getColorName();
            double[] hsv = pipeline.getColorHSV();
            if (hsv != null) {
                telemetry.addData("CenterColor", color);
                telemetry.addData("HSV", String.format("H:%.0f S:%.2f V:%.2f", hsv[0], hsv[1], hsv[2]));
            } else {
                telemetry.addData("CenterColor", "unknown");
            }

            // AprilTag info (if present)
            TagInfo tag = pipeline.getLatestTag();
            if (tag != null && tag.valid) {
                telemetry.addData("TagID", tag.id);
                telemetry.addData("tx_m", String.format("%.3f", tag.tx));
                telemetry.addData("ty_m", String.format("%.3f", tag.ty));
                telemetry.addData("tz_m", String.format("%.3f", tag.tz));
                telemetry.addData("yaw_deg", String.format("%.1f", tag.yawDeg));
                telemetry.addData("pitch_deg", String.format("%.1f", tag.pitchDeg));
                telemetry.addData("roll_deg", String.format("%.1f", tag.rollDeg));
            } else if (!pipeline.aprilAvailable()) {
                telemetry.addLine("AprilTag pipeline not found in build.");
            } else {
                telemetry.addData("AprilTag", "none detected");
            }

            telemetry.update();
            sleep(120);
        }

        try { camera.stopStreaming(); } catch (Exception ignored) {}
        try { camera.closeCameraDevice(); } catch (Exception ignored) {}
    }

    // ---------------- Combined pipeline: color + optional apriltag ----------------
    private static class CombinedPipeline extends OpenCvPipeline {
        private final ColorSampler colorSampler = new ColorSampler();
        private volatile TagInfo latestTag = new TagInfo();
        private volatile boolean aprilAvailable = false;

        // april pipeline instance kept as Object to avoid compile-time dependency
        private final Object aprilPipelineInstance;

        CombinedPipeline() {
            aprilPipelineInstance = makeAprilPipeline();
            aprilAvailable = aprilPipelineInstance != null;
        }

        @Override
        public Mat processFrame(Mat input) {
            if (input == null || input.empty()) return input;

            // color sampler draws overlay and updates values
            colorSampler.processFrameInternal(input);

            if (aprilPipelineInstance != null) {
                try {
                    // If the pipeline has a processFrame(Mat) method, call it (optional)
                    try {
                        java.lang.reflect.Method proc = aprilPipelineInstance.getClass().getMethod("processFrame", Mat.class);
                        proc.invoke(aprilPipelineInstance, input);
                    } catch (NoSuchMethodException ignored) {}

                    // try getDetectionsUpdate() then getDetections()
                    Object detsObj = null;
                    try {
                        java.lang.reflect.Method m = aprilPipelineInstance.getClass().getMethod("getDetectionsUpdate");
                        detsObj = m.invoke(aprilPipelineInstance);
                    } catch (NoSuchMethodException ex1) {
                        try {
                            java.lang.reflect.Method m2 = aprilPipelineInstance.getClass().getMethod("getDetections");
                            detsObj = m2.invoke(aprilPipelineInstance);
                        } catch (NoSuchMethodException ex2) {
                            detsObj = null;
                        }
                    }

                    if (detsObj instanceof List) {
                        @SuppressWarnings("unchecked")
                        List<Object> dets = (List<Object>) detsObj;
                        if (!dets.isEmpty()) {
                            Object det = dets.get(0);
                            TagInfo info = extractTagInfo(det);
                            if (info != null) latestTag = info;
                        } else {
                            latestTag = new TagInfo(); // none
                        }
                    }
                } catch (Exception ignored) {
                    // reflection error; keep color functionality working
                }
            }

            return input;
        }

        public String getColorName() { return colorSampler.getColorName(); }
        public double[] getColorHSV() { return colorSampler.getColorHSV(); }
        public TagInfo getLatestTag() { return latestTag; }
        public boolean aprilAvailable() { return aprilAvailable; }

        // try common apriltag pipeline class names and constructors
        private Object makeAprilPipeline() {
            String[] candidates = new String[] {
                    "org.openftc.apriltag.AprilTagDetectionPipeline",
                    "org.firstinspires.ftc.vision.apriltag.AprilTagPipeline",
                    "org.firstinspires.ftc.vision.apriltag.AprilTagProcessor",
                    "org.firstinspires.ftc.vision.apriltag.AprilTagDetectionPipeline"
            };
            for (String name : candidates) {
                try {
                    Class<?> cls = Class.forName(name);
                    // try common constructor (tagSizeMeters, fx, fy, cx, cy)
                    try {
                        java.lang.reflect.Constructor<?> ctor = cls.getConstructor(double.class,double.class,double.class,double.class,double.class);
                        Object inst = ctor.newInstance(0.0762, 600.0, 600.0, 320.0, 240.0);
                        return inst;
                    } catch (NoSuchMethodException e1) {
                        try {
                            // try no-arg constructor
                            Object inst = cls.getConstructor().newInstance();
                            return inst;
                        } catch (NoSuchMethodException | InstantiationException ignored) {}
                    }
                } catch (ClassNotFoundException ignored) {
                } catch (Exception ignored) {}
            }
            return null;
        }

        private TagInfo extractTagInfo(Object det) {
            if (det == null) return null;
            TagInfo out = new TagInfo();
            try {
                // id field or getId()
                try {
                    java.lang.reflect.Field fId = det.getClass().getField("id");
                    out.id = ((Number) fId.get(det)).intValue();
                } catch (NoSuchFieldException nsf) {
                    try {
                        java.lang.reflect.Method mId = det.getClass().getMethod("getId");
                        out.id = ((Number) mId.invoke(det)).intValue();
                    } catch (Exception ignored) {}
                }

                try {
                    java.lang.reflect.Field fPose = det.getClass().getField("pose");
                    Object poseObj = fPose.get(det);
                    if (poseObj != null) {
                        // translation: x,y,z fields or getTranslation()
                        try {
                            java.lang.reflect.Field fx = poseObj.getClass().getField("x");
                            java.lang.reflect.Field fy = poseObj.getClass().getField("y");
                            java.lang.reflect.Field fz = poseObj.getClass().getField("z");
                            out.tx = ((Number) fx.get(poseObj)).doubleValue();
                            out.ty = ((Number) fy.get(poseObj)).doubleValue();
                            out.tz = ((Number) fz.get(poseObj)).doubleValue();
                        } catch (NoSuchFieldException ex1) {
                            try {
                                java.lang.reflect.Method mTrans = poseObj.getClass().getMethod("getTranslation");
                                Object trans = mTrans.invoke(poseObj);
                                if (trans instanceof double[]) {
                                    double[] tt = (double[]) trans;
                                    if (tt.length >= 3) { out.tx = tt[0]; out.ty = tt[1]; out.tz = tt[2]; }
                                }
                            } catch (Exception ignored) {}
                        }
                        try {
                            java.lang.reflect.Field fyaw = poseObj.getClass().getField("yaw");
                            java.lang.reflect.Field fpitch = poseObj.getClass().getField("pitch");
                            java.lang.reflect.Field froll = poseObj.getClass().getField("roll");
                            out.yawDeg = Math.toDegrees(((Number) fyaw.get(poseObj)).doubleValue());
                            out.pitchDeg = Math.toDegrees(((Number) fpitch.get(poseObj)).doubleValue());
                            out.rollDeg = Math.toDegrees(((Number) froll.get(poseObj)).doubleValue());
                        } catch (NoSuchFieldException ex2) {
                            try {
                                java.lang.reflect.Method my = poseObj.getClass().getMethod("getYaw");
                                java.lang.reflect.Method mp = poseObj.getClass().getMethod("getPitch");
                                java.lang.reflect.Method mr = poseObj.getClass().getMethod("getRoll");
                                out.yawDeg = Math.toDegrees(((Number) my.invoke(poseObj)).doubleValue());
                                out.pitchDeg = Math.toDegrees(((Number) mp.invoke(poseObj)).doubleValue());
                                out.rollDeg = Math.toDegrees(((Number) mr.invoke(poseObj)).doubleValue());
                            } catch (Exception ignored) {}
                        }
                        out.valid = true;
                        return out;
                    }
                } catch (NoSuchFieldException ignored) {}

                try {
                    java.lang.reflect.Field ft = det.getClass().getField("pose_t");
                    Object poseT = ft.get(det);
                    if (poseT instanceof double[]) {
                        double[] pt = (double[]) poseT;
                        if (pt.length >= 3) { out.tx = pt[0]; out.ty = pt[1]; out.tz = pt[2]; out.valid = true; }
                    }
                } catch (NoSuchFieldException ignored) {}

                try {
                    java.lang.reflect.Method mx = det.getClass().getMethod("getPoseX");
                    java.lang.reflect.Method my = det.getClass().getMethod("getPoseY");
                    java.lang.reflect.Method mz = det.getClass().getMethod("getPoseZ");
                    out.tx = ((Number) mx.invoke(det)).doubleValue();
                    out.ty = ((Number) my.invoke(det)).doubleValue();
                    out.tz = ((Number) mz.invoke(det)).doubleValue();
                    out.valid = true;
                } catch (NoSuchMethodException ignored) {}

            } catch (Exception ignored) {}

            return out.valid ? out : null;
        }
    }

    private static class ColorSampler {
        private volatile String colorName = "unknown";
        private volatile double[] colorHSV = null;
        private final int SAMPLE_SIZE = 30;

        public Mat processFrameInternal(Mat input) {
            if (input == null || input.empty()) return input;

            int cx = input.cols() / 2;
            int cy = input.rows() / 2;
            int half = SAMPLE_SIZE / 2;
            int left = Math.max(0, cx - half);
            int top = Math.max(0, cy - half);
            int right = Math.min(input.cols(), cx + half);
            int bottom = Math.min(input.rows(), cy + half);
            Rect roi = new Rect(left, top, right - left, bottom - top);

            Mat region = new Mat(input, roi);
            Mat regionHsv = new Mat();
            Imgproc.cvtColor(region, regionHsv, Imgproc.COLOR_BGR2HSV);

            Scalar avg = Core.mean(regionHsv);
            double h = avg.val[0];
            double s = avg.val[1] / 255.0;
            double v = avg.val[2] / 255.0;
            colorHSV = new double[] { h, s, v };
            colorName = classifyHSV(h, s, v);

            Imgproc.rectangle(input, new Point(left, top), new Point(right, bottom), new Scalar(0, 255, 0), 2);
            Imgproc.putText(input, colorName, new Point(left, top - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);

            region.release();
            regionHsv.release();

            return input;
        }

        public String getColorName() { return colorName; }
        public double[] getColorHSV() { return colorHSV; }

        private String classifyHSV(double h, double s, double v) {
            if (v < 0.10) return "black";
            if (s < 0.15 && v > 0.85) return "white";
            if (s < 0.20) return "gray";
            double hueDeg = h * 2.0;
            if (hueDeg < 15 || hueDeg >= 345) return "red";
            if (hueDeg >= 15 && hueDeg < 45) return "orange";
            if (hueDeg >= 45 && hueDeg < 75) return "yellow";
            if (hueDeg >= 75 && hueDeg < 165) return "green";
            if (hueDeg >= 165 && hueDeg < 195) return "cyan";
            if (hueDeg >= 195 && hueDeg < 255) return "blue";
            if (hueDeg >= 255 && hueDeg < 285) return "magenta";
            if (hueDeg >= 285 && hueDeg < 345) return "pink";
            return "unknown";
        }
    }

    private static class TagInfo {
        boolean valid = false;
        int id = -1;
        double tx = 0.0, ty = 0.0, tz = 0.0;
        double yawDeg = 0.0, pitchDeg = 0.0, rollDeg = 0.0;
    }
}