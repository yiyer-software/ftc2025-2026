package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class BallDetectionPipeline extends OpenCvPipeline {

    // Tune these in EOCV-Sim
    private static final Scalar GREEN_LOWER_BOUND = new Scalar(40, 100, 80);
    private static final Scalar GREEN_UPPER_BOUND = new Scalar(85, 255, 255);
    private static final Scalar PURPLE_LOWER_BOUND = new Scalar(130, 80, 80);
    private static final Scalar PURPLE_UPPER_BOUND = new Scalar(165, 255, 255);
    private static final double MIN_CONTOUR_AREA = 1500.0;

    private String targetColor = "GREEN";  // default target color (you can change to PURPLE)
    private Mat hsvMat = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();

    private volatile boolean ballFound = false;
    private volatile double ballX = 0, ballY = 0, ballXError = 0;
    private final int cameraWidth;

    public BallDetectionPipeline(int cameraWidth) {
        this.cameraWidth = cameraWidth;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Convert to HSV color space
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Threshold based on chosen color
        if (targetColor.equalsIgnoreCase("GREEN"))
            Core.inRange(hsvMat, GREEN_LOWER_BOUND, GREEN_UPPER_BOUND, mask);
        else if (targetColor.equalsIgnoreCase("PURPLE"))
            Core.inRange(hsvMat, PURPLE_LOWER_BOUND, PURPLE_UPPER_BOUND, mask);
        else
            mask.setTo(new Scalar(0));

        // Smooth mask to reduce noise
        Imgproc.GaussianBlur(mask, mask, new Size(5, 5), 0);

        // Find contours
        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = 0;
        Rect largestRect = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > MIN_CONTOUR_AREA && area > maxArea) {
                maxArea = area;
                largestRect = Imgproc.boundingRect(contour);
            }
        }

        // If ball found
        if (largestRect != null) {
            ballFound = true;
            ballX = largestRect.x + (largestRect.width / 2.0);
            ballY = largestRect.y + (largestRect.height / 2.0);
            ballXError = ballX - (cameraWidth / 2.0);

            // Draw rectangle and center point
            Imgproc.rectangle(input, largestRect, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input,
                    new org.opencv.core.Point(ballX, ballY),
                    4, new Scalar(255, 0, 0), -1);
        } else {
            ballFound = false;
            ballXError = 0;
        }

        return input;
    }

    // Accessors
    public void setTargetColor(String color) { this.targetColor = color; }
    public boolean isBallFound() { return ballFound; }
    public double getBallX() { return ballX; }
    public double getBallY() { return ballY; }
    public double getBallXError() { return ballXError; }
}
