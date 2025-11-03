package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class BallDetectionPipeline extends OpenCvPipeline {

    // IMPORTANT: TUNE THESE VALUES in EOCV-Sim or a similar tuning opmode!
    private static final Scalar GREEN_LOWER_BOUND = new Scalar(40, 100, 100);
    private static final Scalar GREEN_UPPER_BOUND = new Scalar(80, 255, 255);
    private static final Scalar PURPLE_LOWER_BOUND = new Scalar(130, 80, 80);
    private static final Scalar PURPLE_UPPER_BOUND = new Scalar(160, 255, 255);
    private static final double MIN_CONTOUR_AREA = 2500;

    private String targetColor = "NONE";
    private Mat hsvMat = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    
    private volatile boolean ballFound = false;
    private volatile double ballX = 0, ballY = 0, ballXError = 0;
    private final int cameraWidth;

    public BallDetectionPipeline(int cameraWidth) { this.cameraWidth = cameraWidth; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        if (targetColor.equals("GREEN")) Core.inRange(hsvMat, GREEN_LOWER_BOUND, GREEN_UPPER_BOUND, mask);
        else if (targetColor.equals("PURPLE")) Core.inRange(hsvMat, PURPLE_LOWER_BOUND, PURPLE_UPPER_BOUND, mask);
        else mask = Mat.zeros(hsvMat.size(), hsvMat.type());

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

        if (largestRect != null) {
            ballFound = true;
            ballX = largestRect.x + (largestRect.width / 2.0);
            ballY = largestRect.y + (largestRect.height / 2.0);
            ballXError = ballX - (cameraWidth / 2.0);
            Imgproc.rectangle(input, largestRect, new Scalar(0, 255, 0), 2);
        } else {
            ballFound = false;
            ballXError = 0;
        }
        return input;
    }
    
    public void setTargetColor(String color) { this.targetColor = color; }
    public boolean isBallFound() { return ballFound; }
    public double getBallX() { return ballX; }
    public double getBallY() { return ballY; }
    public double getBallXError() { return ballXError; }
}
