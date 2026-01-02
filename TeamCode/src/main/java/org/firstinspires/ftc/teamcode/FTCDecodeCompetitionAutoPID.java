package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import java.util.ArrayList;

@Autonomous(name="FTCDecodeCompetitionAutoPID", group="Competition")
public class FTCDecodeCompetitionAutoPID extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor launcherLeft, launcherRight, intakeMotor;
    private Servo liftLeft, liftRight;

    private OpenCvCamera camera;
    private BallDetectionPipeline ballPipeline;
    private AprilTagDetectionPipeline aprilTagPipeline;

    private String[] currentPattern = {"-", "-", "-"};

    private static final int CAMERA_WIDTH = 800, CAMERA_HEIGHT = 448;
    private static final double CAMERA_FX = 578.272, CAMERA_FY = 578.272, CAMERA_CX = 402.145, CAMERA_CY = 221.506;
    private static final double APRILTAG_SIZE = 0.166;
    private static final double LAUNCH_ZONE_MAX_DISTANCE = 2.5;
    private static final int TAG_LEFT = 20, TAG_M1 = 21, TAG_M2 = 22, TAG_M3 = 23, TAG_RIGHT = 24;

    private static final double COUNTS_PER_MOTOR_REV = 28;
    private static final double WHEEL_DIAMETER_INCHES = 4;
    private static final double GEAR_REDUCTION = 1;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV*GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES*Math.PI);
    private static final double ROTATE_180_INCHES = 12;

    private static final double INTAKE_POWER = 0.9;
    private static final double FLAP_UP_POS = 0.85;
    private static final double FLAP_DOWN_POS = 0.0;
    private static final double MAX_DRIVE_POWER = 1.0;
    private static final double FLYWHEEL_MAX_RPM = 6000;

    private double lastLauncherPos = 0;
    private long lastTime = 0;
    private double integral = 0;
    private double lastError = 0;
    private double currentRPM = 0;

    private final double kP = 0.0004, kI = 0.0000005, kD = 0.0001;

    @Override
    public void runOpMode() {
        initHardware();
        initCamera();
        waitForStart();

        lastTime = System.currentTimeMillis();
        lastLauncherPos = launcherLeft.getCurrentPosition();

        driveToLaunchZone();
        AprilTagDetection tag = getLatestTag();
        if(tag!=null) mapTagToPattern(tag.id);
        alignToTagPID(tag);
        launchBalls(3, tag);

        for(int i=0;i<3;i++){
            rotate180();
            acquireBall(currentPattern[i]);
            rotate180();
            driveToLaunchZone();
            tag = getLatestTag();
            alignToTagPID(tag);
            launchBalls(1, tag);
        }

        stopAll();
    }

    private void initHardware(){
        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        backRight = hardwareMap.get(DcMotor.class,"backRight");
        launcherLeft = hardwareMap.get(DcMotor.class,"launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class,"launcherRight");
        intakeMotor = hardwareMap.get(DcMotor.class,"intakeMotor");
        liftLeft = hardwareMap.get(Servo.class,"liftServoLeft");
        liftRight = hardwareMap.get(Servo.class,"liftServoRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();
        setAllMotorsPID();
    }

    private void initCamera(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);
        aprilTagPipeline = new AprilTagDetectionPipeline(
                APRILTAG_SIZE,CAMERA_FX,CAMERA_FY,CAMERA_CX,CAMERA_CY);
        ballPipeline = new BallDetectionPipeline(CAMERA_WIDTH);
        camera.setPipeline(aprilTagPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override public void onOpened(){ camera.startStreaming(CAMERA_WIDTH,CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);}
            @Override public void onError(int errorCode){}
        });
    }

    private void resetEncoders(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setAllMotorsPID(){
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void driveToLaunchZone(){
        long startTime = System.currentTimeMillis();
        while(opModeIsActive()){
            if(System.currentTimeMillis() - startTime > 5000) break; // 5s timeout
            AprilTagDetection tag = getLatestTag();
            if(tag != null && tag.pose.z <= LAUNCH_ZONE_MAX_DISTANCE) break;
            setMecanumDrivePID(0.35,0,0);
            sleep(20);
        }
        stopDrive();
    }

    private void setMecanumDrivePID(double d,double s,double t){
        double fl=d+s+t, fr=d-s-t, bl=d-s+t, br=d+s-t;
        double max = Math.max(MAX_DRIVE_POWER,Math.max(Math.abs(fl),Math.max(Math.abs(fr),Math.max(Math.abs(bl),Math.abs(br)))));
        frontLeft.setPower(fl/max);
        frontRight.setPower(fr/max);
        backLeft.setPower(bl/max);
        backRight.setPower(br/max);
    }

    private AprilTagDetection getLatestTag(){
        ArrayList<AprilTagDetection> detections = aprilTagPipeline.getLatestDetections();
        if(detections != null && detections.size()>0) return detections.get(0);
        return null;
    }

    private void mapTagToPattern(int id){
        if(id==TAG_LEFT) currentPattern=new String[]{"GREEN","PURPLE","PURPLE"};
        else if(id==TAG_M1) currentPattern=new String[]{"PURPLE","GREEN","PURPLE"};
        else if(id==TAG_M2 || id==TAG_M3) currentPattern=new String[]{"PURPLE","PURPLE","GREEN"};
        else if(id==TAG_RIGHT) currentPattern=new String[]{"GREEN","PURPLE","PURPLE"};
    }

    private void alignToTagPID(AprilTagDetection tag){
        long startTime = System.currentTimeMillis();
        while(opModeIsActive()){
            if(System.currentTimeMillis()-startTime>3000) break; // 3s timeout
            AprilTagDetection t = getLatestTag();
            if(t==null){ stopDrive(); sleep(20); continue;}
            double yaw = getYaw(t);
            if(Math.abs(yaw)<2) break;
            setMecanumDrivePID(0,
                    0,-0.02*yaw);
            sleep(20);
        }
        stopDrive();
    }

    private double getYaw(AprilTagDetection d) {
        double r00 = d.pose.R.get(0,0);
        double r10 = d.pose.R.get(1,0);
        double yawRad = Math.atan2(r10, r00);
        return Math.toDegrees(yawRad);
    }

    private void launchBalls(int count, AprilTagDetection tag){
        liftLeft.setPosition(FLAP_DOWN_POS);
        liftRight.setPosition(1-FLAP_DOWN_POS);

        double distance = tag!=null ? tag.pose.z : 2.0;
        double targetRPM = Math.min(FLYWHEEL_MAX_RPM, 3200 + 400*(distance-2.0));
        spinFlywheel(targetRPM);

        sleep(700);
        for(int i=0;i<count;i++){
            liftLeft.setPosition(FLAP_UP_POS);
            liftRight.setPosition(1-FLAP_UP_POS);
            sleep(300);
            liftLeft.setPosition(FLAP_DOWN_POS);
            liftRight.setPosition(1-FLAP_DOWN_POS);
            sleep(200);
        }
        stopFlywheel();
    }

    private void spinFlywheel(double rpm){
        long now = System.currentTimeMillis();
        double pos = launcherLeft.getCurrentPosition();
        double deltaTime = (now-lastTime)/1000.0;
        double deltaPos = pos-lastLauncherPos;
        lastLauncherPos = pos;
        lastTime = now;

        currentRPM = (deltaPos/COUNTS_PER_MOTOR_REV)/deltaTime*60.0;
        double error = rpm-currentRPM;
        integral += error*deltaTime;
        double derivative = (error-lastError)/deltaTime;
        lastError=error;
        double power = Math.max(0.0, Math.min(1.0, kP*error + kI*integral + kD*derivative));
        launcherLeft.setPower(power);
        launcherRight.setPower(power);
    }

    private void stopFlywheel(){
        launcherLeft.setPower(0);
        launcherRight.setPower(0);
    }

    private void rotate180(){
        int move = (int)(ROTATE_180_INCHES*COUNTS_PER_INCH);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition()+move);
        backLeft.setTargetPosition(backLeft.getCurrentPosition()+move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition()-move);
        backRight.setTargetPosition(backRight.getCurrentPosition()-move);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(0.8);
        backLeft.setPower(0.8);
        frontRight.setPower(0.8);
        backRight.setPower(0.8);

        while(opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy()) sleep(20);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stopDrive();
    }

    private void acquireBall(String color){
        camera.setPipeline(ballPipeline);
        ballPipeline.setTargetColor(color);

        while(opModeIsActive() && !ballPipeline.isBallFound()){
            rotateInPlace(0.2);
            sleep(20);
        }
        stopDrive();

        while(opModeIsActive() && ballPipeline.isBallFound()){
            double errorX = ballPipeline.getBallXError();
            double forward = (CAMERA_HEIGHT-ballPipeline.getBallY())*0.004;
            double turn = -errorX*0.004;
            setMecanumDrivePID(Math.max(-0.6, Math.min(0.6,forward)),0,turn);
            intakeMotor.setPower(INTAKE_POWER);
            if(ballPipeline.getBallY()>CAMERA_HEIGHT*0.85) break;
            sleep(20);
        }
        intakeMotor.setPower(0);
        camera.setPipeline(aprilTagPipeline);
    }

    private void rotateInPlace(double p){
        frontLeft.setPower(p);
        backLeft.setPower(p);
        frontRight.setPower(-p);
        backRight.setPower(-p);
    }

    private void stopDrive(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void stopAll(){
        stopDrive();
        stopFlywheel();
        intakeMotor.setPower(0);
        camera.stopStreaming();
    }
}