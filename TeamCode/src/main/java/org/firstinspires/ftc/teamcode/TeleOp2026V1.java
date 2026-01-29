package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;

import java.util.ArrayList;

@TeleOp(name = "TeleOp2026V1", group = "Competition")
public class TeleOp2026V1 extends LinearOpMode {

    // --- DRIVE MOTORS ---
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    // --- SHOOTER MOTORS ---
    private DcMotorEx launcherLeft, launcherRight;
    // --- INTAKE & SORTING MOTORS ---
    private DcMotor intakeMotor, sortMotor;
    // --- SERVOS ---
    private Servo hoodLeft, hoodRight, liftLeft, liftRight;
    // --- COLOR SENSOR ---
    private ColorSensor colorSensor;

    // --- DRIVE CONSTANTS ---
    private double TICKS_PER_REV = 28.0;
    private double MAX_RPM = 7000;
    private double DRIVE_TICKS_PER_SEC;
    private double DRIVE_DEADZONE = 0.03;

    // --- TOGGLES ---
    private boolean shooterOn=false, intakeOn=false, liftUp=false, sortOn=false;
    private boolean lastY=false, lastB=false, lastLeftBumper=false, lastRightStickDown=false, lastX=false, lastA=false, lastDpadUp=false, lastDpadDown=false;

    // --- SHOOTER & HOOD ---
    private double targetRPM=0;
    private double hoodPos=0.0;

    // --- DASHBOARD ---
    private FtcDashboard dashboard;
    private MultipleTelemetry telemetryDashboard;

    // --- CAMERA ---
    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagPipeline;
    private AprilTagDetection tagOfInterest=null;

    // --- ALIGNMENT PID ---
    private ElapsedTime alignmentTimer=new ElapsedTime();
    private double kP=0.03, kI=0.0, kD=0.002;
    private double lastRotationError=0;
    private double rotationIntegral=0;

    // --- AUTO-ALIGN DISTANCE TARGET ---
    private double TARGET_DISTANCE_INCHES=25.0;

    @Override
    public void runOpMode() {

        // --- HARDWARE MAP ---
        frontLeft = hardwareMap.get(DcMotorEx.class,"frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class,"frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class,"backLeft");
        backRight = hardwareMap.get(DcMotorEx.class,"backRight");

        launcherLeft = hardwareMap.get(DcMotorEx.class,"launcherLeft");
        launcherRight = hardwareMap.get(DcMotorEx.class,"launcherRight");

        intakeMotor = hardwareMap.get(DcMotor.class,"intakeMotor");
        sortMotor = hardwareMap.get(DcMotor.class,"sortMotor");

        hoodLeft = hardwareMap.get(Servo.class,"hoodLeft");
        hoodRight = hardwareMap.get(Servo.class,"hoodRight");
        liftLeft = hardwareMap.get(Servo.class,"liftLeft");
        liftRight = hardwareMap.get(Servo.class,"liftRight");

        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");

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
        sortMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DRIVE_TICKS_PER_SEC = (MAX_RPM/60.0)*TICKS_PER_REV;

        // --- DASHBOARD ---
        dashboard = FtcDashboard.getInstance();
        telemetryDashboard = new MultipleTelemetry(telemetry,dashboard.getTelemetry());

        // --- CAMERA ---
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"),cameraMonitorViewId);

        aprilTagPipeline = new AprilTagDetectionPipeline(0.166,578.272,578.272,402.145,221.506);
        camera.setPipeline(aprilTagPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override public void onOpened(){ camera.startStreaming(800,448,OpenCvCameraRotation.UPRIGHT);}
            @Override public void onError(int errorCode){}
        });

        waitForStart();

        while(opModeIsActive()){
            handleDrive();
            handleShooter();
            handleIntake();
            handleSort();
            handleLift();
            handleAutoAlignAndMove();
            handleHoodManual();
            updateTelemetry();
        }
    }

    // --- DRIVE ---
    private void handleDrive(){
        double y=-gamepad1.left_stick_y;
        double x=gamepad1.left_stick_x*1.15;
        double rx=-gamepad1.right_stick_x;

        if(Math.abs(y)<DRIVE_DEADZONE) y=0;
        if(Math.abs(x)<DRIVE_DEADZONE) x=0;
        if(Math.abs(rx)<DRIVE_DEADZONE) rx=0;

        double fl=y+x+rx;
        double fr=y-x-rx;
        double bl=y-x+rx;
        double br=y+x-rx;

        double max=Math.max(Math.abs(fl),Math.max(Math.abs(fr),Math.max(Math.abs(bl),Math.abs(br))));
        if(max>1.0){ fl/=max; fr/=max; bl/=max; br/=max; }

        frontLeft.setVelocity(fl*DRIVE_TICKS_PER_SEC);
        frontRight.setVelocity(fr*DRIVE_TICKS_PER_SEC);
        backLeft.setVelocity(bl*DRIVE_TICKS_PER_SEC);
        backRight.setVelocity(br*DRIVE_TICKS_PER_SEC);
    }

    // --- SHOOTER ---
    private void handleShooter(){
        boolean y=gamepad1.y;
        if(y&&!lastY) shooterOn=!shooterOn;
        lastY=y;

        if(shooterOn) targetRPM=5000;
        else targetRPM=0;

        if(gamepad1.right_trigger>0.5) targetRPM=6500;
        if(gamepad1.left_trigger>0.5) targetRPM=3500;

        double vel=targetRPM/60.0*TICKS_PER_REV;
        launcherLeft.setVelocity(vel);
        launcherRight.setVelocity(vel);
    }

    // --- INTAKE ---
    private void handleIntake(){
        boolean b=gamepad1.b;
        if(b&&!lastB) intakeOn=!intakeOn;
        lastB=b;
        intakeMotor.setPower(intakeOn?1:0);
    }

    // --- SORT ---
    private void handleSort(){
        boolean x=gamepad1.x;
        if(x&&!lastX) sortOn=!sortOn;
        lastX=x;
        sortMotor.setPower(sortOn?1:0);
    }

    // --- LIFT ---
    private void handleLift(){
        boolean lb=gamepad1.left_bumper;
        if(lb&&!lastLeftBumper) liftUp=!liftUp;
        lastLeftBumper=lb;
        liftLeft.setPosition(liftUp?1:0);
        liftRight.setPosition(liftUp?1:0);
    }

    // --- HOOD MANUAL UP/DOWN ---
    private void handleHoodManual(){
        boolean dpadUp=gamepad1.dpad_up;
        boolean dpadDown=gamepad1.dpad_down;
        if(dpadUp&&!lastDpadUp) hoodPos+=0.05;
        if(dpadDown&&!lastDpadDown) hoodPos-=0.05;
        lastDpadUp=dpadUp;
        lastDpadDown=dpadDown;
        if(hoodPos>1) hoodPos=1;
        if(hoodPos<0) hoodPos=0;
        hoodLeft.setPosition(hoodPos);
        hoodRight.setPosition(1-hoodPos);
    }

    // --- AUTO ALIGN & MOVE ---
    private void handleAutoAlignAndMove(){
        boolean rStickDown=gamepad1.right_stick_button;
        if(rStickDown&&!lastRightStickDown) alignmentTimer.reset();
        lastRightStickDown=rStickDown;

        if(alignmentTimer.milliseconds()<5000){
            Pose2D tagPose=getTag24Pose();
            if(tagPose==null) return;

            double headingError=0-tagPose.heading;
            rotationIntegral+=headingError*0.02;
            double rotationDerivative=(headingError-lastRotationError)/0.02;
            lastRotationError=headingError;

            double rotPower=kP*headingError+kI*rotationIntegral+kD*rotationDerivative;

            double distanceError=TARGET_DISTANCE_INCHES-tagPose.distance;
            double forwardPower=0.05*distanceError;
            if(forwardPower>0.5) forwardPower=0.5;
            if(forwardPower<-0.5) forwardPower=-0.5;

            double fl=forwardPower+rotPower;
            double fr=forwardPower-rotPower;
            double bl=forwardPower+rotPower;
            double br=forwardPower-rotPower;

            frontLeft.setVelocity(fl*DRIVE_TICKS_PER_SEC);
            frontRight.setVelocity(fr*DRIVE_TICKS_PER_SEC);
            backLeft.setVelocity(bl*DRIVE_TICKS_PER_SEC);
            backRight.setVelocity(br*DRIVE_TICKS_PER_SEC);

            if(tagPose.distance<12){ hoodPos=0.0; targetRPM=3500; }
            else if(tagPose.distance<24){ hoodPos=0.3; targetRPM=5000; }
            else{ hoodPos=0.5; targetRPM=6500; }

            double vel=targetRPM/60.0*TICKS_PER_REV;
            launcherLeft.setVelocity(vel);
            launcherRight.setVelocity(vel);

            hoodLeft.setPosition(hoodPos);
            hoodRight.setPosition(1-hoodPos);
        }
    }

    // --- GET TAG 24 DATA ---
    private Pose2D getTag24Pose(){
        ArrayList<AprilTagDetection> detections=aprilTagPipeline.getLatestDetections();
        for(AprilTagDetection tag:detections){
            if(tag.id==24){
                tagOfInterest=tag;
                double x=tag.pose.x;
                double y=tag.pose.y;
                double distance=Math.hypot(x,y)*39.37;
                double heading=Math.toDegrees(Math.atan2(y,x));
                return new Pose2D(x,y,heading,distance);
            }
        }
        tagOfInterest=null;
        return null;
    }

    // --- TELEMETRY ---
    private void updateTelemetry(){
        telemetryDashboard.clearAll();
        telemetryDashboard.addLine("SHOOTER");
        telemetryDashboard.addData("RPM",targetRPM);
        telemetryDashboard.addData("On",shooterOn);
        telemetryDashboard.addLine("HOOD");
        telemetryDashboard.addData("Pos",hoodPos);
        telemetryDashboard.addLine("INTAKE");
        telemetryDashboard.addData("Power",intakeMotor.getPower());
        telemetryDashboard.addLine("SORT");
        telemetryDashboard.addData("Power",sortMotor.getPower());
        telemetryDashboard.addLine("LIFT");
        telemetryDashboard.addData("Up",liftUp);
        telemetryDashboard.addLine("COLOR");
        telemetryDashboard.addData("Red",colorSensor.red());
        telemetryDashboard.addData("Blue",colorSensor.blue());
        telemetryDashboard.addLine("DRIVE");
        telemetryDashboard.addData("FL Vel",frontLeft.getVelocity());
        telemetryDashboard.addData("FR Vel",frontRight.getVelocity());
        telemetryDashboard.addData("BL Vel",backLeft.getVelocity());
        telemetryDashboard.addData("BR Vel",backRight.getVelocity());
        telemetryDashboard.addLine("AUTO-ALIGN");
        if(tagOfInterest!=null){
            telemetryDashboard.addData("Tag24 X/Y",tagOfInterest.pose.x+","+tagOfInterest.pose.y);
            telemetryDashboard.addData("Distance",Math.hypot(tagOfInterest.pose.x,tagOfInterest.pose.y)*39.37);
        } else telemetryDashboard.addLine("Tag 24 not detected");
        telemetryDashboard.update();
    }

    private class Pose2D{
        public double x,y,heading,distance;
        public Pose2D(double x,double y,double h,double d){ this.x=x; this.y=y; this.heading=h; this.distance=d; }
    }

    private class AprilTagDetectionPipeline extends OpenCvPipeline {
        public double tagsize, fx, fy, cx, cy;
        public ArrayList<AprilTagDetection> latestDetections=new ArrayList<>();
        public AprilTagDetectionPipeline(double ts,double fx,double fy,double cx,double cy){
            this.tagsize=ts; this.fx=fx; this.fy=fy; this.cx=cx; this.cy=cy;
        }
        @Override
        public Mat processFrame(Mat input){
            latestDetections.clear();
            // TODO: Add real AprilTag detection logic here
            return input;
        }
        public ArrayList<AprilTagDetection> getLatestDetections(){ return latestDetections; }
    }
}