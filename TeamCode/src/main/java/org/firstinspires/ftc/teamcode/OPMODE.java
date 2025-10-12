package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import java.util.Locale;
import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.DriveCommandMessages;
import org.firstinspires.ftc.teamcode.LocalizationTest;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.PoseMessage;

@Autonomous(name = "AutoVision", group = "Vision")
public class AutoVision extends LinearOpMode(){
  private VisionPortal vision1;
  private AprilTagProcessor april1;
  Private DcMotor frontLeft, frontRight, backLeft, backRight;

  @Override
  public void runOpMode() {
    frontLeft = hardwareMap.get(DcMotor.class, deviceName: "frontLeft); //Hardware map is telling the hub whats plugged in
    frontRight = hardwareMap.get(DcMotor.class, deviceName: "frontRight");
    backLeft = hardwareMap.get(DcMotor.class, deviceName: "backLeft");
    backRight = hardwareMap.get(DcMotor.class, deviceName: "backRight");

    frontLeft.setDirection(DcMotor.Direction.FORWARD);
    frontRight.setDirection(DcMotor.Direction.FORWARD);
    backLeft.setDirection(DcMotor.Direction.FORWARD);
    backRight.setDirection(DcMotor.Direction.FORWARD);

    //Hard Brake: motorOne.setZeroPowerBehavior(DcMotor.setZeroPowerBehavior.BRAKE)
    //Soft/Coast Brake: motorOne.setZeroPowerBehavior(DcMotor.setZeroPowerBehavior.FLOAT)
    //Encoders
    //motorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER)- Tell's motor to use encoder to manage any power set to it (doesn't matter what battery, etc...)
    //motorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)- Opposite of above
    //motorOne.setMode(DcMotor.RunMode.STOP_AND_REST_ENCODER)- Measures in ticks value, sets to zero, and starts tick count again
    //motorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION)

    //USE DCMOTOREX (DcMotorEx) b/c its motor accurate, and precise 
    waitForStart();

    frontLeft.setPower(1)
    frontRight.setPower(1)
    BackLeft.setPower(1)
    BackRight.setPower(1)

      
    
  
