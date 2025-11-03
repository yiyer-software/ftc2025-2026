package org.firstinspires.ftc.teamcode;// --- FTC SDK Core ---
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// --- Telemetry (optional but useful) ---
import org.firstinspires.ftc.robotcore.external.Telemetry;

// --- Road Runner Core ---
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

// --- Your Road Runner Drive Code ---
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

// --- Math Utilities ---
import java.util.Arrays;
import java.lang.Math;

// --- Optional: Dashboard telemetry (if you use FTC Dashboard) ---
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Autonomous(name="AutoMode")
public class AutoMode extends LinearOpMode{
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private Servo servo1 = null;
    private Servo servo2 = null;
    private DcMotor flywheel1 = null;
    private DcMotor flywheel2 = null;
    private DcMotor Intake = null;


    @Override
    public void runOpMode(){

        Pose2d begin = new Pose2d(-70, -34, 90);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, begin);

        waitForStart();

        Trajectory traj = drive.trajectoryBuilder(begin).build(); //Add type of path, like spilne, or strafe in between the build, and the builder
        //Can also add the stopAndAdd method call




    }

    private void servoRaiseBall(){

    }

    private void flyWheelShoot(){

    }

    private void intakeActivate(){

    }



}