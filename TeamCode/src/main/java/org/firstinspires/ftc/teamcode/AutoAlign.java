package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

public class AutoAlign implements Action {

    private final Limelight3A limelight;
    private final MecanumDrive drive;

    private double kStrafe = 0.03;
    private double kTurn = 0.02;

    public AutoAlign(Limelight3A limelight, MecanumDrive drive) {
        this.limelight = limelight;
        this.drive = drive;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            stopDrive();
            return false;
        }

        List<FiducialResult> tags = result.getFiducialResults();
        if (tags.isEmpty()) {
            stopDrive();
            return false;
        }

        FiducialResult tag = tags.get(0);

        double tx = tag.getTargetXDegrees();
        double yaw = tag.getTargetPoseCameraSpace().getOrientation().getYaw();

        double strafe = -tx * kStrafe;
        double turn = -yaw * kTurn;

        strafe = Range.clip(strafe, -0.6, 0.6);
        turn = Range.clip(turn, -0.6, 0.6);

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0, strafe),
                        turn
                )
        );

        return Math.abs(tx) < 1.0 && Math.abs(yaw) < 2.0;
    }

    private void stopDrive() {
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0, 0),
                        0
                )
        );
    }
}