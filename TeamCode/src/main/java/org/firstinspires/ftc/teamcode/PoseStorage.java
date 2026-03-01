// PoseStorage.java
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    public static Pose2d currentPose = new Pose2d(0, 0, 0);
}

//PoseStorage.currentPose = drive.localizer.getPose(); add at the end of the AUTOs
//Intialize this pose in mecanumdrive intialization in teleop!!!!!