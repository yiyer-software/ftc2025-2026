package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TeleOpParameters {
    // Motor Directions
    public static final DcMotor.Direction SHOULDER_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction ELBOW_DIRECTION = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction SPECIMEN_DIRECTION = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction INTAKE_DIRECTION = DcMotor.Direction.REVERSE;

    // Servo Directions
    public static final double SPEC_LSERVO_POWER = 1.0;
    public static final double SPEC_RSERVO_POWER = -1.0;
    public static final double CLIMBER_LSERVO_POWER = -1.0;
    public static final double CLIMBER_RSERVO_POWER = 1.0;
}
