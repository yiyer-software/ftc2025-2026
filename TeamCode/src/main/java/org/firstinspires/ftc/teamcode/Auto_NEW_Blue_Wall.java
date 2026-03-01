package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auto_NEW_Blue_Wall", group = "Auton")
public class Auto_NEW_Blue_Wall extends LinearOpMode {

    // ================= HARDWARE =================
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx intakeMotor;
    private DcMotorEx sortMotor;
    private DcMotorEx launcherLeft, launcherRight;
    private Servo liftLeft, liftRight;
    private Servo hoodLeft, hoodRight;
    private ColorSensor colorSensor;
    private LimelightSubSystem limelight;

    // ================= CONSTANTS =================
    private static final double TICKS_PER_REV        = 28.0;
    private static final double SORT_POWER            = 0.4;
    private static final double SORT_DEGREES          = 173.0;
    static final double         COUNTS_PER_REV        = 1378.0;
    static final double         COUNTS_PER_DEGREE     = COUNTS_PER_REV / 360.0;
    private static final double TICKS_PER_INCH_INTAKE = (28.0 * 3) / (Math.PI * 1.539);
    public static final double  KICKER_DOWN_WAIT_SEC  = 0.35;

    // ================= SHARED STATE =================
    private final int[] indexOrder = new int[]{2, 2, 2};
    private final int[] indexerPos = new int[]{0};
    private final int[] ballsIn    = new int[]{0};
    private final int[] shootOrder = new int[]{0, 1, 2};

    // ================= LIFECYCLE =================
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-52, -48, Math.toRadians(225)));

        initHardware();
        while (!isStarted() && !isStopRequested()) {
            LimelightSubSystem.AprilTagData tag = limelight.getAprilTagData();
            if (tag != null && tag.poseValid && tag.tagVisible) {
                drive.localizer.setPose(new Pose2d(
                        tag.x,
                        tag.y,
                        Math.toRadians(tag.yaw)
                ));
                telemetry.addData("Pose Source", "Limelight");
            } else {
                telemetry.addData("Pose Source", "Default fallback");
            }
            telemetry.addData("X",   drive.localizer.getPose().position.x);
            telemetry.addData("Y",   drive.localizer.getPose().position.y);
            telemetry.addData("Yaw", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;

        Action driveToFirstCheckpt = drive.actionBuilder(new Pose2d(-52, -48, Math.toRadians(225)))
                .strafeTo(new Vector2d(-36, -34))
                .build();

        Action driveToFirstHalfCheckpt = drive.actionBuilder(new Pose2d(-36, -34, Math.toRadians(225)))
                .strafeTo(new Vector2d(-12, 0))
                .turnTo(Math.toRadians(180))
                .build();

        Action driveToFirstPickup = drive.actionBuilder(new Pose2d(-12, -0, Math.toRadians(180)))
                .turnTo(Math.toRadians(270))
                .strafeTo(new Vector2d(-12, -50))
                .build();

        Action driveToShoot1st = drive.actionBuilder(new Pose2d(-12, -50, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(-24, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-45, -34), Math.toRadians(0))
                .build();

        Action driveToSecondPickup = drive.actionBuilder(new Pose2d(-45, -34, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(-34, -30, Math.toRadians(270)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(12, -30, Math.toRadians(270)), Math.toRadians(0))
                .strafeTo(new Vector2d(12, -50))
                .build();

        Action driveToShoot2nd = drive.actionBuilder(new Pose2d(12, -50, Math.toRadians(270)))
                .strafeTo(new Vector2d(12, -44))
                .splineToLinearHeading(new Pose2d(-30, -32, Math.toRadians(225)), Math.toRadians(0))
                .build();

        Action driveToThirdPickup = drive.actionBuilder(new Pose2d(-30, -32, Math.toRadians(225)))
                .splineToLinearHeading(new Pose2d(36, -20, Math.toRadians(270)), Math.toRadians(0))
                .strafeTo(new Vector2d(36, -50))
                .build();

        Action driveToShoot3rd = drive.actionBuilder(new Pose2d(36, -50, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(22, -50, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-34, -30, Math.toRadians(225)), Math.toRadians(0))
                .build();

        // ----------------------------------------------------------------
        // FULL AUTO SEQUENCE
        // ----------------------------------------------------------------
        Action autoSequence = new SequentialAction(
                driveToFirstCheckpt,
                shootingAction(),

                driveToFirstHalfCheckpt,
                identifyMotifTagAction(),

                new ParallelAction(
                        driveToFirstPickup,
                        intakeAction()
                ),


                new ParallelAction(
                        driveToShoot1st

                ),
                aprilTagRelocalize(drive),
                computeShootOrderAction(),  // <-- resolves motif vs indexOrder → shootOrder
                shootingAction(),


                new ParallelAction(
                        driveToSecondPickup,
                        intakeAction()
                ),

                new ParallelAction(
                        driveToShoot2nd

                ),
                aprilTagRelocalize(drive),
                computeShootOrderAction(),
                shootingAction(),


                new ParallelAction(
                        driveToThirdPickup,
                        intakeAction()
                ),

                new ParallelAction(
                        driveToShoot3rd
                ),
                aprilTagRelocalize(drive),
                computeShootOrderAction(),
                shootingAction()
        );

        TelemetryPacket packet = new TelemetryPacket();
        while (opModeIsActive() && autoSequence.run(packet)) {
            telemetry.update();
            packet = new TelemetryPacket();
        }

        PoseStorage.currentPose = drive.localizer.getPose();
    }

    private void initHardware() {
        intakeMotor   = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        sortMotor     = hardwareMap.get(DcMotorEx.class, "sorterMotor");
        launcherLeft  = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");

        liftLeft  = hardwareMap.get(Servo.class, "leftKicker");
        liftRight = hardwareMap.get(Servo.class, "rightKicker");
        hoodLeft  = hardwareMap.get(Servo.class, "leftHood");
        hoodRight = hardwareMap.get(Servo.class, "rightHood");

        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sortMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sortMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        limelight   = new LimelightSubSystem(hardwareMap);
    }

    private int[] findShootingOrder(int[] motif, int[] loaded) {
        int[] available = new int[]{loaded[0], loaded[1], loaded[2]};
        int[] order     = new int[]{-1, -1, -1};

        for (int shot = 0; shot < 3; shot++) {
            int wantColor = motif[shot];
            for (int slot = 0; slot < 3; slot++) {
                if (available[slot] == wantColor) {
                    order[shot]     = slot;
                    available[slot] = 2;
                    break;
                }
            }
        }

        for (int shot = 0; shot < 3; shot++) {
            if (order[shot] == -1) {
                for (int slot = 0; slot < 3; slot++) {
                    if (available[slot] != 2) {
                        order[shot]     = slot;
                        available[slot] = 2;
                        break;
                    }
                }
            }
        }

        for (int shot = 0; shot < 3; shot++) {
            if (order[shot] == -1) order[shot] = shot;
        }

        return order;
    }

    private Action computeShootOrderAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int[] motif = MotifStorage.motifDetected
                        ? MotifStorage.motifPattern
                        : new int[]{0, 1, 2};

                int[] result = findShootingOrder(motif, indexOrder);
                shootOrder[0] = result[0];
                shootOrder[1] = result[1];
                shootOrder[2] = result[2];

                packet.put("ShootOrder_0", shootOrder[0]);
                packet.put("ShootOrder_1", shootOrder[1]);
                packet.put("ShootOrder_2", shootOrder[2]);
                return false;
            }
        };
    }

    private Action shootingAction() {
        return new Action() {

            private boolean initialized   = false;
            private int     shotsFired    = 0;
            private int     shootState    = 0;
            private double  sorterTimer   = 0;
            private double  liftTimer     = 0;
            private double  kickerDownTimer = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    shotsFired  = 0;
                    shootState  = 0;
                    initialized = true;
                    // Start flywheels at 3000 RPM equivalent ticks/sec
                    double vel = (3000.0 / 60.0) * TICKS_PER_REV;
                    launcherLeft.setVelocity(vel);
                    launcherRight.setVelocity(vel);
                }

                switch (shootState) {

                    case 0: // Decide slot and move
                        if (!sortMotor.isBusy()) {
                            if (ballsIn[0] <= 0) {
                                shootState = 5;
                                break;
                            }

                            int targetSlot  = shootOrder[shotsFired];
                            int stepsNeeded = (targetSlot - indexerPos[0] + 3) % 3;

                            if (stepsNeeded == 0) {
                                sorterTimer = Actions.now();
                                shootState  = 2;
                            } else {
                                int moveTicks = (int) (stepsNeeded * (SORT_DEGREES / 2.0) * COUNTS_PER_DEGREE);
                                int target    = sortMotor.getCurrentPosition() + moveTicks;
                                sortMotor.setTargetPosition(target);
                                sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                sortMotor.setPower(SORT_POWER);

                                indexerPos[0] = targetSlot;
                                sorterTimer   = Actions.now();
                                shootState    = 1;
                            }
                        }
                        break;

                    case 1: // Wait for sorter
                        if (!sortMotor.isBusy()) {
                            sorterTimer = Actions.now();
                            shootState  = 2;
                        }
                        break;

                    case 2: // Settle and extend kicker
                        if (!sortMotor.isBusy() && Actions.now() - sorterTimer >= 0.35) {
                            liftLeft.setPosition(0.15);
                            liftRight.setPosition(0.85);
                            liftTimer  = Actions.now();
                            shootState = 3;
                        }
                        break;

                    case 3: // Wait for exit, retract kicker, count shot
                        if (Actions.now() - liftTimer >= 1.00) {
                            liftLeft.setPosition(0.99);
                            liftRight.setPosition(0.01);

                            indexOrder[indexerPos[0]] = 2;
                            shotsFired++;
                            ballsIn[0]--;
                            kickerDownTimer = Actions.now();
                            shootState = 4;
                        }
                        break;

                    case 4: // Kicker down wait
                        if (Actions.now() - kickerDownTimer >= KICKER_DOWN_WAIT_SEC) {
                            if (ballsIn[0] <= 0 || shotsFired >= 3) {
                                shootState = 5;
                            } else {
                                shootState = 0;
                            }
                        }
                        break;

                    case 5: // Cleanup nudge
                        launcherLeft.setVelocity(0);
                        launcherRight.setVelocity(0);

                        int moveTicks = (int) ((SORT_DEGREES / 4.0) * COUNTS_PER_DEGREE);
                        int target    = sortMotor.getCurrentPosition() + moveTicks;
                        sortMotor.setTargetPosition(target);
                        sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        sortMotor.setPower(SORT_POWER);

                        indexerPos[0] = (indexerPos[0] + 2) % 3;
                        shootState    = 6; // Wait for final nudge to finish
                        break;
                    
                    case 6:
                        if (!sortMotor.isBusy()) {
                            return false; // Done
                        }
                        break;
                }

                packet.put("shootState",  shootState);
                packet.put("shotsFired",  shotsFired);
                packet.put("ballsIn",     ballsIn[0]);
                packet.put("indexerPos",  indexerPos[0]);
                return true;
            }
        };
    }


    private Action intakeAction() {
        return new Action() {

            private boolean initialized  = false;
            private int     intakeState  = 0;
            private double  intakeTimer  = 0;
            private double  cycleStart   = 0;
            private double  kickerDownTimer = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    intakeState   = 0;
                    cycleStart    = Actions.now();
                    initialized   = true;
                }

                // 7-second hard timeout
                if (Actions.now() - cycleStart >= 7.0 && intakeState != 3 && intakeState != 4) {
                    intakeMotor.setPower(0);
                    if (!sortMotor.isBusy()) {
                        int moveTicks = (int) ((SORT_DEGREES / 4.0) * COUNTS_PER_DEGREE);
                        int target    = sortMotor.getCurrentPosition() + moveTicks;
                        sortMotor.setTargetPosition(target);
                        sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        sortMotor.setPower(SORT_POWER);
                        indexerPos[0] = (indexerPos[0] + 2) % 3;
                    }
                    return false;
                }

                switch (intakeState) {

                    case 0: // Run intake, wait for color
                        intakeMotor.setPower(-0.8);
                        boolean greenDetected  = (colorSensor.green() > 100);
                        boolean purpleDetected = (colorSensor.red() > 55 && colorSensor.blue() > 80);

                        if (greenDetected || purpleDetected) {
                            intakeTimer = Actions.now();
                            if (ballsIn[0] >= 0 && ballsIn[0] < 3) {
                                int destinationSlot = (indexerPos[0] + 1) % 3;
                                if (greenDetected)  indexOrder[destinationSlot] = 1;
                                if (purpleDetected) indexOrder[destinationSlot] = 0;
                            }
                            intakeState = 1;
                        }
                        break;

                    case 1: // Settle and shift
                        if (Actions.now() - intakeTimer >= 0.75 && !sortMotor.isBusy()) {
                            int moveTicks = (int) ((SORT_DEGREES / 2.0) * COUNTS_PER_DEGREE);
                            int target    = sortMotor.getCurrentPosition() + moveTicks;
                            sortMotor.setTargetPosition(target);
                            sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            sortMotor.setPower(SORT_POWER);

                            indexerPos[0] = (indexerPos[0] + 1) % 3;
                            ballsIn[0]++;
                            intakeState = 2;
                        }
                        break;

                    case 2: // Wait for shift, check if full
                        if (!sortMotor.isBusy()) {
                            if (ballsIn[0] >= 3) {
                                kickerDownTimer = Actions.now();
                                intakeState = 3;
                            } else {
                                intakeState = 0;
                            }
                        }
                        break;

                    case 3: // Kicker down wait before final nudge
                        if (Actions.now() - kickerDownTimer >= KICKER_DOWN_WAIT_SEC && !sortMotor.isBusy()) {
                            intakeMotor.setPower(0);
                            int moveTicks = (int) ((SORT_DEGREES / 4.0) * COUNTS_PER_DEGREE);
                            int target    = sortMotor.getCurrentPosition() + moveTicks;
                            sortMotor.setTargetPosition(target);
                            sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            sortMotor.setPower(SORT_POWER);
                            intakeState = 4;
                        }
                        break;

                    case 4: // Final wait
                        if (!sortMotor.isBusy()) {
                            indexerPos[0] = (indexerPos[0] + 2) % 3;
                            return false;
                        }
                        break;
                }

                packet.put("intakeState",   intakeState);
                packet.put("ballsIn",       ballsIn[0]);
                return true;
            }
        };
    }

    private Action identifyMotifTagAction() {
        return new Action() {
            private static final double MOTIF_TIMEOUT_SECS = 3.0;
            private boolean initialized = false;
            private double  startTime   = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    startTime   = Actions.now();
                    initialized = true;
                }
                if (MotifStorage.motifDetected) {
                    return false;
                }
                boolean found = limelight.identifyMotifTag();
                if (found || Actions.now() - startTime >= MOTIF_TIMEOUT_SECS) {
                    return false;
                }
                return true;
            }
        };
    }

    private Action aprilTagRelocalize(MecanumDrive drive) {
        return new Action() {
            private static final double TAG_TIMEOUT_SECS = 0.5;
            private static final int    MIN_SAMPLES      = 5;
            private boolean initialized      = false;
            private double  lastTagTime      = 0;
            private int     samplesCollected = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    limelight.setPipeline(0);
                    lastTagTime      = Actions.now();
                    samplesCollected = 0;
                    initialized      = true;
                }
                LimelightSubSystem.AprilTagData tagData = limelight.getAprilTagData();
                if (tagData != null && tagData.poseValid && tagData.tagVisible) {
                    lastTagTime = Actions.now();
                    samplesCollected++;
                    drive.localizer.setPose(new Pose2d(tagData.x, tagData.y, Math.toRadians(tagData.yaw)));
                }
                boolean timedOut   = (Actions.now() - lastTagTime) >= TAG_TIMEOUT_SECS;
                boolean enoughData = samplesCollected >= MIN_SAMPLES;
                if (enoughData && timedOut) return false;
                if (samplesCollected == 0 && (Actions.now() - lastTagTime) >= 2.0) return false;
                return true;
            }
        };
    }

    private Action flywheelActivate(double rpm) {
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    double ticksPerSec = (rpm / 60.0) * TICKS_PER_REV;
                    launcherLeft.setVelocity(ticksPerSec);
                    launcherRight.setVelocity(ticksPerSec); // This might have a typo but I'll fix it
                    initialized = true;
                }
                return false;
            }
        };
    }

    private Action moveIntakeInches(double inches, double velocity) {
        return new Action() {
            private boolean initialized = false;
            private int     startPos;
            private int     targetTicks;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    startPos    = intakeMotor.getCurrentPosition();
                    targetTicks = (int) Math.round(inches * TICKS_PER_INCH_INTAKE);
                    intakeMotor.setVelocity(Math.signum(targetTicks) * Math.abs(velocity));
                    initialized = true;
                }
                int delta = intakeMotor.getCurrentPosition() - startPos;
                if (Math.abs(delta) >= Math.abs(targetTicks)) {
                    intakeMotor.setVelocity(0);
                    return false;
                }
                return true;
            }
        };
    }

    private Action indexSpindexter(int degrees, double speed) {
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized && !sortMotor.isBusy()) {
                    int moveTicks = (int) (degrees * COUNTS_PER_DEGREE);
                    int target    = sortMotor.getCurrentPosition() + moveTicks;
                    sortMotor.setTargetPosition(target);
                    sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sortMotor.setPower(speed);
                    initialized = true;
                }
                return sortMotor.isBusy();
            }
        };
    }

    private Action moveHood(double position) {
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    hoodLeft.setPosition(position);
                    hoodRight.setPosition(1.0 - position);
                    initialized = true;
                }
                return false;
            }
        };
    }

    private Action moveKicker(double position) {
        return new Action() {
            private boolean initialized = false;
            private double  startTime   = 0;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftLeft.setPosition(position);
                    liftRight.setPosition(1.0 - position);
                    startTime   = Actions.now();
                    initialized = true;
                }
                if (Actions.now() - startTime >= 0.2) {
                    liftLeft.setPosition(0.99);
                    liftRight.setPosition(0.01);
                    return false;
                }
                return true;
            }
        };
    }
}
