package eu.qrobotics.ultimategoal.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import eu.qrobotics.ultimategoal.teamcode.util.DashboardUtil;
import eu.qrobotics.ultimategoal.teamcode.util.MecanumUtil;

import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.HEADING_PID;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.HEADING_PID_TELEOP;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.LATERAL_MULTIPLIER;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.MAX_ANG_ACCEL;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.MAX_ANG_VEL;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.MOTOR_VELO_PID;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.RUN_USING_ENCODER;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.TRACK_WIDTH;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.TRANSLATIONAL_PID;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.TRANSLATIONAL_PID_TELEOP;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.encoderTicksToInches;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.kA;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.kStatic;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.kV;

@Config
public class Drivetrain extends MecanumDrive implements Subsystem {
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static int POSE_HISTORY_LIMIT = 100;

    public enum Mode {
        TURN,
        FOLLOW_TRAJECTORY,
        IDLE
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private TrajectoryFollower follower;

    private LinkedList<Pose2d> poseHistory;

    private Robot robot;
    private boolean isAutonomous;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private double[] motorPowers;

    private VoltageSensor batteryVoltageSensor;

    private Pose2d lastPoseOnTurn;

    private static Pose2d autonomousEndPose = new Pose2d(0, 0, 0);
    public boolean fieldCentric = false;

    Drivetrain(HardwareMap hardwareMap, Robot robot, boolean isAutonomous) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        this.robot = robot;
        this.isAutonomous = isAutonomous;

        dashboard = FtcDashboard.getInstance();
        clock = NanoClock.system();

        mode = Mode.IDLE;

        // Initialize autonomous specific stuff
        turnController = new PIDFController(isAutonomous ? HEADING_PID : HEADING_PID_TELEOP);
        turnController.setInputBounds(0, 2 * Math.PI);

        if(isAutonomous) {
            follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                    new Pose2d(0.25, 0.25, Math.toRadians(0.0)), 1.5);
        }
        else {
            follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID_TELEOP, TRANSLATIONAL_PID_TELEOP, HEADING_PID_TELEOP,
                    new Pose2d(0.25, 0.25, Math.toRadians(0.0)), 1.5);
        }
        motorPowers = new double[]{0.0, 0.0, 0.0, 0.0};

        poseHistory = new LinkedList<>();

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        setLocalizer(new OdometryTwoWheel(hardwareMap));

        setPoseEstimate(autonomousEndPose);
    }

    public void turn(double angle) {
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                MAX_ANG_VEL,
                MAX_ANG_ACCEL
        );
        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turnSync(double angle) {
        turn(angle);
        waitForIdle();
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectorySync(Trajectory trajectory) {
        followTrajectory(trajectory);
        waitForIdle();
    }

    public void cancelTrajectory() {
        mode = Mode.IDLE;
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return isAutonomous ? follower.getLastError() : new Pose2d();
        }
        throw new AssertionError();
    }

    public void setMotorPowersFromGamepad(Gamepad gg, double scale) {
        MecanumUtil.Motion motion = MecanumUtil.joystickToMotion(gg.left_stick_x, gg.left_stick_y,
                gg.right_stick_x, gg.right_stick_y);
        if (fieldCentric) {
            motion = motion.toFieldCentricMotion(getPoseEstimate().getHeading());
        }
        MecanumUtil.Wheels wh = MecanumUtil.motionToWheelsFullSpeed(motion).scaleWheelPower(scale); // Use full forward speed on 19:1 motors
        motorPowers[0] = wh.frontLeft;
        motorPowers[1] = wh.backLeft;
        motorPowers[2] = wh.backRight;
        motorPowers[3] = wh.frontRight;
    }

    public double[] getMotorPower() {
        return motorPowers;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if(IS_DISABLED) return;
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();
        autonomousEndPose = currentPose;

        poseHistory.add(currentPose);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading (deg)", Math.toDegrees(currentPose.getHeading()));

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError (deg)", Math.toDegrees(lastError.getHeading()));

        switch (mode) {
            case IDLE:
                if(isAutonomous) {
                    DriveSignal driveSignal = follower.update(currentPose);
                    setDriveSignal(driveSignal);
                    packet.put("Target velocity", follower.getTrajectory().velocity(follower.getTrajectory().duration()).toString());
                    packet.put("Target position", follower.getTrajectory().get(follower.getTrajectory().duration()).toString());
                    packet.put("Drive vel", driveSignal.getVel().toString());
                    packet.put("Drive accel", driveSignal.getAccel().toString());
                }
                else {
                    setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
                }
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose, getPoseVelocity()));

//                if (!follower.isFollowing()) {
                if(follower.getTrajectory().duration() - follower.elapsedTime() < 0) {
                    mode = Mode.IDLE;
//                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);

        fieldOverlay.setStroke("#000000");
        fieldOverlay.fillCircle(robot.outtake.outtakeTarget.getPosition().getX(), robot.outtake.outtakeTarget.getPosition().getY(), 3);

        packet.put("Battery voltage", batteryVoltageSensor.getVoltage());
        packet.put("Target RPM", robot.outtake.getTargetRPM());
        packet.put("Current RPM", robot.outtake.getCurrentRPM());
        packet.put("Target angle", robot.outtake.getTargetTurretAngle());
        packet.put("Outtake target", robot.outtake.outtakeTarget);
//        packet.put("Camera Pose2d", ((T265Odometry)getLocalizer()).getCameraUpdate().pose.toString());
//        packet.put("Camera Speed", ((T265Odometry)getLocalizer()).getCameraUpdate().velocity.toString());
//        packet.put("Camera Confidence", ((T265Odometry)getLocalizer()).getCameraUpdate().confidence);
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return 0;//imu.getAngularOrientation().firstAngle;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            robot.sleep(0.05);
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }
}
