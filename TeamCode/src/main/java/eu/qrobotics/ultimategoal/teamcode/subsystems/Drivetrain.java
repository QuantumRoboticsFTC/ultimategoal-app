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
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import eu.qrobotics.ultimategoal.teamcode.util.DashboardUtil;
import eu.qrobotics.ultimategoal.teamcode.util.MecanumUtil;

import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.BASE_CONSTRAINTS;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.HEADING_PID;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.LATERAL_MULTIPLIER;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.MOTOR_VELO_PID;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.RUN_USING_ENCODER;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.TRACK_WIDTH;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.TRANSLATIONAL_PID;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.WHEEL_BASE;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.encoderTicksToInches;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.kA;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.kStatic;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.kV;

@Config
public class Drivetrain extends MecanumDrive implements Subsystem {

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

    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    private List<Double> lastWheelPositions;
    private double lastTimestamp;

    private Robot robot;
    private boolean isAutonomous;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private double[] motorPowers;

    private static Pose2d autonomousEndPose = new Pose2d(0, 0, 0);
    public boolean fieldCentric = false;

    Drivetrain(HardwareMap hardwareMap, Robot robot, boolean isAutonomous) {
        super(kV, kA, kStatic, TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);

        this.robot = robot;
        this.isAutonomous = isAutonomous;

        dashboard = FtcDashboard.getInstance();
        clock = NanoClock.system();
        mode = Mode.IDLE;

        // Initialize autonomous specific stuff
        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID);
        motorPowers = new double[]{0.0, 0.0, 0.0, 0.0};

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        setLocalizer(new Odometry(hardwareMap));
    }

    public void turn(double angle) {
        double heading = getPoseEstimate().getHeading();
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
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
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void setMotorPowersFromGamepad(Gamepad gg, double scale) {
        MecanumUtil.Motion motion = MecanumUtil.joystickToMotion(gg.left_stick_x, gg.left_stick_y,
                gg.right_stick_x, gg.right_stick_y);
        if (fieldCentric) {
            motion = motion.toFieldCentricMotion(getPoseEstimate().getHeading());
        }
        MecanumUtil.Wheels wh = MecanumUtil.motionToWheels(motion).scaleWheelPower(scale);
        motorPowers[0] = wh.frontLeft;
        motorPowers[1] = wh.backLeft;
        motorPowers[2] = wh.backRight;
        motorPowers[3] = wh.frontRight;
    }

    public double[] getMotorPower() {
        return motorPowers;
    }

    @Override
    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();
        autonomousEndPose = currentPose;

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        switch (mode) {
            case IDLE:
                setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                double correction = turnController.update(currentPose.getHeading(), targetOmega);

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
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());

                fieldOverlay.setStroke("#F44336");
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

//        dashboard.sendTelemetryPacket(packet);
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

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    public List<Integer> getEncoders() {
        List<Integer> wheelPositions = new ArrayList<>();
        for (DcMotor motor : motors) {
            wheelPositions.add(motor.getCurrentPosition());
        }
        return wheelPositions;
    }

    public PIDFCoefficients getPIDFCoefficients(DcMotor.RunMode runMode) {
        return leftFront.getPIDFCoefficients(runMode);
    }

    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.p, coefficients.i, coefficients.d, coefficients.f
            ));
        }
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            robot.sleep(0.05);
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }
}
