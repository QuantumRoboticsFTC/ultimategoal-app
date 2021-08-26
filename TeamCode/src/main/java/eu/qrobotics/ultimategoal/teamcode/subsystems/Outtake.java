package eu.qrobotics.ultimategoal.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Outtake implements Subsystem {
    private static final Vector2d OUTTAKE_ROBOT_POS = new Vector2d(0, -5.5);
    private static final double DEFAULT_IDLE_RPM = 3500;
    public static final double MAX_RPM = 4000;
    private OuttakeMode lastOuttakeMode;

    public enum OuttakeMode {
        ON,
        IDLE,
        OFF,
    }

    public enum TurretMode {
        ON,
        FORCED,
    }

    public enum OuttakeTarget {
        HIGH_GOAL(new Vector2d(72, -36), new Vector2d(72, 36)),
        MID_GOAL(new Vector2d(72, 36), new Vector2d(72, -36)),
        POWER_SHOT_1(new Vector2d(72, -20), new Vector2d(72, 18)),
        POWER_SHOT_2(new Vector2d(72, -12.5), new Vector2d(72, 10.5)),
        POWER_SHOT_3(new Vector2d(72, -5), new Vector2d(72, 3));

        Vector2d redPosition;
        Vector2d bluePosition;

        OuttakeTarget(Vector2d redPosition, Vector2d bluePosition) {
            this.redPosition = redPosition;
            this.bluePosition = bluePosition;
        }

        public Vector2d getPosition() {
            return RED_ALLIANCE ? redPosition : bluePosition;
        }
    }

    private static double TICKS_PER_REV = 28;
    private static double READY_RPM_THRESHOLD = 75;

    public static double TURRET_FORCE_RED_ANGLE = 0.1;
    public static double TURRET_FORCE_BLUE_ANGLE = 0.05;
    public static double MIN_TURRET_POSITION = 0.18;
    public static double MAX_TURRET_POSITION = 0.8;
    public static double MIN_TURRET_ANGLE = Math.toRadians(-9.5);
    public static double MAX_TURRET_ANGLE = Math.toRadians(18.4);

    public static boolean RED_ALLIANCE = true;
//    public static PIDFCoefficients OUTTAKE_PIDF_COEFFICIENTS = new PIDFCoefficients(600, 10, 100, 15);
    public static PIDFCoefficients OUTTAKE_PIDF_COEFFICIENTS = new PIDFCoefficients(300, 8, 10, 13);
    public static PIDFCoefficients PREV_OUTTAKE_PIDF_COEFFICIENTS = new PIDFCoefficients(OUTTAKE_PIDF_COEFFICIENTS);

    private static class Coefficients {
        public double kA, kB, kC, kD;
        public Coefficients(double kA, double kB, double kC, double kD) { this.kA = kA; this.kB = kB; this.kC = kC; this.kD = kD; }
        public double apply(double val) { return kA * val * val * val + kB * val * val + kC * val + kD; }
    }

    public static Coefficients AUTO_HIGH_GOAL_COEFFICIENTS = new Coefficients(-0.003196226, 1.076639, -118.6705, 7239.618);
    public static Coefficients HIGH_GOAL_COEFFICIENTS = new Coefficients(-0.003196226, 1.076639, -118.6705, 7239.618);
    public static Coefficients MID_GOAL_COEFFICIENTS = new Coefficients(-0.02263642, 5.838198, -491.6129, 15947.71);
    public static Coefficients POWER_SHOT_COEFFICIENTS = new Coefficients(-0.02263642, 5.838198, -491.6129, 16147.71);

    public OuttakeMode outtakeMode;
    public OuttakeTarget outtakeTarget;
    public TurretMode turretMode;
    public double rpmOffset;
    public double overrideRPM;

    private DcMotorEx outtakeMotor;
    private boolean isAutonomous;
    private Servo turretServo;

    private Robot robot;

    public Outtake(HardwareMap hardwareMap, Robot robot, boolean isAutonomous) {
        this.robot = robot;
        this.isAutonomous = isAutonomous;

        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, OUTTAKE_PIDF_COEFFICIENTS);

        turretServo = hardwareMap.get(Servo.class, "turretServo");

        outtakeMode = OuttakeMode.OFF;
        turretMode = TurretMode.ON;
        outtakeTarget = OuttakeTarget.HIGH_GOAL;
        overrideRPM = 0;
    }

    public static boolean IS_DISABLED = false;

    private double lastTurretPosition = -1;
    private double lastTicks = DEFAULT_IDLE_RPM * TICKS_PER_REV / 60;
    public Double robotHeadingOverride = null;

    @Override
    public void update() {
        if(IS_DISABLED) return;
//        if(PREV_OUTTAKE_PIDF_COEFFICIENTS.p != OUTTAKE_PIDF_COEFFICIENTS.p ||
//                PREV_OUTTAKE_PIDF_COEFFICIENTS.i != OUTTAKE_PIDF_COEFFICIENTS.i ||
//                PREV_OUTTAKE_PIDF_COEFFICIENTS.d != OUTTAKE_PIDF_COEFFICIENTS.d ||
//                PREV_OUTTAKE_PIDF_COEFFICIENTS.f != OUTTAKE_PIDF_COEFFICIENTS.f)
//        {
//            outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, OUTTAKE_PIDF_COEFFICIENTS);
//            PREV_OUTTAKE_PIDF_COEFFICIENTS = new PIDFCoefficients(OUTTAKE_PIDF_COEFFICIENTS);
//        }
        switch (outtakeMode) {
            case OFF:
                if(lastOuttakeMode != OuttakeMode.OFF) {
                    outtakeMotor.setPower(0);
                }
                break;
            case IDLE:
                if(lastOuttakeMode != OuttakeMode.IDLE) {
                    outtakeMotor.setVelocity(lastTicks);
                }
                break;
            case ON:
                outtakeMotor.setVelocity(lastTicks = getTargetTicks());
                break;
        }
        lastOuttakeMode = outtakeMode;
        switch (turretMode) {
            case FORCED:
                turretServo.setPosition(angleToTurretPosition(RED_ALLIANCE ? TURRET_FORCE_RED_ANGLE : TURRET_FORCE_BLUE_ANGLE));
                break;
            case ON:
                double targetTurretAngle = getTargetTurretAngle();
                if(-Math.PI / 2 < targetTurretAngle && targetTurretAngle < Math.PI / 2) {
                    double targetPosition = angleToTurretPosition(targetTurretAngle);
                    if(Math.abs(lastTurretPosition - targetPosition) > 0.01) {
                        turretServo.setPosition(targetPosition);
                        lastTurretPosition = targetPosition;
                    }
                }
                break;
        }
    }

    public boolean isReady() {
        return Math.abs(getCurrentRPM() - getTargetRPM()) < READY_RPM_THRESHOLD && isTurretInRange();
    }

    double getTargetTicks() {
        return Math.round(getTargetRPM() / 60 * TICKS_PER_REV);
    }

    public double getDistance() {
        Vector2d currentPos = robot.drive.getPoseEstimate().vec();
        if(currentPos.getY() > 0 )
            currentPos = new Vector2d(currentPos.getX(), 0);
        return currentPos.minus(outtakeTarget.getPosition()).norm();
    }

    public double getTargetRPM() {
        if (overrideRPM > 0) {
            return overrideRPM;
        }
        if(outtakeTarget == OuttakeTarget.POWER_SHOT_1 || outtakeTarget == OuttakeTarget.POWER_SHOT_2 || outtakeTarget == OuttakeTarget.POWER_SHOT_3) {
            return Math.min(POWER_SHOT_COEFFICIENTS.apply(getDistance()) + rpmOffset, MAX_RPM);
        }
        if(outtakeTarget == OuttakeTarget.MID_GOAL) {
            return Math.min(MID_GOAL_COEFFICIENTS.apply(getDistance()) + rpmOffset, MAX_RPM);
        }
        if(isAutonomous) {
            return Math.min(AUTO_HIGH_GOAL_COEFFICIENTS.apply(getDistance()) + rpmOffset, MAX_RPM);
        }
        return Math.min(HIGH_GOAL_COEFFICIENTS.apply(getDistance()) + rpmOffset, MAX_RPM);
    }

    public double getCurrentRPM() {
        return outtakeMotor.getVelocity() / TICKS_PER_REV * 60;
    }

    public double getTargetTurretAngle() {
        Vector2d targetPosition = outtakeTarget.getPosition();
        Vector2d outtakePosition = robot.drive.getPoseEstimate().vec().plus(OUTTAKE_ROBOT_POS.rotated(robot.drive.getPoseEstimate().getHeading()));
        double robotHeading = robotHeadingOverride != null ? robotHeadingOverride : robot.drive.getPoseEstimate().getHeading();
        double targetAngle = targetPosition.minus(outtakePosition).angle() - robotHeading;
        while(targetAngle > Math.PI)
            targetAngle -= 2 * Math.PI;
        while(targetAngle < -Math.PI)
            targetAngle += 2 * Math.PI;
        return targetAngle;
    }

    public double getTargetTurretPosition() {
        return angleToTurretPosition(getTargetTurretAngle());
    }

    private double angleToTurretPosition(double angle) {
        double position = Range.scale(angle, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE, MIN_TURRET_POSITION, MAX_TURRET_POSITION);
        return Range.clip(position, MIN_TURRET_POSITION, MAX_TURRET_POSITION);
    }

    public boolean isTurretInRange() {
        if(turretMode == TurretMode.FORCED)
            return true;
        double targetAngle = getTargetTurretAngle();
        return MIN_TURRET_ANGLE <= targetAngle && targetAngle <= MAX_TURRET_ANGLE;
    }
}
