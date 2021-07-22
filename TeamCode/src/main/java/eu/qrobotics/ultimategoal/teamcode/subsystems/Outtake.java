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
    private OuttakeMode lastOuttakeMode;

    public enum OuttakeMode {
        ON,
        IDLE,
        OFF,
    }

    public enum TurretMode {
        ON,
        OFF,
    }

    public enum OuttakeTarget {
        HIGH_GOAL(new Vector2d(72, -36), new Vector2d(72, 36)),
        POWER_SHOT_1(new Vector2d(72, -20), new Vector2d(72, 22)),
        POWER_SHOT_2(new Vector2d(72, -12.5), new Vector2d(72, 12)),
        POWER_SHOT_3(new Vector2d(72, -5), new Vector2d(72, 2));

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
    private static double READY_RPM_THRESHOLD = 100;

    public static double TURRET_IDLE_POSITION = 0.5;
    public static double MIN_TURRET_POSITION = 0.15;
    public static double MAX_TURRET_POSITION = 0.8;
    public static double MIN_TURRET_ANGLE = Math.toRadians(-9.5);
    public static double MAX_TURRET_ANGLE = Math.toRadians(18.4);

    public static boolean RED_ALLIANCE = true;
    public static PIDFCoefficients OUTTAKE_PIDF_COEFFICIENTS = new PIDFCoefficients(600, 10, 100, 15);
    public static PIDFCoefficients PREV_OUTTAKE_PIDF_COEFFICIENTS = new PIDFCoefficients(OUTTAKE_PIDF_COEFFICIENTS);

    private static class Coefficients {
        public double kA, kB, kC, kD;
        public Coefficients(double kA, double kB, double kC, double kD) { this.kA = kA; this.kB = kB; this.kC = kC; this.kD = kD; }
        public double apply(double val) { return kA * val * val * val + kB * val * val + kC * val + kD; }
    }

    public static Coefficients HIGH_GOAL_COEFFICIENTS = new Coefficients(-0.0103352, 3.361013, -352.4242, 14928.33);
    public static Coefficients POWER_SHOT_COEFFICIENTS = new Coefficients(-0.03278689, 8.821494, -780.1676, 25304.94);

    public OuttakeMode outtakeMode;
    public OuttakeTarget outtakeTarget;
    public TurretMode turretMode;
    public double overrideRPM;

    private DcMotorEx outtakeMotor;
    private Servo turretServo;

    private Robot robot;

    public Outtake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

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
            case OFF:
                turretServo.setPosition(TURRET_IDLE_POSITION);
                break;
            case ON:
                double targetTurretAngle = getTargetTurretAngle();
                if(-Math.PI / 2 < targetTurretAngle && targetTurretAngle < Math.PI / 2) {
                    double targetPosition = angleToTurretPosition(targetTurretAngle);
                    if(Math.abs(lastTurretPosition - targetPosition) > 0.02) {
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
        return getTargetRPM() * TICKS_PER_REV / 60;
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
            return POWER_SHOT_COEFFICIENTS.apply(getDistance());
        }
        return HIGH_GOAL_COEFFICIENTS.apply(getDistance());
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
        double targetAngle = getTargetTurretAngle();
        return MIN_TURRET_ANGLE <= targetAngle && targetAngle <= MAX_TURRET_ANGLE;
    }
}
