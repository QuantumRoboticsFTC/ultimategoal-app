package eu.qrobotics.ultimategoal.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Outtake implements Subsystem {
    public enum OuttakeMode {
        ON,
        OFF,
    }

    public enum OuttakeTarget {
        HIGH_GOAL,
        POWER_SHOT,
    }

    private static double TICKS_PER_REV = 28;
    private static double READY_RPM_THRESHOLD = 100;

    public static Vector2d TOWER_GOAL_POS = new Vector2d(72, -36);
    public static Vector2d POWERSHOT_POS = new Vector2d(72, -10);
    public static PIDFCoefficients OUTTAKE_PIDF_COEFFICIENTS = new PIDFCoefficients(95, 0, 40, 15);

    private static class Coefficients {
        public double kA, kB, kC, kD;
        public Coefficients(double kA, double kB, double kC, double kD) { this.kA = kA; this.kB = kB; this.kC = kC; this.kD = kD; }
        public double apply(double val) { return kA * val * val * val + kB * val * val + kC * val + kD; }
    }

    public static Coefficients HIGH_GOAL_COEFFICIENTS = new Coefficients(0.002944254, -0.7317014, 57.38531, 1910.281);
    public static Coefficients POWER_SHOT_COEFFICIENTS = new Coefficients(0.002944254, -0.7317014, 57.38531, 1670.281);

    public OuttakeMode outtakeMode;
    public OuttakeTarget outtakeTarget;
    public double overrideRPM;

    private DcMotorEx outtakeMotor;

    private Robot robot;

    public Outtake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, OUTTAKE_PIDF_COEFFICIENTS);

        outtakeMode = OuttakeMode.OFF;
        outtakeTarget = OuttakeTarget.HIGH_GOAL;
        overrideRPM = 0;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if(IS_DISABLED) return;
        switch (outtakeMode) {
            case OFF:
                outtakeMotor.setPower(0);
                break;
            case ON:
                outtakeMotor.setVelocity(getTargetTicks());
                break;
        }
    }

    public boolean isReady() {
        return Math.abs(getCurrentRPM() - getTargetRPM()) < READY_RPM_THRESHOLD;
    }

    double getTargetTicks() {
        return getTargetRPM() * TICKS_PER_REV / 60;
    }

    public double getDistance() {
        return robot.drive.getPoseEstimate().vec().minus(TOWER_GOAL_POS).norm();
    }

    public double getTargetRPM() {
        if (overrideRPM > 0) {
            return overrideRPM;
        }
        if(outtakeTarget == OuttakeTarget.POWER_SHOT) {
            return POWER_SHOT_COEFFICIENTS.apply(getDistance());
        }
        return HIGH_GOAL_COEFFICIENTS.apply(getDistance());
    }

    public double getCurrentRPM() {
        return outtakeMotor.getVelocity() / TICKS_PER_REV * 60;
    }
}
