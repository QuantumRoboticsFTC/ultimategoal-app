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

    private static double TICKS_PER_REV = 28;
    private static double READY_RPM_THRESHOLD = 100;

    public static Vector2d TOWER_GOAL_POS = new Vector2d(72, -36);
    public static Vector2d POWERSHOT_POS = new Vector2d(72, -10);
    public static PIDFCoefficients OUTTAKE_PIDF_COEFFICIENTS = new PIDFCoefficients(100, 0, 17, 15);

    public static double kA = 0.002944254;
    public static double kB = -0.7317014;
    public static double kC = 57.38531;
    public static double kD = 1880.281;

    public OuttakeMode outtakeMode;
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
        return kA * getDistance() * getDistance() * getDistance() + kB * getDistance() * getDistance() + kC * getDistance() + kD;
    }

    public double getCurrentRPM() {
        return outtakeMotor.getVelocity() / TICKS_PER_REV * 60;
    }
}
