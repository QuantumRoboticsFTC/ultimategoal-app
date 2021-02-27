package eu.qrobotics.ultimategoal.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConstants {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0.3, 2.6, 2.6);
    public static PIDCoefficients LATERAL_PID = new PIDCoefficients(0.3, 2.6, 2.6);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0.35);

    public static double LATERAL_MULTIPLIER = 1;
    public static double WHEEL_BASE = 13.25;

    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            60.0, 40.0, 0.0,
            Math.toRadians(360.0), Math.toRadians(120.0), 0.0
    );

    public static DriveConstraints SLOW_CONSTRAINTS = new DriveConstraints(
            30.0, 20.0, 0.0,
            Math.toRadians(60.0), Math.toRadians(30.0), 0.0
    );

    public static final double TICKS_PER_REV = 383.6;
    public static final double MAX_RPM = 435;

    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double WHEEL_RADIUS = 1.9685; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 15.89; // in

    public static double kV = 0.01027;
    public static double kA = 0.00004;
    public static double kStatic = 0.08229;


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
