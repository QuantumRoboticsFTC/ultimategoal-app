package eu.qrobotics.ultimategoal.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Arrays;

@Config
public class DriveConstants {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(13, 0.4, 1.5);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(2.1, 0.6, 1);

//    public static PIDCoefficients TRANSLATIONAL_PID_TELEOP = new PIDCoefficients(15, 0.5, 2);
//    public static PIDCoefficients HEADING_PID_TELEOP = new PIDCoefficients(15, 2, 0.5);
//    public static PIDCoefficients TRANSLATIONAL_PID_TELEOP = new PIDCoefficients(8, 4, 3.5);
//    public static PIDCoefficients HEADING_PID_TELEOP = new PIDCoefficients(7, 2, 3);
    public static PIDCoefficients TRANSLATIONAL_PID_TELEOP = new PIDCoefficients(13, 0.4, 1.5);
    public static PIDCoefficients HEADING_PID_TELEOP = new PIDCoefficients(2.1, 0.6, 1);

    public static double LATERAL_MULTIPLIER = 1.7;
    public static double WHEEL_BASE = 12.3;

    public static double WHEEL_RADIUS = 1.8897; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 15.4; // in

    public static double MAX_ANG_VEL = Math.toRadians(360);
    public static double MAX_ANG_ACCEL = Math.toRadians(90);
    public static double MAX_VEL = 60;
    public static double MAX_ACCEL = 40;

    public static TrajectoryVelocityConstraint AUTOAIM_VEL_CONSTRAINT = new MecanumVelocityConstraint(80,TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);
    public static TrajectoryAccelerationConstraint AUTOAIM_ACCEL_CONSTRAINT = new ProfileAccelerationConstraint(200);

    public static TrajectoryVelocityConstraint BASE_VEL_CONSTRAINT = new MecanumVelocityConstraint(30, TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);
    public static TrajectoryAccelerationConstraint BASE_ACCEL_CONSTRAINT = new ProfileAccelerationConstraint(60);

    public static TrajectoryVelocityConstraint MEDIUM_VEL_CONSTRAINT = new MecanumVelocityConstraint(35,TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);
    public static TrajectoryAccelerationConstraint MEDIUM_ACCEL_CONSTRAINT = new ProfileAccelerationConstraint(75);

    public static TrajectoryVelocityConstraint FAST_VEL_CONSTRAINT = new MecanumVelocityConstraint(40,TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);
    public static TrajectoryAccelerationConstraint FAST_ACCEL_CONSTRAINT = new ProfileAccelerationConstraint(90);

    public static TrajectoryVelocityConstraint SLOW_VEL_CONSTRAINT = new MecanumVelocityConstraint(2.5, TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);
    public static TrajectoryAccelerationConstraint SLOW_ACCEL_CONSTRAINT = new ProfileAccelerationConstraint(20);

    public static TrajectoryVelocityConstraint MEDIUM_SLOW_VEL_CONSTRAINT = new MecanumVelocityConstraint(5, TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);
    public static TrajectoryAccelerationConstraint MEDIUM_SLOW_ACCEL_CONSTRAINT = new ProfileAccelerationConstraint(20);

    public static TrajectoryVelocityConstraint PARK_VEL_CONSTRAINT = new MecanumVelocityConstraint(20, TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);
    public static TrajectoryAccelerationConstraint PARK_ACCEL_CONSTRAINT = new ProfileAccelerationConstraint(50);

    public static TrajectoryVelocityConstraint POWERSHOT_VEL_CONSTRAINT = new MecanumVelocityConstraint(45, TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);
    public static TrajectoryAccelerationConstraint POWERSHOT_ACCEL_CONSTRAINT = new ProfileAccelerationConstraint(60);

    public static TrajectoryVelocityConstraint AUTOAIM_POWERSHOT_VEL_CONSTRAINT = new MecanumVelocityConstraint(10, TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);
    public static TrajectoryAccelerationConstraint AUTOAIM_POWERSHOT_ACCEL_CONSTRAINT = new ProfileAccelerationConstraint(20);


    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 312;

    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double kV = 0.0170;
    public static double kA = 0.0035;
    public static double kStatic = 0;

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
