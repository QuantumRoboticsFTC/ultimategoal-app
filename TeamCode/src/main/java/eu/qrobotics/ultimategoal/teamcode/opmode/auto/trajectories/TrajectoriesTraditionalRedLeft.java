package eu.qrobotics.ultimategoal.teamcode.opmode.auto.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.ArrayList;
import java.util.List;

import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.BASE_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.BASE_VEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.FAST_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.FAST_VEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.MEDIUM_SLOW_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.MEDIUM_SLOW_VEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.PARK_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.PARK_VEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.POWERSHOT_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.POWERSHOT_VEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.SLOW_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.SLOW_VEL_CONSTRAINT;

public class TrajectoriesTraditionalRedLeft {
    public static Pose2d START_POSE = new Pose2d(-63, -18, Math.toRadians(0));

    private static Pose2d getTrajectorySequenceEndPose(List<Trajectory> trajectories) {
        if(trajectories.size() == 0)
            return START_POSE;
        return trajectories.get(trajectories.size() - 1).end();
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startTangent, TrajectoryVelocityConstraint velocityConstraint, TrajectoryAccelerationConstraint accelerationConstraint) {
        return new TrajectoryBuilder(getTrajectorySequenceEndPose(trajectories), startTangent, velocityConstraint, accelerationConstraint);
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startTangent) {
        return new TrajectoryBuilder(getTrajectorySequenceEndPose(trajectories), startTangent, BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT);
    }

    public static List<Trajectory> getTrajectoriesA() {
        List<Trajectory> trajectories = new ArrayList<>();

        // power shots
        trajectories.add(makeTrajectoryBuilder(trajectories, 0, POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(0, -12, Math.toRadians(0)), 0)
                .build());

        // place first wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(24, -45, Math.toRadians(90)), Math.toRadians(-90))
                .build());

        // bounce backs
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(60, -45, Math.toRadians(80)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(60, -20, Math.toRadians(80)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-10, -12, Math.toRadians(-10)), Math.toRadians(180))
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(10, -15, Math.toRadians(180)), Math.toRadians(0))
                .build());

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesB() {
        List<Trajectory> trajectories = new ArrayList<>();

        // power shots
        trajectories.add(makeTrajectoryBuilder(trajectories, 0, POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(0, -12, Math.toRadians(0)), 0)
                .build());

        // place first wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(48, -18, Math.toRadians(90)))
                .build());

        // bounce backs
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(60, -12, Math.toRadians(-80)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(60, -36, Math.toRadians(-80)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(60, -30, Math.toRadians(-80)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-10, -12, Math.toRadians(-10)), Math.toRadians(180))
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), PARK_VEL_CONSTRAINT, PARK_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(10, -15, Math.toRadians(180)), Math.toRadians(0))
                .build());

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesC() {
        List<Trajectory> trajectories = new ArrayList<>();

        // power shots
        trajectories.add(makeTrajectoryBuilder(trajectories, 0, POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(0, -12, Math.toRadians(0)), 0)
                .build());

        // bounce backs & wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(50, -12, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(60, -12, Math.toRadians(-80)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(60, -32, Math.toRadians(-80)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(57, -42, Math.toRadians(135)), Math.toRadians(-90))
                .build());

        // shoot bounce backs
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(40, -12), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-10, -12, Math.toRadians(-10)), Math.toRadians(180))
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(10, -15, Math.toRadians(180)), Math.toRadians(0))
                .build());

        return trajectories;
    }
}
