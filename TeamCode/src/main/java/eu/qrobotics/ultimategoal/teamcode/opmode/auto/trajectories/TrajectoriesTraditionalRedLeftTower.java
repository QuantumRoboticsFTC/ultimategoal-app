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
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.POWERSHOT_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.POWERSHOT_VEL_CONSTRAINT;

public class TrajectoriesTraditionalRedLeftTower {
    public static Pose2d START_POSE = new Pose2d(-63, -17, Math.toRadians(0));

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

        // high goal
        trajectories.add(makeTrajectoryBuilder(trajectories, 0, POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(-4, -12, Math.toRadians(-5)), 0)
                .build());

        // place first wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(24, -45, Math.toRadians(90)), Math.toRadians(-90))
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(10, -15, Math.toRadians(180)), Math.toRadians(180))
                .build());

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesB() {
        List<Trajectory> trajectories = new ArrayList<>();

        // high goal
        trajectories.add(makeTrajectoryBuilder(trajectories, 0, POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(-4, -12, Math.toRadians(-5)), 0)
                .build());

        // place first wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(50, -20, Math.toRadians(90)))
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(10, -15, Math.toRadians(180)), Math.toRadians(180))
                .build());

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesC() {
        List<Trajectory> trajectories = new ArrayList<>();

        // high goal
        trajectories.add(makeTrajectoryBuilder(trajectories, 0, POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(-4, -12, Math.toRadians(-5)), 0)
                .build());

        // wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(56, -48, Math.toRadians(135)), Math.toRadians(-90))
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(10, -15, Math.toRadians(180)), Math.toRadians(180))
                .build());

        return trajectories;
    }
}
