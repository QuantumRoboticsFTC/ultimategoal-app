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
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.PARK_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.PARK_VEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.POWERSHOT_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.POWERSHOT_VEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.SLOW_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.SLOW_VEL_CONSTRAINT;

public class TrajectoriesTraditionalRedRight {
    public static Pose2d START_POSE = new Pose2d(-63, -57, Math.toRadians(0));

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
                .splineToSplineHeading(new Pose2d(-20, -53, Math.toRadians(0)), 0)
                .build());

        // place first wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(0, -60, Math.toRadians(-180)), Math.toRadians(0))
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30, -60), Math.toRadians(180))
                .build());
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(10, -36), Math.toRadians(0))
                .build());

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesB() {
        List<Trajectory> trajectories = new ArrayList<>();

        // high goal
        trajectories.add(makeTrajectoryBuilder(trajectories, 0, POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(-20, -53, Math.toRadians(0)), 0)
                .build());

        // place first wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(40, -55, Math.toRadians(-90)))
                .build());

        // collect 1 Ring
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-10.5, -40, Math.toRadians(180)), Math.toRadians(-180), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-20, -40))
                .splineToSplineHeading(new Pose2d(-20, -53, Math.toRadians(0)), 0)
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-180), PARK_VEL_CONSTRAINT, PARK_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(12, -60))
                .build());

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesC() {
        List<Trajectory> trajectories = new ArrayList<>();

        // high goal
        trajectories.add(makeTrajectoryBuilder(trajectories, 0, POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(-20, -53, Math.toRadians(0)), 0)
                .build());

        // 3 rings high goal
        // break stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-170))
                .splineToSplineHeading(new Pose2d(-50, -47, Math.toRadians(-3)), Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(-32.5, -35), Math.toRadians(0))
                .build());
        // intake
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(-25, -35), Math.toRadians(0))
                .build());

        // 1 ring high goal
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0)/*, SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT*/)
                .lineToSplineHeading(new Pose2d(-10, -35, Math.toRadians(-3)))
                .build());

        // place first wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(10, -60, Math.toRadians(180)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(60, -60, Math.toRadians(180)))
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-180), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(12, -60, Math.toRadians(180)))
                .build());

        return trajectories;
    }
}
