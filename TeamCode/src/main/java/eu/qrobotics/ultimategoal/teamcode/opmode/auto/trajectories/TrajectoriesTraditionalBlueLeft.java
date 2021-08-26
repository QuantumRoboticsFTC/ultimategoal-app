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

public class TrajectoriesTraditionalBlueLeft {
    public static Pose2d START_POSE = new Pose2d(-63, 57, Math.toRadians(0));

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
                .splineToSplineHeading(new Pose2d(-10, 57, Math.toRadians(-10)), 0)
                .build());

        // place first wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(0, 56, Math.toRadians(-135)), Math.toRadians(0))
                .build());

        // make space
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-24, 54, Math.toRadians(90)), Math.toRadians(180))
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-4, 56), Math.toRadians(0))
                .build());

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesB() {
        List<Trajectory> trajectories = new ArrayList<>();

        // high goal
        trajectories.add(makeTrajectoryBuilder(trajectories, 0, POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(-10, 57, Math.toRadians(-10)), 0)
                .build());

        // collect 1 Ring
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-170))
                .splineToConstantHeading(new Vector2d(-40, 36), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-20, 36), 0)
                .build());

        // place first wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(15, 58, Math.toRadians(90)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(39, 53), Math.toRadians(0))
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(12, 57, Math.toRadians(180)), Math.toRadians(180))
                .build());

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesC() {
        List<Trajectory> trajectories = new ArrayList<>();

        // high goal
        trajectories.add(makeTrajectoryBuilder(trajectories, 0, POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(-10, 57, Math.toRadians(-10)), 0)
                .build());

        // collect 2 Rings
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-170))
                .splineToConstantHeading(new Vector2d(-40, 36), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-20, 36), 0)
                .build());

        // collect another 2 Rings
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-10, 36), 0)
                .build());

        // place first wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(12, 58, Math.toRadians(-125)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(42, 58), Math.toRadians(0))
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(12, 57, Math.toRadians(180)), Math.toRadians(180))
                .build());

        return trajectories;
    }
}
