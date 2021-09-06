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
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.FASTER_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.FASTER_VEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.FAST_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.FAST_VEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.POWERSHOT_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.POWERSHOT_VEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.SLOW_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.SLOW_VEL_CONSTRAINT;

public class TrajectoriesTraditionalBlueLeft_Solo {
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

        // get 2nd wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(-35, 16, Math.toRadians(0)), Math.toRadians(180))
                .build());

        // place 2nd wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(12, 45, Math.toRadians(-90)), Math.toRadians(90))
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(10, 38, Math.toRadians(180)))
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
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(23, 48, Math.toRadians(180)), Math.toRadians(0))
                .build());

        // get 2nd wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(-35, 16, Math.toRadians(0)), Math.toRadians(180))
                .build());

        // place 2nd wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(19, 40, Math.toRadians(180)), Math.toRadians(0))
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(12, 34))
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
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), FASTER_VEL_CONSTRAINT, FASTER_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(60, 44, Math.toRadians(-90)), Math.toRadians(0))
                .build());

        // get 2nd wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), FASTER_VEL_CONSTRAINT, FASTER_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(-32, 16, Math.toRadians(0)), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-35, 16), SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                .build());

        // place 2nd wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), FASTER_VEL_CONSTRAINT, FASTER_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(54, 44, Math.toRadians(-90)), Math.toRadians(0))
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(10, 36))
                .build());

        return trajectories;
    }
}
