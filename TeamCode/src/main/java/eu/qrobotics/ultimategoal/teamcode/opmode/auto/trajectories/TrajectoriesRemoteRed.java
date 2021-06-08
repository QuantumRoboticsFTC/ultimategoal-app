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
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.MEDIUM_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.MEDIUM_VEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.PARK_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.PARK_VEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.POWERSHOT_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.POWERSHOT_VEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.SLOW_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.SLOW_VEL_CONSTRAINT;

public class TrajectoriesRemoteRed {
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

        // powershots
        trajectories.add(makeTrajectoryBuilder(trajectories, 0, POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(-20, -22, Math.toRadians(-8)), 0)
                .build());
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-20, -21.5, Math.toRadians(-4)))
                .build());
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-20, -21, Math.toRadians(0)))
                .build());
        /*
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(-35, -40, Math.toRadians(-5)))
                .build());
         */

        // place first wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-4, -60, Math.toRadians(-180)), Math.toRadians(-90))
                .build());

        // go to second wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(-12, -48), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-30, -53, Math.toRadians(0)), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-37, -53))
                .build());

        // place second wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-23, -50, Math.toRadians(-90)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-8, -50, Math.toRadians(-185)), Math.toRadians(0))
                .build());

        // collect rings
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(48, 10, Math.toRadians(-10)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(55, -5, Math.toRadians(-45)), Math.toRadians(-90), MEDIUM_VEL_CONSTRAINT, MEDIUM_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(55, -46, Math.toRadians(-45)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(24, -35, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-12, -35, Math.toRadians(-6)), Math.toRadians(180))
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), PARK_VEL_CONSTRAINT, PARK_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(0, -30, Math.toRadians(0)), Math.toRadians(180))
                .build());

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesB() {
        List<Trajectory> trajectories = new ArrayList<>();

        // powershots
        trajectories.add(makeTrajectoryBuilder(trajectories, 0, POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(-20, -22, Math.toRadians(-8)), 0)
                .build());
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-20, -21.5, Math.toRadians(-4)))
                .build());
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-20, -21, Math.toRadians(0)))
                .build());

        // place first wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(40, -20, Math.toRadians(90)))
                .build());

        // collect bounce back rings
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(51, 4, Math.toRadians(-10)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(56, -5, Math.toRadians(-45)), Math.toRadians(-90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(56, -45, Math.toRadians(-45)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(40, -54, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(12, -54, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-12, -45, Math.toRadians(-3)), Math.toRadians(180))
                .build());

        // go to second wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(-30, -51), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-38.5, -51))
                .build());

        // collect & shoot ring
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(110))
                .splineToConstantHeading(new Vector2d(-45, -51), Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(-45, -32), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-20.5, -32, Math.toRadians(-9)), Math.toRadians(0))
                .build());

        // place second wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(15, -36, Math.toRadians(-180)))
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-180), PARK_VEL_CONSTRAINT, PARK_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(10, -36))
                .build());

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesC() {
        List<Trajectory> trajectories = new ArrayList<>();

        // powershots
        trajectories.add(makeTrajectoryBuilder(trajectories, 0, POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(-20, -22, Math.toRadians(-8)), 0)
                .build());
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-20, -21.5, Math.toRadians(-4)))
                .build());
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), POWERSHOT_VEL_CONSTRAINT, POWERSHOT_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-20, -21, Math.toRadians(0)))
                .build());

        // collect bounce back rings and place wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(48, 10, Math.toRadians(-10)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(55, -5, Math.toRadians(-45)), Math.toRadians(-90), MEDIUM_VEL_CONSTRAINT, MEDIUM_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(55, -46, Math.toRadians(-45)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(50, -50, Math.toRadians(180)), Math.toRadians(180))
                .build());

        // shoot rings
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), MEDIUM_VEL_CONSTRAINT, MEDIUM_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(-10, -48, Math.toRadians(2)), Math.toRadians(180))
                .build());

        // go to second wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(-30, -51), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-34.5, -51))
                .build());

        // 3 rings high goal
        // break stack
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(110))
                .splineToConstantHeading(new Vector2d(-45, -51), Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(-45, -32), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-32.5, -32, Math.toRadians(-9)), Math.toRadians(0))
                .build());

        // intake
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(-23, -34), Math.toRadians(0))
                .build());

        // 1 ring high goal
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0)/*, SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT*/)
                .lineToSplineHeading(new Pose2d(-10, -34, Math.toRadians(-9)))
                .build());

        // place second wobble
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .splineToSplineHeading(new Pose2d(20, -50, Math.toRadians(90)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(44, -54, Math.toRadians(180)), Math.toRadians(0))
                .build());

        // park
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-180), FAST_VEL_CONSTRAINT, FAST_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(20, -54))
                .build());

        return trajectories;
    }
}
