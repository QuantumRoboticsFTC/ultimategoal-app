package eu.qrobotics.ultimategoal.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Outtake;

public class AutoAim {
    public static Polygon OPTIMAL_LAUNCH_AREA = new Polygon(new Vector2d[]{new Vector2d(-4, -48), new Vector2d(2, -40), new Vector2d(-4, -20), new Vector2d(-36, -20), new Vector2d(-36, -48)});

    public static Trajectory makeTowerLaunchTrajectory(Pose2d robotPose) {
        Vector2d targetLocation = new Vector2d(-12, -12);
        /* if(OPTIMAL_LAUNCH_AREA.isInsidePolygon(robotPose.vec())) {
            targetLocation = robotPose.vec().plus(new Vector2d(0.5, 0.5));
        }
        else {
            targetLocation = OPTIMAL_LAUNCH_AREA.closestPointOnPolygon(robotPose.vec());
        }*/

        double targetAngle = Outtake.TOWER_GOAL_POS.minus(targetLocation).angle() - Math.toRadians(1);

        return new TrajectoryBuilder(robotPose, DriveConstants.AUTOAIM_VEL_CONSTRAINT, DriveConstants.AUTOAIM_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(targetLocation, targetAngle))
                .build();
    }


    public static Trajectory makePowershotLaunchTrajectory(Pose2d robotPose, int powerShot) {
        if(powerShot == 0) {
            return new TrajectoryBuilder(robotPose, DriveConstants.AUTOAIM_VEL_CONSTRAINT, DriveConstants.AUTOAIM_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-0.07, -14.86, Math.toRadians(353.5)))
                    .build();
        }
        else if(powerShot == 1) {
            return new TrajectoryBuilder(robotPose, DriveConstants.AUTOAIM_VEL_CONSTRAINT, DriveConstants.AUTOAIM_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(1.15, -2.34, Math.toRadians(350.1)))
                    .build();
        }
        else {
            return new TrajectoryBuilder(robotPose, DriveConstants.AUTOAIM_VEL_CONSTRAINT, DriveConstants.AUTOAIM_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(-0.07, -0.86, Math.toRadians(353.5)))
                    .build();
        }
    }
}
