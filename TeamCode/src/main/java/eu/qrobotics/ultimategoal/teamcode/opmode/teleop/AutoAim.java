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
        Vector2d targetLocation;
        if(OPTIMAL_LAUNCH_AREA.isInsidePolygon(robotPose.vec())) {
            targetLocation = robotPose.vec().plus(new Vector2d(1, 1));
        }
        else {
            targetLocation = OPTIMAL_LAUNCH_AREA.closestPointOnPolygon(robotPose.vec());
        }

        double targetAngle = Outtake.TOWER_GOAL_POS.minus(targetLocation).angle() - Math.toRadians(5);

        return new TrajectoryBuilder(robotPose, DriveConstants.AUTOAIM_VEL_CONSTRAINT, DriveConstants.AUTOAIM_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(targetLocation, targetAngle))
                .build();
    }


    public static Trajectory makePowershotLaunchTrajectory(Pose2d robotPose) {
        return new TrajectoryBuilder(robotPose, DriveConstants.AUTOAIM_VEL_CONSTRAINT, DriveConstants.AUTOAIM_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(-20, -20, Math.toRadians(0)))
                .build();
    }
}
