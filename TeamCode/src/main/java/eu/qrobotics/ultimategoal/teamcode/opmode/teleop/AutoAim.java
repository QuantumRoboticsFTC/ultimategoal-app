package eu.qrobotics.ultimategoal.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import eu.qrobotics.ultimategoal.teamcode.subsystems.Buffer;
import eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Outtake;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Robot;

import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.AUTOAIM_POWERSHOT_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.AUTOAIM_POWERSHOT_VEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.PARK_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.PARK_VEL_CONSTRAINT;

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


    public static Trajectory makePowershotLaunchTrajectory(Pose2d robotPose, Robot robot) {
        return new TrajectoryBuilder(robotPose, Math.toRadians(-90), AUTOAIM_POWERSHOT_VEL_CONSTRAINT, AUTOAIM_POWERSHOT_ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(-10, -9), Math.toRadians(-90), PARK_VEL_CONSTRAINT, PARK_ACCEL_CONSTRAINT)
                .addDisplacementMarker(() -> robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE)
                .lineToConstantHeading(new Vector2d(-10, -16))
                .addDisplacementMarker(() -> robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE)
                .lineToConstantHeading(new Vector2d(-10, -23))
                .addDisplacementMarker(() -> robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE)
                .lineToConstantHeading(new Vector2d(-10, -26))
                .build();
    }
}
