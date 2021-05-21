package eu.qrobotics.ultimategoal.teamcode.opmode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.ultimategoal.teamcode.subsystems.Robot;

import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.BASE_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.BASE_VEL_CONSTRAINT;

@TeleOp
@Disabled
public class RobotBullyTuner extends OpMode {
    Robot robot;
    Trajectory trajectory1;
    Trajectory trajectory2;

    @Override
    public void init() {
        robot = new Robot(this, false);
        trajectory1 = new TrajectoryBuilder(new Pose2d(0,0,0), Math.toRadians(0), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .forward(60)
                .build();
        trajectory2 = new TrajectoryBuilder(new Pose2d(60,0,0), Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .back(60)
                .build();
    }

    @Override
    public void start() {
        robot.start();
    }

    boolean direction = true;

    @Override
    public void loop() {
        if(!robot.drive.isBusy()) {
            if (direction) {
                robot.drive.followTrajectory(trajectory1);
            }
            else {
                robot.drive.followTrajectory(trajectory2);
            }
            direction = !direction;
        }
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
