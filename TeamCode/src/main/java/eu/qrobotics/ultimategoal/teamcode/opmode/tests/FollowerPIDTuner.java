package eu.qrobotics.ultimategoal.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.ultimategoal.teamcode.subsystems.Robot;
import eu.qrobotics.ultimategoal.teamcode.util.StickyGamepad;

import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.BASE_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.BASE_VEL_CONSTRAINT;

@TeleOp(group = "Test")
public class FollowerPIDTuner extends OpMode {
    private Robot robot;
    private MultipleTelemetry telemetry;

    private StickyGamepad stickyGamepad1;

    @Override
    public void init() {
        stickyGamepad1 = new StickyGamepad(gamepad1);
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(this, false);
        telemetry.log().add("Ready!");
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        stickyGamepad1.update();

        if(stickyGamepad1.a) {
            robot.drive.followTrajectory(new TrajectoryBuilder(robot.drive.getPoseEstimate(), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-20, -20, Math.toRadians(0)))
                .build()
            );
        }
        if(stickyGamepad1.b) {
            robot.drive.followTrajectory(new TrajectoryBuilder(robot.drive.getPoseEstimate(), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(20, -20, Math.toRadians(90)))
                .build()
            );
        }
        if(stickyGamepad1.x) {
            robot.drive.followTrajectory(new TrajectoryBuilder(robot.drive.getPoseEstimate(), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-20, 20, Math.toRadians(180)))
                .build()
            );
        }
        if(stickyGamepad1.y) {
            robot.drive.followTrajectory(new TrajectoryBuilder(robot.drive.getPoseEstimate(), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(20, 20, Math.toRadians(270)))
                .build()
            );
        }

//        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
