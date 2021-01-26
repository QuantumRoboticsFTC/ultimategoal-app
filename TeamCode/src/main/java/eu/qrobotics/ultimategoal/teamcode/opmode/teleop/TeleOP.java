package eu.qrobotics.ultimategoal.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.ultimategoal.teamcode.subsystems.Buffer.BufferMode;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Buffer.BufferPusherMode;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Intake.IntakeMode;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Outtake;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Outtake.OuttakeMode;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Robot;
import eu.qrobotics.ultimategoal.teamcode.subsystems.WobbleGoalGrabber.WobbleGoalArmMode;
import eu.qrobotics.ultimategoal.teamcode.subsystems.WobbleGoalGrabber.WobbleGoalClawMode;
import eu.qrobotics.ultimategoal.teamcode.util.StickyGamepad;

import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.BASE_CONSTRAINTS;

@TeleOp
public class TeleOP extends OpMode {
    enum DriveMode {
        NORMAL,
        SLOW,
        SUPER_SLOW
    }

    Robot robot;
    DriveMode driveMode;
    StickyGamepad stickyGamepad1 = null;
    StickyGamepad stickyGamepad2 = null;

    @Override
    public void init() {
        robot = new Robot(this, false);
//        robot.drive.fieldCentric = true;
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
        driveMode = DriveMode.NORMAL;

        telemetry.log().add("Ready!");
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();

        //region Driver 1 controls

        switch (driveMode) {
            case NORMAL:
                robot.drive.setMotorPowersFromGamepad(gamepad1, 1);
                break;
            case SLOW:
                robot.drive.setMotorPowersFromGamepad(gamepad1, 0.7);
                break;
            case SUPER_SLOW:
                robot.drive.setMotorPowersFromGamepad(gamepad1, 0.5);
                break;
        }

        if (stickyGamepad1.a) {
            if (driveMode != DriveMode.SLOW)
                driveMode = DriveMode.SLOW;
            else
                driveMode = DriveMode.NORMAL;
        } else if (stickyGamepad1.b) {
            if (driveMode != DriveMode.SUPER_SLOW)
                driveMode = DriveMode.SUPER_SLOW;
            else
                driveMode = DriveMode.NORMAL;
        }

        if (stickyGamepad1.left_bumper) {
            driveMode = DriveMode.SLOW;
        }
        if (stickyGamepad1.right_bumper) {
            driveMode = DriveMode.NORMAL;
        }

        if (stickyGamepad1.dpad_up) {
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalClawMode.CLOSE;
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalArmMode.UP;
        }
        if (stickyGamepad1.dpad_right) {
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalClawMode.OPEN;
        }
        if (stickyGamepad1.dpad_down) {
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalArmMode.DOWN;
        }

        //endregion

        // region Driver 2 controls

        if (gamepad2.left_stick_y < -0.1)
            robot.intake.intakeMode = IntakeMode.IN;
        else if (gamepad2.left_stick_y > 0.1)
            robot.intake.intakeMode = IntakeMode.OUT_SLOW;
        else if (robot.intake.intakeMode == IntakeMode.OUT_SLOW)
            robot.intake.intakeMode = IntakeMode.IDLE;

        if (stickyGamepad2.a) {
            robot.buffer.bufferMode = BufferMode.COLLECT;
            robot.outtake.outtakeMode = OuttakeMode.OFF;
        }

        if(stickyGamepad2.b) {
            robot.buffer.bufferMode = BufferMode.OUTTAKE;
            robot.outtake.outtakeMode = OuttakeMode.ON;
        }

        if (stickyGamepad2.x) {
            robot.buffer.bufferPusherMode = BufferPusherMode.PUSH_SINGLE;
        }
        if (stickyGamepad2.y) {
            robot.buffer.bufferPusherMode = BufferPusherMode.PUSH_ALL;
        }

        // endregion

        telemetry.addData("Buffer rings", robot.buffer.getRingCount());
        telemetry.addData("Outtake target RPM", robot.outtake.getTargetRPM());
        telemetry.addData("Outtake current RPM", robot.outtake.getCurrentRPM());
        telemetry.addData("Outtake ready?", robot.outtake.isReady());
        telemetry.addData("Buffer pusher state", robot.buffer.getBufferPusherState());
        telemetry.addData("Buffer pusher mode", robot.buffer.bufferPusherMode);
        telemetry.addData("Target robot angle", Outtake.TOWER_GOAL_POS.minus(robot.drive.getPoseEstimate().vec()).angle());
        telemetry.addData("Current robot angle", robot.drive.getPoseEstimate().getHeading());
        telemetry.addData("Wobble Arm", robot.wobbleGoalGrabber.wobbleGoalArmMode);
        telemetry.addData("Wobble Claw", robot.wobbleGoalGrabber.wobbleGoalClawMode);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
