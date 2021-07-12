package eu.qrobotics.ultimategoal.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import eu.qrobotics.ultimategoal.teamcode.subsystems.Buffer.BufferMode;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Buffer.BufferPusherMode;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Intake;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Intake.IntakeMode;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Outtake;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Outtake.OuttakeMode;
import eu.qrobotics.ultimategoal.teamcode.subsystems.RingStopper;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Robot;
import eu.qrobotics.ultimategoal.teamcode.subsystems.WobbleGoalGrabber.WobbleGoalArmMode;
import eu.qrobotics.ultimategoal.teamcode.subsystems.WobbleGoalGrabber.WobbleGoalClawMode;
import eu.qrobotics.ultimategoal.teamcode.util.DashboardUtil;
import eu.qrobotics.ultimategoal.teamcode.util.StickyGamepad;

@TeleOp
public class TeleOPRed extends OpMode {
    enum DriveMode {
        NORMAL,
        SLOW,
        SUPER_SLOW
    }

    Robot robot;
    DriveMode driveMode;
    StickyGamepad stickyGamepad1 = null;
    StickyGamepad stickyGamepad2 = null;

    MultipleTelemetry telemetry;

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void init() {
        Outtake.RED_ALLIANCE = true;
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(this, false);
//        robot.drive.fieldCentric = true;
        robot.intake.intakeStopperMode = Intake.IntakeStopperMode.UP;
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
        driveMode = DriveMode.NORMAL;

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry.log().add("Ready!");
    }

    @Override
    public void start() {
        robot.start();
    }

    private ElapsedTime wobbleGrabTimer = new ElapsedTime();
    private ElapsedTime bufferUpTimer = new ElapsedTime();
    private ElapsedTime buffer2RingsTimer = new ElapsedTime();
    private ElapsedTime buffer3RingsTimer = new ElapsedTime();
    private ElapsedTime autoPowershotsTimer = new ElapsedTime();

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();

        if(autoPowershotsTimer.seconds() < 5) {
            if(autoPowershotsTimer.seconds() < 0.5 && !robot.outtake.isReady())
                autoPowershotsTimer.reset();
            if(1 < autoPowershotsTimer.seconds() && autoPowershotsTimer.seconds() < 1.1)
                robot.buffer.bufferPusherMode = BufferPusherMode.PUSH_SINGLE;
            if(1.5 < autoPowershotsTimer.seconds() && autoPowershotsTimer.seconds() < 1.6)
                robot.outtake.outtakeTarget = Outtake.OuttakeTarget.POWER_SHOT_2;
            if(2 < autoPowershotsTimer.seconds() && autoPowershotsTimer.seconds() < 2.1)
                robot.buffer.bufferPusherMode = BufferPusherMode.PUSH_SINGLE;
            if(2.5 < autoPowershotsTimer.seconds() && autoPowershotsTimer.seconds() < 2.6)
                robot.outtake.outtakeTarget = Outtake.OuttakeTarget.POWER_SHOT_3;
            if(3 < autoPowershotsTimer.seconds() && autoPowershotsTimer.seconds() < 3.1)
                robot.buffer.bufferPusherMode = BufferPusherMode.PUSH_SINGLE;
        }

        //region Driver 1 controls

        if(!robot.drive.isBusy()) {
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
            if(gamepad1.right_trigger > 0.25) {
                robot.drive.setPoseEstimate(new Pose2d(-63, -63, 0));
            }
            if(gamepad1.left_trigger > 0.25) {
                robot.drive.setPoseEstimate(new Pose2d(2, -39, 0));
            }
            if(stickyGamepad1.y) {
                autoPowershotsTimer.reset();
                robot.buffer.bufferMode = BufferMode.OUTTAKE;
                robot.outtake.outtakeMode = OuttakeMode.ON;
                robot.outtake.outtakeTarget = Outtake.OuttakeTarget.POWER_SHOT_1;
            }
        }
        if (stickyGamepad1.left_bumper) {
            driveMode = DriveMode.SUPER_SLOW;
        }
        if (stickyGamepad1.right_bumper) {
            driveMode = DriveMode.NORMAL;
        }

        if (stickyGamepad1.dpad_up) {
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalClawMode.CLOSE;
            wobbleGrabTimer.reset();
        }
        if (stickyGamepad1.dpad_right) {
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalClawMode.OPEN;
        }
        if (stickyGamepad1.dpad_down) {
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalArmMode.DOWN;
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalClawMode.OPEN;
        }
        if (stickyGamepad1.dpad_left) {
            if(robot.wobbleGoalGrabber.wobbleGoalArmMode == WobbleGoalArmMode.UP) {
                robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalArmMode.INITIAL;
            }
            else if(robot.wobbleGoalGrabber.wobbleGoalArmMode == WobbleGoalArmMode.INITIAL) {
                robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalArmMode.UP;
            }
        }

        if (wobbleGrabTimer.seconds() > 0.5 && wobbleGrabTimer.seconds() < 0.6) {
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalArmMode.UP;
        }

        //endregion

        // region Driver 2 controls

        if (gamepad2.left_stick_y < -0.1)
            robot.intake.intakeMode = IntakeMode.IN;
        else if (gamepad2.left_stick_y > 0.1) {
            if(gamepad2.left_stick_button)
                robot.intake.intakeMode = IntakeMode.OUT;
            else
                robot.intake.intakeMode = IntakeMode.OUT_SLOW;
        }
        else if (robot.intake.intakeMode == IntakeMode.OUT_SLOW || robot.intake.intakeMode == IntakeMode.OUT)
            robot.intake.intakeMode = IntakeMode.IDLE;

        if (stickyGamepad2.a) {
            robot.buffer.bufferMode = BufferMode.COLLECT;
            robot.outtake.outtakeMode = OuttakeMode.IDLE;
            robot.intake.intakeMode = IntakeMode.IN;
        }

        if(stickyGamepad2.b) {
            robot.buffer.bufferMode = BufferMode.OUTTAKE;
            robot.outtake.outtakeMode = OuttakeMode.ON;
            robot.intake.intakeMode = IntakeMode.IN_SLOW;
            bufferUpTimer.reset();
        }

        if (stickyGamepad2.x) {
            robot.buffer.bufferPusherMode = BufferPusherMode.PUSH_SINGLE;
        }
        if (stickyGamepad2.y) {
            robot.buffer.bufferPusherMode = BufferPusherMode.PUSH_ALL;
        }

        if(stickyGamepad2.left_bumper) {
            if(robot.outtake.outtakeTarget == Outtake.OuttakeTarget.HIGH_GOAL)
                robot.outtake.outtakeTarget = Outtake.OuttakeTarget.POWER_SHOT_1;
            else if(robot.outtake.outtakeTarget == Outtake.OuttakeTarget.POWER_SHOT_1)
                robot.outtake.outtakeTarget = Outtake.OuttakeTarget.POWER_SHOT_2;
            else if(robot.outtake.outtakeTarget == Outtake.OuttakeTarget.POWER_SHOT_2)
                robot.outtake.outtakeTarget = Outtake.OuttakeTarget.POWER_SHOT_3;
        }
        if(stickyGamepad2.right_bumper) {
            if(robot.outtake.outtakeTarget == Outtake.OuttakeTarget.POWER_SHOT_1)
                robot.outtake.outtakeTarget = Outtake.OuttakeTarget.HIGH_GOAL;
            else if(robot.outtake.outtakeTarget == Outtake.OuttakeTarget.POWER_SHOT_2)
                robot.outtake.outtakeTarget = Outtake.OuttakeTarget.POWER_SHOT_1;
            else if(robot.outtake.outtakeTarget == Outtake.OuttakeTarget.POWER_SHOT_3)
                robot.outtake.outtakeTarget = Outtake.OuttakeTarget.POWER_SHOT_2;
        }
        if(robot.buffer.bufferMode == BufferMode.COLLECT && (0.6 <= buffer2RingsTimer.seconds() && buffer2RingsTimer.seconds() <= 0.7)) {
            robot.ringStopper.ringStopperMode = RingStopper.RingStopperMode.UP;
        }
        if(robot.buffer.bufferMode == BufferMode.OUTTAKE && robot.buffer.bufferPusherMode != BufferPusherMode.IDLE && robot.outtake.outtakeTarget == Outtake.OuttakeTarget.HIGH_GOAL) {
            robot.ringStopper.ringStopperMode = RingStopper.RingStopperMode.DOWN;
        }
        if(gamepad2.dpad_up) {
            robot.ringStopper.ringStopperMode = RingStopper.RingStopperMode.UP;
        }
        if(gamepad2.dpad_down) {
            robot.ringStopper.ringStopperMode = RingStopper.RingStopperMode.DOWN;
        }
        if(gamepad2.dpad_left) {
            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.DOWN;
        }
        if(gamepad2.dpad_right) {
            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.UP;
        }
        if(gamepad2.left_trigger > 0.25) {
            robot.outtake.outtakeMode = OuttakeMode.OFF;
        }
        if(gamepad2.right_trigger > 0.25) {
            robot.outtake.outtakeMode = OuttakeMode.IDLE;
        }
        if (bufferUpTimer.seconds() > 0.7 && bufferUpTimer.seconds() < 0.8) {
            robot.intake.intakeMode = IntakeMode.IDLE;
        }

//        if((0.6 <= buffer3RingsTimer.seconds() && buffer3RingsTimer.seconds() <= 0.7) && robot.intake.intakeMode == IntakeMode.IN) {
//            robot.intake.intakeMode = IntakeMode.IN_SLOW;
//            robot.buffer.bufferMode = BufferMode.OUTTAKE;
//            robot.outtake.outtakeMode = OuttakeMode.ON;
//        }
//        if((0.8 <= buffer3RingsTimer.seconds() && buffer3RingsTimer.seconds() <= 0.9) && robot.intake.intakeMode == IntakeMode.IN_SLOW) {
//            robot.intake.intakeMode = IntakeMode.IDLE;
//        }
        // endregion

        if(robot.buffer.getRingCount() < 2)
            buffer2RingsTimer.reset();
        if(robot.buffer.getRingCount() < 3)
            buffer3RingsTimer.reset();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, robot.drive.getPoseEstimate());

        packet.put("Battery voltage", batteryVoltageSensor.getVoltage());
        packet.put("Mean250 sensor time", robot.buffer.avgSensorTime10.getMean() * 1000);
        packet.put("Dev250 sensor time", robot.buffer.avgSensorTime10.getStandardDeviation() * 1000);
        packet.put("Mean250 robot time", robot.top10.getMean() * 1000);
        packet.put("Dev250 robot time", robot.top10.getStandardDeviation() * 1000);
        packet.put("Buffer rings", robot.buffer.getRingCount());
        packet.put("Buffer current distance", robot.buffer.ringSensorValues.getMean() + robot.buffer.ringSensorValues.getStandardDeviation() / 2);
        packet.put("1 ring", 110);
        packet.put("2 rings", 100);
        packet.put("3 rings", 90);
        packet.put("Buffer distance mean", robot.buffer.ringSensorValues.getMean());
        packet.put("Buffer distance stdev", robot.buffer.ringSensorValues.getStandardDeviation());
        packet.put("Outtake target", robot.outtake.outtakeTarget);
        packet.put("Outtake target RPM", robot.outtake.getTargetRPM());
        packet.put("Outtake current RPM", robot.outtake.getCurrentRPM());
        packet.put("Outtake ready?", robot.outtake.isReady());
        packet.put("Buffer pusher state", robot.buffer.getBufferPusherState());
        packet.put("Buffer pusher mode", robot.buffer.bufferPusherMode);
        packet.put("Wobble Arm", robot.wobbleGoalGrabber.wobbleGoalArmMode);
        packet.put("Wobble Claw", robot.wobbleGoalGrabber.wobbleGoalClawMode);
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
//        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
