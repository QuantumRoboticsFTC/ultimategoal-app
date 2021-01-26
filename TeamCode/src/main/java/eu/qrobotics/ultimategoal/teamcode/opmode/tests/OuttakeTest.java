package eu.qrobotics.ultimategoal.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.ultimategoal.teamcode.subsystems.Buffer;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Outtake;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Robot;
import eu.qrobotics.ultimategoal.teamcode.util.StickyGamepad;

@TeleOp(group = "Test")
public class OuttakeTest extends OpMode {
    private Robot robot;
    private StickyGamepad stickyGamepad1;

    @Override
    public void init() {
        robot = new Robot(this, false);
        stickyGamepad1 = new StickyGamepad(gamepad1);
        telemetry.log().add("Ready!");
    }

    @Override
    public void start() {
        robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
        robot.start();
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        if(stickyGamepad1.a) {
            robot.outtake.outtakeMode = Outtake.OuttakeMode.ON;
        }
        if(stickyGamepad1.b) {
            robot.outtake.outtakeMode = Outtake.OuttakeMode.OFF;
        }
        if(stickyGamepad1.dpad_up) {
            robot.outtake.overrideRPM += 100;
        }
        if(stickyGamepad1.dpad_down) {
            robot.outtake.overrideRPM -= 100;
        }
        if(stickyGamepad1.dpad_left) {
            robot.outtake.overrideRPM -= 1;
        }
        if(stickyGamepad1.dpad_right) {
            robot.outtake.overrideRPM += 1;
        }
        if(stickyGamepad1.x) {
            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
        }
        telemetry.addData("Outtake current RPM", robot.outtake.getCurrentRPM());
        telemetry.addData("Outtake target RPM", robot.outtake.getTargetRPM());
        telemetry.addData("Robot location", robot.drive.getPoseEstimate());
        telemetry.addData("Distance", robot.outtake.getDistance());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
