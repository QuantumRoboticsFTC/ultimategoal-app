package eu.qrobotics.ultimategoal.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.ultimategoal.teamcode.subsystems.Robot;

@TeleOp(group = "Test")
public class OdometryTest extends OpMode {
    private Robot robot;
    private MultipleTelemetry telemetry;

    @Override
    public void init() {
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
        telemetry.addData("Pose", robot.drive.getPoseEstimate().toString());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
