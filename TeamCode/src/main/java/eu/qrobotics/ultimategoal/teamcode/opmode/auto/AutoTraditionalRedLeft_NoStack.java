package eu.qrobotics.ultimategoal.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

import eu.qrobotics.ultimategoal.teamcode.opmode.auto.cv.RingDetector;
import eu.qrobotics.ultimategoal.teamcode.opmode.auto.trajectories.TrajectoriesTraditionalRedLeft_NoStack;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Buffer;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Intake;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Outtake;
import eu.qrobotics.ultimategoal.teamcode.subsystems.RingStopper;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Robot;
import eu.qrobotics.ultimategoal.teamcode.subsystems.WobbleGoalGrabber;

@Config
@Autonomous
@Disabled
public class AutoTraditionalRedLeft_NoStack extends LinearOpMode {
    //    public static Point TOP_LEFT = new Point(500, 250);
//    public static Point BOTTOM_RIGHT = new Point(775, 500);
    public static Point TOP_LEFT = new Point(900, 400);
    public static Point BOTTOM_RIGHT = new Point(1200, 700); // Camera

//    public static RingDetector.Stack RING_STACK = RingDetector.Stack.ZERO;

//    private MultipleTelemetry telemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        Outtake.RED_ALLIANCE = true;
//        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this, true);
        robot.drive.setPoseEstimate(TrajectoriesTraditionalRedLeft_NoStack.START_POSE);
        List<Trajectory> trajectoriesA = TrajectoriesTraditionalRedLeft_NoStack.getTrajectoriesA();
        List<Trajectory> trajectoriesB = TrajectoriesTraditionalRedLeft_NoStack.getTrajectoriesB();
        List<Trajectory> trajectoriesC = TrajectoriesTraditionalRedLeft_NoStack.getTrajectoriesC();
        robot.start();

        telemetry.addLine("Driver 1, press A to close WG Grabber");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.showFpsMeterOnViewport(true);
        RingDetector ringDetector = new RingDetector(webcam, TOP_LEFT, BOTTOM_RIGHT);

//        webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
        webcam.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT); // We forgot camera so we bought shit one from Altex
        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        RingDetector.Stack ringStack = RingDetector.Stack.FOUR;

        while(!isStarted()) {
            if(gamepad1.a) {
                robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.CLOSE;
            }

            ringStack = ringDetector.getStack();
            telemetry.addData("Count", ringDetector.getCount() );
            telemetry.addData("Rings", ringStack);
            telemetry.update();
        }
        webcam.stopStreaming();

        if(isStopRequested()) {
            robot.stop();
            return;
        }
        resetStartTime();

        robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.CLOSE;

        robot.outtake.outtakeTarget = Outtake.OuttakeTarget.HIGH_GOAL;
        robot.outtake.outtakeMode = Outtake.OuttakeMode.ON;
        robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
        robot.intake.intakeMode = Intake.IntakeMode.IN_SLOW;
        robot.ringStopper.ringStopperMode = RingStopper.RingStopperMode.DOWN;

        if(ringStack == RingDetector.Stack.ZERO) {
            // A
            robot.sleep(10);

            robot.drive.followTrajectorySync(trajectoriesA.get(0));

            robot.sleep(1);

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_ALL;
            robot.buffer.pushAttempts = 0;
            while(robot.buffer.bufferPusherMode != Buffer.BufferPusherMode.IDLE && robot.buffer.pushAttempts < 4) {
                robot.sleep(0.05);
                if(isStopRequested()) {
                    robot.stop();
                    return;
                }
            }
            robot.sleep(0.2);

            robot.buffer.bufferMode = Buffer.BufferMode.COLLECT;
            robot.ringStopper.ringStopperMode = RingStopper.RingStopperMode.INITIAL;

            robot.drive.followTrajectorySync(trajectoriesA.get(1));

            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);

            robot.drive.followTrajectorySync(trajectoriesA.get(2));
        }
        else if(ringStack == RingDetector.Stack.ONE) {
            // B
            robot.sleep(14);

            robot.drive.followTrajectorySync(trajectoriesB.get(0));

            robot.sleep(1);

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_ALL;
            robot.buffer.pushAttempts = 0;
            while(robot.buffer.bufferPusherMode != Buffer.BufferPusherMode.IDLE && robot.buffer.pushAttempts < 4) {
                robot.sleep(0.05);
                if(isStopRequested()) {
                    robot.stop();
                    return;
                }
            }
            robot.sleep(0.2);

            robot.buffer.bufferMode = Buffer.BufferMode.COLLECT;
            robot.ringStopper.ringStopperMode = RingStopper.RingStopperMode.INITIAL;

            robot.drive.followTrajectorySync(trajectoriesB.get(1));

            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.UP;

            robot.drive.followTrajectorySync(trajectoriesB.get(2));
        }
        else if(ringStack == RingDetector.Stack.FOUR) {
            // C
            robot.sleep(15);

            robot.drive.followTrajectorySync(trajectoriesC.get(0));

            robot.sleep(1);

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_ALL;
            robot.buffer.pushAttempts = 0;
            while(robot.buffer.bufferPusherMode != Buffer.BufferPusherMode.IDLE && robot.buffer.pushAttempts < 4) {
                robot.sleep(0.05);
                if(isStopRequested()) {
                    robot.stop();
                    return;
                }
            }
            robot.sleep(0.2);

            robot.buffer.bufferMode = Buffer.BufferMode.COLLECT;
            robot.ringStopper.ringStopperMode = RingStopper.RingStopperMode.INITIAL;

            robot.drive.followTrajectorySync(trajectoriesC.get(1));

            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.UP;

            robot.drive.followTrajectorySync(trajectoriesC.get(2));
        }

        robot.sleep(0.2);

        robot.stop();
    }
}