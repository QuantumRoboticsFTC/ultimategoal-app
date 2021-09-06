package eu.qrobotics.ultimategoal.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

import eu.qrobotics.ultimategoal.teamcode.opmode.auto.cv.RingDetector;
import eu.qrobotics.ultimategoal.teamcode.opmode.auto.trajectories.TrajectoriesRemoteRed;
import eu.qrobotics.ultimategoal.teamcode.opmode.auto.trajectories.TrajectoriesTraditionalRedLeftTower;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Buffer;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Intake;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Outtake;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Robot;
import eu.qrobotics.ultimategoal.teamcode.subsystems.WobbleGoalGrabber;

@Config
@Autonomous
//@Disabled
public class AutoTraditionalRedLeftTower extends LinearOpMode {
    public static Point TOP_LEFT = new Point(975, 325);
    public static Point BOTTOM_RIGHT = new Point(1225, 900); // Camera

//    public static RingDetector.Stack RING_STACK = RingDetector.Stack.ZERO;

//    private MultipleTelemetry telemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        Outtake.RED_ALLIANCE = true;
//        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this, true);
        robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.INITIAL;
        robot.drive.setPoseEstimate(TrajectoriesTraditionalRedLeftTower.START_POSE);
        List<Trajectory> trajectoriesA = TrajectoriesTraditionalRedLeftTower.getTrajectoriesA();
        List<Trajectory> trajectoriesB = TrajectoriesTraditionalRedLeftTower.getTrajectoriesB();
        List<Trajectory> trajectoriesC = TrajectoriesTraditionalRedLeftTower.getTrajectoriesC();
        robot.start();

        telemetry.addLine("Driver 1, press A to close WG Grabber");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.showFpsMeterOnViewport(true);
        RingDetector ringDetector = new RingDetector(webcam, TOP_LEFT, BOTTOM_RIGHT);

        webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
//        webcam.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT); // We forgot camera so we bought shit one from Altex

        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        RingDetector.Stack ringStack = RingDetector.Stack.FOUR;

        while(!isStarted() && !isStopRequested()) {
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
        robot.intake.intakeMode = Intake.IntakeMode.IN;

        if(ringStack == RingDetector.Stack.ZERO) {
            // A
            robot.drive.followTrajectorySync(trajectoriesA.get(0));
            robot.sleep(0.5);
            robot.outtake.outtakeMode = Outtake.OuttakeMode.ON;
            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;

            robot.scoringBlocker.moveOut();
            robot.sleep(0.1);
            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.MID;

            while(!robot.outtake.isReady() && !isStopRequested()) {
                robot.sleep(0.05);
            }
            robot.sleep(0.5);
            robot.scoringBlocker.moveIn();

            robot.buffer.pushAttempts = 0;
            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_ALL;
            while(robot.buffer.bufferPusherMode != Buffer.BufferPusherMode.IDLE && robot.buffer.pushAttempts < 4 && !isStopRequested()) {
                robot.sleep(0.05);
            }

            robot.outtake.outtakeMode = Outtake.OuttakeMode.OFF;

            robot.drive.followTrajectorySync(trajectoriesA.get(1));

            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.MID;
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.UP;

            robot.drive.followTrajectory(trajectoriesA.get(2));
        }
        else if(ringStack == RingDetector.Stack.ONE) {
            // B
            robot.drive.followTrajectorySync(trajectoriesB.get(0));
            robot.sleep(0.5);
            robot.outtake.outtakeMode = Outtake.OuttakeMode.ON;
            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;

            robot.scoringBlocker.moveOut();
            robot.sleep(0.1);
            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.MID;

            while(!robot.outtake.isReady() && !isStopRequested()) {
                robot.sleep(0.05);
            }
            robot.sleep(0.5);
            robot.scoringBlocker.moveIn();

            robot.buffer.pushAttempts = 0;
            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_ALL;
            while(robot.buffer.bufferPusherMode != Buffer.BufferPusherMode.IDLE && robot.buffer.pushAttempts < 4 && !isStopRequested()) {
                robot.sleep(0.05);
            }

            robot.outtake.outtakeMode = Outtake.OuttakeMode.OFF;

            robot.drive.followTrajectorySync(trajectoriesB.get(1));

            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.MID;
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.UP;

            robot.drive.followTrajectory(trajectoriesB.get(2));
        }
        else if(ringStack == RingDetector.Stack.FOUR) {
            // C
            robot.drive.followTrajectorySync(trajectoriesC.get(0));
            robot.sleep(0.5);
            robot.outtake.outtakeMode = Outtake.OuttakeMode.ON;
            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;

            robot.scoringBlocker.moveOut();
            robot.sleep(0.1);
            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.MID;

            while(!robot.outtake.isReady() && !isStopRequested()) {
                robot.sleep(0.05);
            }
            robot.sleep(0.5);
            robot.scoringBlocker.moveIn();

            robot.buffer.pushAttempts = 0;
            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_ALL;
            while(robot.buffer.bufferPusherMode != Buffer.BufferPusherMode.IDLE && robot.buffer.pushAttempts < 4 && !isStopRequested()) {
                robot.sleep(0.05);
            }

            robot.outtake.outtakeMode = Outtake.OuttakeMode.OFF;

            robot.drive.followTrajectorySync(trajectoriesC.get(1));

            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.MID;
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.UP;

            robot.drive.followTrajectory(trajectoriesC.get(2));
        }

        robot.sleep(30.0); // anti defence

        robot.stop();
    }
}