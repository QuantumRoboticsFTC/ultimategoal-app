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
import eu.qrobotics.ultimategoal.teamcode.opmode.auto.trajectories.TrajectoriesTraditionalBlueLeft_Solo;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Buffer;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Intake;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Outtake;
import eu.qrobotics.ultimategoal.teamcode.subsystems.RingStopper;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Robot;
import eu.qrobotics.ultimategoal.teamcode.subsystems.WobbleGoalGrabber;

@Config
@Autonomous
public class AutoTraditionalBlueLeft_Solo extends LinearOpMode {
    public static Point TOP_LEFT = new Point(975, 325);
    public static Point BOTTOM_RIGHT = new Point(1225, 900); // Camera

//    public static RingDetector.Stack RING_STACK = RingDetector.Stack.ZERO;

//    private MultipleTelemetry telemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        Outtake.RED_ALLIANCE = false;
//        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this, true);
        robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.INITIAL;
        robot.drive.setPoseEstimate(TrajectoriesTraditionalBlueLeft_Solo.START_POSE);
        List<Trajectory> trajectoriesA = TrajectoriesTraditionalBlueLeft_Solo.getTrajectoriesA();
        List<Trajectory> trajectoriesB = TrajectoriesTraditionalBlueLeft_Solo.getTrajectoriesB();
        List<Trajectory> trajectoriesC = TrajectoriesTraditionalBlueLeft_Solo.getTrajectoriesC();
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
        robot.ringStopper.ringStopperMode = RingStopper.RingStopperMode.DOWN;

        if(ringStack == RingDetector.Stack.ZERO) {
            // A
            robot.drive.followTrajectorySync(trajectoriesA.get(0));

            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
            robot.scoringBlocker.moveOut();
            robot.sleep(0.1);
            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.MID;

            while(!robot.outtake.isReady() && !isStopRequested()) {
                robot.sleep(0.05);
            }
            robot.sleep(0.5);
            robot.scoringBlocker.moveIn();

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
            robot.outtake.outtakeMode = Outtake.OuttakeMode.OFF;
            robot.ringStopper.ringStopperMode = RingStopper.RingStopperMode.INITIAL;

            robot.drive.followTrajectorySync(trajectoriesA.get(1));

            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.MID;
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);

            robot.drive.followTrajectorySync(trajectoriesA.get(2));

            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.CLOSE;
            robot.sleep(0.4);
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.UP;
            robot.sleep(0.2);

            robot.drive.followTrajectorySync(trajectoriesA.get(3));

            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);

            robot.scoringBlocker.blockerPosition = 0.75;

            robot.drive.followTrajectorySync(trajectoriesA.get(4));
        }
        else if(ringStack == RingDetector.Stack.ONE) {
            // B
            robot.drive.followTrajectorySync(trajectoriesB.get(0));

            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
            robot.scoringBlocker.moveOut();
            robot.sleep(0.1);
            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.MID;

            while(!robot.outtake.isReady() && !isStopRequested()) {
                robot.sleep(0.05);
            }
            robot.sleep(0.5);
            robot.scoringBlocker.moveIn();

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
            robot.ringStopper.ringStopperMode = RingStopper.RingStopperMode.INITIAL;

            robot.buffer.bufferMode = Buffer.BufferMode.COLLECT;
            robot.sleep(0.2);
            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.DOWN;
            robot.intake.intakeMode = Intake.IntakeMode.IN;

            robot.drive.followTrajectorySync(trajectoriesB.get(1));

            robot.sleep(1.0);
            robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
            robot.sleep(0.2);
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
            robot.outtake.outtakeMode = Outtake.OuttakeMode.OFF;
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;
            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.MID;

            robot.drive.followTrajectorySync(trajectoriesB.get(2));

            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.MID;
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);

            robot.buffer.bufferMode = Buffer.BufferMode.COLLECT;
            robot.outtake.outtakeTarget = Outtake.OuttakeTarget.HIGH_GOAL;

            robot.drive.followTrajectorySync(trajectoriesB.get(3));

            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.CLOSE;
            robot.sleep(0.4);
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.UP;
            robot.sleep(0.2);

            robot.drive.followTrajectorySync(trajectoriesB.get(4));

            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);

            robot.drive.followTrajectorySync(trajectoriesB.get(5));
        }
        else if(ringStack == RingDetector.Stack.FOUR) {
            // C
            robot.drive.followTrajectorySync(trajectoriesC.get(0));

            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
            robot.scoringBlocker.moveOut();
            robot.sleep(0.1);
            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.MID;

            while(!robot.outtake.isReady() && !isStopRequested()) {
                robot.sleep(0.05);
            }
            robot.sleep(0.5);
            robot.scoringBlocker.moveIn();

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
            robot.outtake.outtakeTarget = Outtake.OuttakeTarget.HIGH_GOAL;
            robot.intake.intakeMode = Intake.IntakeMode.IN;
            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.DOWN;

            robot.drive.followTrajectory(trajectoriesC.get(1));

            while(robot.drive.isBusy()) {
                if(robot.buffer.getRingCount() >= 3) {
                    robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
                }
            }

            if(robot.intake.intakeMode == Intake.IntakeMode.IN) {
                robot.sleep(0.3);
                robot.intake.intakeMode = Intake.IntakeMode.OUT;
                robot.sleep(0.5);
                robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
                robot.sleep(0.2);
            }
            robot.intake.intakeMode = Intake.IntakeMode.IN_SLOW;
            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
            robot.sleep(0.2);
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;
            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_ALL;
            robot.buffer.pushAttempts = 0;
            while(robot.buffer.bufferPusherMode != Buffer.BufferPusherMode.IDLE && robot.buffer.pushAttempts < 4) {
                robot.sleep(0.05);
                if(isStopRequested()) {
                    robot.stop();
                    return;
                }
            }
            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.IDLE;
            robot.buffer.bufferMode = Buffer.BufferMode.COLLECT;
            robot.intake.intakeMode = Intake.IntakeMode.IN;

            robot.drive.followTrajectory(trajectoriesC.get(2));

            if(robot.intake.intakeMode == Intake.IntakeMode.IN) {
                robot.sleep(3);
            }
            robot.intake.intakeMode = Intake.IntakeMode.OUT;
            robot.sleep(0.3);
            robot.intake.intakeMode = Intake.IntakeMode.IN_SLOW;
            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
            robot.sleep(0.2);
            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_ALL;
            robot.buffer.pushAttempts = 0;
            while(robot.buffer.bufferPusherMode != Buffer.BufferPusherMode.IDLE && robot.buffer.pushAttempts < 4) {
                robot.sleep(0.05);
                if(isStopRequested()) {
                    robot.stop();
                    return;
                }
            }
            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.IDLE;
            robot.outtake.outtakeMode = Outtake.OuttakeMode.OFF;
            robot.buffer.bufferMode = Buffer.BufferMode.COLLECT;
            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.MID;
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;

            robot.drive.followTrajectorySync(trajectoriesC.get(3));

            robot.intake.intakeStopperMode = Intake.IntakeStopperMode.MID;
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.2);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.1);

            robot.drive.followTrajectorySync(trajectoriesC.get(4));

            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.CLOSE;
            robot.sleep(0.4);

            robot.drive.followTrajectorySync(trajectoriesC.get(5));

            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);
            robot.scoringBlocker.moveOut();

            robot.drive.followTrajectorySync(trajectoriesC.get(6));
        }

        robot.sleep(30.0); // anti defence

        robot.stop();
    }
}