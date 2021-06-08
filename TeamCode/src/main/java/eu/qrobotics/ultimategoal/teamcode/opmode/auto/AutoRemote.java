package eu.qrobotics.ultimategoal.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

import eu.qrobotics.ultimategoal.teamcode.opmode.auto.cv.RingDetector;
import eu.qrobotics.ultimategoal.teamcode.opmode.auto.trajectories.TrajectoriesRemoteRed;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Buffer;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Intake;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Outtake;
import eu.qrobotics.ultimategoal.teamcode.subsystems.RingStopper;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Robot;
import eu.qrobotics.ultimategoal.teamcode.subsystems.WobbleGoalGrabber;

@Config
@Autonomous
//@Disabled
public class AutoRemote extends LinearOpMode {
    private static final String VUFORIA_KEY =
            "AZkUdjT/////AAABmXTO5InzOk5Yo1OlXRo+EFQkVF1qqCBnur0y1G+RBktOx7nVuzRvRaahrrHE0OJTAUwmyuPkGSbIFETtV9VN5Ezo8vTtN90u2lqAMZx5ZY5qWtTs+rm/2y4CctYrNhnxeme+qRNeRj6gKhUMa2FAVHr2qBtJK/CrZ0Ud/1vpLavIr+TrHuIjABcEXRyXBcdIaj5gw4EiVChCFrjv24qMiHuOq1pHOAbpTqe392045VnPLDlJ6bJKq0cNZ3TR86ccLGd2Pg0lnVLvf/qthVFRy8NASoyrgQkEU0P5WSC+8A3IlWPPEgG2LMu8FACw+6t1da+EiznSyu5dSW7UcAw5oHKpGgfxRV3pXmNJ3bn+AfMi";

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public static Point TOP_LEFT = new Point(500, 250);
    public static Point BOTTOM_RIGHT = new Point(775, 500);

//    public static RingDetector.Stack RING_STACK = RingDetector.Stack.ZERO;

//    private MultipleTelemetry telemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        Outtake.RED_ALLIANCE = true;
//        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this, true);
        robot.drive.setPoseEstimate(TrajectoriesRemoteRed.START_POSE);
        List<Trajectory> trajectoriesA = TrajectoriesRemoteRed.getTrajectoriesA();
        List<Trajectory> trajectoriesB = TrajectoriesRemoteRed.getTrajectoriesB();
        List<Trajectory> trajectoriesC = TrajectoriesRemoteRed.getTrajectoriesC();
        robot.start();

        telemetry.addLine("Driver 1, press A to close WG Grabber");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.showFpsMeterOnViewport(true);
        RingDetector ringDetector = new RingDetector(webcam, TOP_LEFT, BOTTOM_RIGHT);

        webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
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

        robot.outtake.outtakeTarget = Outtake.OuttakeTarget.POWER_SHOT;
        robot.outtake.outtakeMode = Outtake.OuttakeMode.ON;
        robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
        robot.intake.intakeMode = Intake.IntakeMode.IN_SLOW;
        robot.ringStopper.ringStopperMode = RingStopper.RingStopperMode.DOWN;

        if(ringStack == RingDetector.Stack.ZERO) {
            // A
            robot.drive.followTrajectorySync(trajectoriesA.get(0));

            robot.sleep(0.5);
            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(0.5);

            robot.drive.followTrajectorySync(trajectoriesA.get(1));

            robot.sleep(0.5);
            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(0.5);

            robot.drive.followTrajectorySync(trajectoriesA.get(2));

            robot.sleep(0.5);
            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(0.5);

            robot.outtake.outtakeMode = Outtake.OuttakeMode.OFF;
            robot.buffer.bufferMode = Buffer.BufferMode.COLLECT;
            robot.ringStopper.ringStopperMode = RingStopper.RingStopperMode.INITIAL;

            robot.drive.followTrajectorySync(trajectoriesA.get(3));

            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);

            robot.drive.followTrajectorySync(trajectoriesA.get(4));

            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.CLOSE;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.UP;
            robot.sleep(0.2);

            robot.drive.followTrajectorySync(trajectoriesA.get(5));

            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);
            robot.intake.intakeMode = Intake.IntakeMode.IN;
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.UP;

            robot.drive.followTrajectorySync(trajectoriesA.get(6));

            robot.sleep(0.5);
            robot.outtake.outtakeTarget = Outtake.OuttakeTarget.HIGH_GOAL;
            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
            robot.outtake.outtakeMode = Outtake.OuttakeMode.ON;
            robot.sleep(0.5);
            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_ALL;
            robot.buffer.pushAttempts = 0;
            while(robot.buffer.bufferPusherMode != Buffer.BufferPusherMode.IDLE && robot.buffer.pushAttempts < 4) {
                robot.sleep(0.05);
                if(isStopRequested()) {
                    robot.stop();
                    return;
                }
            }
            robot.outtake.outtakeMode = Outtake.OuttakeMode.OFF;

            robot.drive.followTrajectorySync(trajectoriesA.get(7));
        }
        else if(ringStack == RingDetector.Stack.ONE) {
            // B
            robot.drive.followTrajectorySync(trajectoriesA.get(0));

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(0.75);

            robot.drive.followTrajectorySync(trajectoriesA.get(1));

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(0.75);

            robot.drive.followTrajectorySync(trajectoriesA.get(2));

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(0.75);

            robot.buffer.bufferMode = Buffer.BufferMode.COLLECT;
            robot.intake.intakeMode = Intake.IntakeMode.IN;
            robot.outtake.outtakeTarget = Outtake.OuttakeTarget.HIGH_GOAL;
            robot.ringStopper.ringStopperMode = RingStopper.RingStopperMode.INITIAL;

            robot.drive.followTrajectorySync(trajectoriesB.get(3));

            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);
            robot.intake.intakeMode = Intake.IntakeMode.IN;

            robot.drive.followTrajectorySync(trajectoriesB.get(4));

            robot.sleep(0.5);
            robot.outtake.outtakeTarget = Outtake.OuttakeTarget.HIGH_GOAL;
            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
            robot.outtake.outtakeMode = Outtake.OuttakeMode.ON;
            robot.sleep(0.5);
            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_ALL;
            robot.buffer.pushAttempts = 0;
            while(robot.buffer.bufferPusherMode != Buffer.BufferPusherMode.IDLE && robot.buffer.pushAttempts < 4) {
                robot.sleep(0.05);
                if(isStopRequested()) {
                    robot.stop();
                    return;
                }
            }
            robot.buffer.bufferMode = Buffer.BufferMode.COLLECT;

            robot.drive.followTrajectorySync(trajectoriesB.get(5));

            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.CLOSE;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.UP;
            robot.sleep(0.2);
            robot.intake.intakeMode = Intake.IntakeMode.IN;

            robot.drive.followTrajectorySync(trajectoriesB.get(6));

            robot.sleep(1.5);
            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
            robot.sleep(0.5);
            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_ALL;
            robot.buffer.pushAttempts = 0;
            while(robot.buffer.bufferPusherMode != Buffer.BufferPusherMode.IDLE && robot.buffer.pushAttempts < 4) {
                robot.sleep(0.05);
                if(isStopRequested()) {
                    robot.stop();
                    return;
                }
            }
            robot.outtake.outtakeMode = Outtake.OuttakeMode.OFF;
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;

            robot.drive.followTrajectorySync(trajectoriesB.get(7));

            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.UP;

            robot.drive.followTrajectorySync(trajectoriesB.get(8));
        }
        else if(ringStack == RingDetector.Stack.FOUR) {
            // C
            robot.drive.followTrajectorySync(trajectoriesC.get(0));

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(0.5);

            robot.drive.followTrajectorySync(trajectoriesC.get(1));

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(0.5);

            robot.drive.followTrajectorySync(trajectoriesC.get(2));

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(0.5);

            robot.buffer.bufferMode = Buffer.BufferMode.COLLECT;
            robot.outtake.outtakeTarget = Outtake.OuttakeTarget.HIGH_GOAL;
            robot.intake.intakeMode = Intake.IntakeMode.IN;
            robot.ringStopper.ringStopperMode = RingStopper.RingStopperMode.INITIAL;

            robot.drive.followTrajectorySync(trajectoriesC.get(3));

            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);

            robot.drive.followTrajectorySync(trajectoriesC.get(4));

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

            robot.drive.followTrajectorySync(trajectoriesC.get(5));

            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.CLOSE;
            robot.sleep(0.3);
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.UP;
            robot.sleep(0.2);
            robot.intake.intakeMode = Intake.IntakeMode.IN;

            robot.drive.followTrajectorySync(trajectoriesC.get(6));

            robot.intake.intakeMode = Intake.IntakeMode.IN;

            robot.drive.followTrajectory(trajectoriesC.get(7));

            ElapsedTime collectTimer = new ElapsedTime();
            while(robot.drive.isBusy()) {
                if(robot.buffer.getRingCount() >= 2) {
                    if(collectTimer.seconds() > 0.4) {
                        robot.intake.intakeMode = Intake.IntakeMode.IN_SLOW;
                        robot.drive.cancelTrajectory();
                    }
                }
                else {
                    collectTimer.reset();
                }
                if(robot.buffer.getRingCount() >= 3) {
                    robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
                }
            }

            if(robot.intake.intakeMode != Intake.IntakeMode.IN_SLOW) {
                robot.intake.intakeMode = Intake.IntakeMode.OUT;
                robot.sleep(0.2);
            }
            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
            robot.intake.intakeMode = Intake.IntakeMode.IN_SLOW;
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

            robot.drive.followTrajectorySync(trajectoriesC.get(8));

            robot.sleep(1.3);
            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
            robot.sleep(0.1);
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

            robot.drive.followTrajectorySync(trajectoriesC.get(9));

            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.1);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.1);

            robot.drive.followTrajectorySync(trajectoriesC.get(10));
        }

        robot.sleep(0.2);

        robot.stop();
    }
}