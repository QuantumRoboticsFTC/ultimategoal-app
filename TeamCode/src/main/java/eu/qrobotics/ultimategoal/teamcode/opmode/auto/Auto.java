package eu.qrobotics.ultimategoal.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import eu.qrobotics.ultimategoal.teamcode.subsystems.Buffer;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Intake;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Outtake;
import eu.qrobotics.ultimategoal.teamcode.subsystems.Robot;
import eu.qrobotics.ultimategoal.teamcode.subsystems.WobbleGoalGrabber;

import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.PARK_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.PARK_VEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.SLOW_ACCEL_CONSTRAINT;
import static eu.qrobotics.ultimategoal.teamcode.subsystems.DriveConstants.SLOW_VEL_CONSTRAINT;

@Config
@Autonomous
public class Auto extends LinearOpMode {
    private static final String VUFORIA_KEY =
            "AZkUdjT/////AAABmXTO5InzOk5Yo1OlXRo+EFQkVF1qqCBnur0y1G+RBktOx7nVuzRvRaahrrHE0OJTAUwmyuPkGSbIFETtV9VN5Ezo8vTtN90u2lqAMZx5ZY5qWtTs+rm/2y4CctYrNhnxeme+qRNeRj6gKhUMa2FAVHr2qBtJK/CrZ0Ud/1vpLavIr+TrHuIjABcEXRyXBcdIaj5gw4EiVChCFrjv24qMiHuOq1pHOAbpTqe392045VnPLDlJ6bJKq0cNZ3TR86ccLGd2Pg0lnVLvf/qthVFRy8NASoyrgQkEU0P5WSC+8A3IlWPPEgG2LMu8FACw+6t1da+EiznSyu5dSW7UcAw5oHKpGgfxRV3pXmNJ3bn+AfMi";

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

//    private MultipleTelemetry telemetry;

    @Override
    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this, true);
        robot.drive.setPoseEstimate(Trajectories.START_POSE);
        List<Trajectory> trajectoriesA = Trajectories.getTrajectoriesA();
        List<Trajectory> trajectoriesB = Trajectories.getTrajectoriesB();
        List<Trajectory> trajectoriesC = Trajectories.getTrajectoriesC();
        robot.start();

        telemetry.addLine("Driver 1, press A to close WG Grabber");
        telemetry.update();

//        RingDetector ringDetector = new RingDetector(hardwareMap, "Webcam 1");
//        ringDetector.init();
//        ringDetector.setBottomRectangle(0.25, 0.6);
//        ringDetector.setTopRectangle(0.35, 0.6);

        RingDetector.Stack ringStack = RingDetector.Stack.FOUR;


        initVuforia();
        initTfod();
        FtcDashboard.getInstance().startCameraStream(tfod, 0);

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(3, 16.0/9.0);
        }

        while(!isStarted()) {
            if(gamepad1.a) {
                robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.CLOSE;
            }

            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() == 0) {
                        ringStack = RingDetector.Stack.ZERO;
                    } else {
                        if (updatedRecognitions.get(0).getLabel().equals("Single")) {
                            ringStack = RingDetector.Stack.ONE;
                        } else {
                            ringStack = RingDetector.Stack.FOUR;
                        }
                    }
                }
            }
//            ringStack = ringDetector.getStack();
            telemetry.addData("Rings", ringStack);
            telemetry.update();
        }
//        ringDetector.close();

        if(isStopRequested()) {
            robot.stop();
            return;
        }
        resetStartTime();

        robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.CLOSE;

        robot.outtake.outtakeTarget = Outtake.OuttakeTarget.POWER_SHOT;
        robot.outtake.outtakeMode = Outtake.OuttakeMode.ON;
        robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;

        if(ringStack == RingDetector.Stack.ZERO) {
            // A
            robot.drive.followTrajectorySync(trajectoriesA.get(0));

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(1);

            robot.drive.followTrajectorySync(trajectoriesA.get(1));

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(1);

            robot.drive.followTrajectorySync(trajectoriesA.get(2));

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(1);

            /*
            robot.drive.followTrajectorySync(trajectoriesA.get(0));

            robot.sleep(2);

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_ALL;
            while(robot.buffer.bufferPusherMode != Buffer.BufferPusherMode.IDLE) {
                robot.sleep(0.05);
                if(isStopRequested()) {
                    robot.stop();
                    return;
                }
            }
            */
            robot.outtake.outtakeMode = Outtake.OuttakeMode.OFF;

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

            robot.drive.followTrajectorySync(trajectoriesA.get(6));
        }
        else if(ringStack == RingDetector.Stack.ONE) {
            // B
            robot.drive.followTrajectorySync(trajectoriesA.get(0));

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(1);

            robot.drive.followTrajectorySync(trajectoriesA.get(1));

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(1);

            robot.drive.followTrajectorySync(trajectoriesA.get(2));

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(1);

            /*
            robot.drive.followTrajectorySync(trajectoriesB.get(0));

            robot.sleep(2);

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_ALL;
            while(robot.buffer.bufferPusherMode != Buffer.BufferPusherMode.IDLE) {
                robot.sleep(0.05);
                if(isStopRequested()) {
                    robot.stop();
                    return;
                }
            }
             */

            robot.buffer.bufferMode = Buffer.BufferMode.COLLECT;
            robot.intake.intakeMode = Intake.IntakeMode.IN;
            robot.outtake.outtakeTarget = Outtake.OuttakeTarget.HIGH_GOAL;

            robot.drive.followTrajectorySync(trajectoriesB.get(3));

            robot.sleep(1);
            robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
            robot.sleep(1);
            robot.intake.intakeMode = Intake.IntakeMode.IN;
            robot.sleep(2);
            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;
            robot.sleep(0.5);
            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            while(robot.buffer.bufferPusherMode != Buffer.BufferPusherMode.IDLE) {
                robot.sleep(0.05);
                if(isStopRequested()) {
                    robot.stop();
                    return;
                }
            }
            robot.outtake.outtakeMode = Outtake.OuttakeMode.OFF;

            robot.drive.followTrajectorySync(trajectoriesB.get(4));

            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);

            robot.drive.followTrajectorySync(trajectoriesB.get(5));

            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.CLOSE;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.UP;
            robot.sleep(0.2);

            robot.drive.followTrajectorySync(trajectoriesB.get(6));

            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.6);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);

            robot.drive.followTrajectorySync(trajectoriesB.get(7));

            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.UP;

            robot.sleep(0.5);
        }
        else if(ringStack == RingDetector.Stack.FOUR) {
            // C
            robot.drive.followTrajectorySync(trajectoriesC.get(0));

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(0.9);

            robot.drive.followTrajectorySync(trajectoriesC.get(1));

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(0.9);

            robot.drive.followTrajectorySync(trajectoriesC.get(2));

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_SINGLE;
            robot.sleep(0.9);
            /*
            robot.drive.followTrajectorySync(trajectoriesC.get(0));

            robot.sleep(2);

            robot.buffer.bufferPusherMode = Buffer.BufferPusherMode.PUSH_ALL;
            while(robot.buffer.bufferPusherMode != Buffer.BufferPusherMode.IDLE) {
                robot.sleep(0.05);
                if(isStopRequested()) {
                    robot.stop();
                    return;
                }
            }
            */

            robot.buffer.bufferMode = Buffer.BufferMode.COLLECT;
            robot.outtake.outtakeTarget = Outtake.OuttakeTarget.HIGH_GOAL;
            robot.intake.intakeMode = Intake.IntakeMode.OUT;

            robot.drive.followTrajectorySync(trajectoriesC.get(3));

            robot.intake.intakeMode = Intake.IntakeMode.IN;

            robot.drive.followTrajectory(trajectoriesC.get(4));

            while(robot.drive.isBusy()) {
                if(robot.buffer.getRingCount() >= 2) {
                    robot.intake.intakeMode = Intake.IntakeMode.IDLE;
                }
                if(robot.buffer.getRingCount() >= 3) {
                    robot.intake.intakeMode = Intake.IntakeMode.OUT;
                }
            }

            robot.sleep(1);
            robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
            robot.sleep(0.5);
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;
            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
            robot.sleep(0.1);
            robot.buffer.bufferMode = Buffer.BufferMode.COLLECT;
            robot.sleep(0.1);
            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
            robot.sleep(0.1);
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

            robot.drive.followTrajectorySync(trajectoriesC.get(5));

            robot.sleep(1);
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;
            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
            robot.sleep(0.1);
            robot.buffer.bufferMode = Buffer.BufferMode.COLLECT;
            robot.sleep(0.1);
            robot.buffer.bufferMode = Buffer.BufferMode.OUTTAKE;
            robot.sleep(0.1);
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

            robot.drive.followTrajectorySync(trajectoriesC.get(6));

            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.5);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);

            robot.drive.followTrajectorySync(trajectoriesC.get(7));

            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.CLOSE;
            robot.sleep(0.3);
            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.UP;
            robot.sleep(0.3);

            robot.drive.followTrajectorySync(trajectoriesC.get(8));

            robot.wobbleGoalGrabber.wobbleGoalArmMode = WobbleGoalGrabber.WobbleGoalArmMode.DOWN;
            robot.sleep(0.5);
            robot.wobbleGoalGrabber.wobbleGoalClawMode = WobbleGoalGrabber.WobbleGoalClawMode.OPEN;
            robot.sleep(0.2);

            robot.drive.followTrajectory(trajectoriesC.get(9));
            while(robot.drive.isBusy()) {
                if(getRuntime() > 29.8) {
                    robot.drive.followTrajectory(new TrajectoryBuilder(robot.drive.getPoseEstimate(), SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                            .forward(0.1)
                            .build());
                    break;
                }
            }
            if(getRuntime() < 29.8) {
                robot.drive.followTrajectorySync(new TrajectoryBuilder(robot.drive.getPoseEstimate(), PARK_VEL_CONSTRAINT, PARK_ACCEL_CONSTRAINT)
                        .lineToConstantHeading(new Vector2d(16, -40))
                        .build());
            }
        }

        robot.stop();
    }



    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
