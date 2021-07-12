package eu.qrobotics.ultimategoal.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Buffer implements Subsystem {
    public enum BufferMode {
        DOWN,
        COLLECT,
        OUTTAKE,
    }

    public enum BufferPusherMode {
        IDLE,
        PUSH_SINGLE,
        PUSH_ALL,
    }

    private enum BufferPusherState {
        IDLE,
        PUSHING,
        RETRACTING,
    }

    public static double BUFFER_DOWN_POSITION = 0.97;
    public static double BUFFER_RING1_POSITION = 0.97;
    public static double BUFFER_RING2_POSITION = 0.97;
    public static double BUFFER_RING3_POSITION = 0.97;
    public static double BUFFER_OUTTAKE_POSITION = 0.865;

    public static double BUFFER_PUSHER_IDLE_POSITION = 0.575;
    public static double BUFFER_PUSHER_PUSH_POSITION = 0.7 ;

    public BufferMode bufferMode;
    public BufferPusherMode bufferPusherMode;
    private BufferPusherState bufferPusherState;
    private ElapsedTime bufferPushTime = new ElapsedTime();

    public BufferPusherState getBufferPusherState() {
        return bufferPusherState;
    }

    private Servo bufferServo;
    private Servo bufferPusherServo;
    private ColorRangeSensor bufferRingSensor;

    private Robot robot;
    private boolean isAutonomous;

    public MovingStatistics ringSensorValues;

    public Buffer(HardwareMap hardwareMap, Robot robot, boolean isAutonomous) {
        this.robot = robot;
        this.isAutonomous = isAutonomous;

        bufferServo = hardwareMap.get(Servo.class, "bufferServo");
        bufferPusherServo = hardwareMap.get(Servo.class, "bufferPusherServo");
        bufferRingSensor = hardwareMap.get(ColorRangeSensor.class, "bufferRingSensor");

        bufferMode = BufferMode.DOWN;
        bufferPusherMode = BufferPusherMode.IDLE;
        bufferPusherState = BufferPusherState.IDLE;

        ringSensorValues = new MovingStatistics(isAutonomous ? 16 : 4);
        ringSensorValues.add(bufferRingSensor.getDistance(DistanceUnit.MM));
    }

    public double lastSensorTime = 0;

    public static boolean SENSOR_ENABLE = true;
    public static double SENSOR_WAIT = 0;

    private ElapsedTime sensorTime = new ElapsedTime();

    public MovingStatistics avgSensorTime1 = new MovingStatistics(1);
    public MovingStatistics avgSensorTime10 = new MovingStatistics(10);
    public MovingStatistics avgSensorTime250 = new MovingStatistics(250);

    private static double getCurrentTime() {
        return System.nanoTime() / 1_000_000_000.0;
    }

    public static boolean IS_DISABLED = false;
    public double lastDistance = 0;

    public int pushAttempts = 0;

    private ElapsedTime bufferUpTimer = new ElapsedTime();

    @Override
    public void update() {
        if(IS_DISABLED) return;
        double start = getCurrentTime();
        double distance = -1;
        if(SENSOR_ENABLE) {
            if(sensorTime.milliseconds() > SENSOR_WAIT) {
                sensorTime.reset();
                distance = bufferRingSensor.getDistance(DistanceUnit.MM);
            }
        }
        lastDistance = distance;
        lastSensorTime = getCurrentTime() - start;
        avgSensorTime1.add(lastSensorTime);
        avgSensorTime10.add(lastSensorTime);
        avgSensorTime250.add(lastSensorTime);
        if(distance >= 0) {
            ringSensorValues.add(distance);
        }
        switch (bufferMode) {
            case DOWN:
                bufferServo.setPosition(BUFFER_DOWN_POSITION);
                break;
            case COLLECT:
                if(getRingCount() == 0) {
                    bufferServo.setPosition(BUFFER_RING1_POSITION);
                }
                else if(getRingCount() == 1) {
                    bufferServo.setPosition(BUFFER_RING2_POSITION);
                }
                else {
                    bufferServo.setPosition(BUFFER_RING3_POSITION);
                }
                break;
            case OUTTAKE:
                bufferServo.setPosition(BUFFER_OUTTAKE_POSITION);
                break;
        }

        if (bufferMode != BufferMode.OUTTAKE) {
            bufferPusherMode = BufferPusherMode.IDLE;
            bufferPusherState = BufferPusherState.IDLE;
            bufferUpTimer.reset();
        }
        if (getRingCount() == 0 && !(isAutonomous && bufferPusherMode == BufferPusherMode.PUSH_SINGLE)) {
            bufferPusherMode = BufferPusherMode.IDLE;
        }

        switch (bufferPusherMode) {
            case PUSH_SINGLE:
            case PUSH_ALL:
                if(bufferPusherState == BufferPusherState.IDLE && bufferUpTimer.seconds() > 0.5 && robot.outtake.isReady()) {
                    push();
                }
                break;
            default:
                break;
        }

        switch (bufferPusherState) {
            case IDLE:
                bufferPusherServo.setPosition(BUFFER_PUSHER_IDLE_POSITION);
                break;
            case PUSHING:
                bufferPusherServo.setPosition(BUFFER_PUSHER_PUSH_POSITION);
                if (bufferPushTime.seconds() > 0.1) {
                    bufferPusherState = BufferPusherState.RETRACTING;
                }
                break;
            case RETRACTING:
                bufferPusherServo.setPosition(BUFFER_PUSHER_IDLE_POSITION);
                if (bufferPushTime.seconds() > 0.1 * 2) {
                    bufferPusherState = BufferPusherState.IDLE;

                    pushAttempts++;

                    if (bufferPusherMode == BufferPusherMode.PUSH_SINGLE) {
                        bufferPusherMode = BufferPusherMode.IDLE;
                    }
                }
                break;
        }
    }

    private void push() {
        if (bufferPusherState == BufferPusherState.IDLE) {
            bufferPusherState = BufferPusherState.PUSHING;
            bufferPushTime.reset();
        }
    }

    public int getRingCount() {
        double mean = ringSensorValues.getMean();
        double stdev = ringSensorValues.getStandardDeviation() / 2;

        double distance = mean + (Double.isNaN(stdev) ? 0 : stdev);
        if(distance > 108) {
            return 0;
        }
        if(distance > 95) {
            return 1;
        }
        if(distance > 85) {
            return 2;
        }
        return 3;
    }
}
