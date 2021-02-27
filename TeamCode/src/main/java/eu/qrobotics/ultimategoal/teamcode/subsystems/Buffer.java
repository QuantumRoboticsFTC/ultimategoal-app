package eu.qrobotics.ultimategoal.teamcode.subsystems;

import eu.qrobotics.ultimategoal.teamcode.sensors.Rev2mDistanceSensorAsync;

import com.acmerobotics.dashboard.config.Config;
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

    public static double BUFFER_DOWN_POSITION = 0.65;
    public static double BUFFER_RING1_POSITION = 0.6;
    public static double BUFFER_RING2_POSITION = 0.58;
    public static double BUFFER_RING3_POSITION = 0.62;
    public static double BUFFER_OUTTAKE_POSITION = 0.515;

    public static double BUFFER_PUSHER_IDLE_POSITION = 1.0;
    public static double BUFFER_PUSHER_PUSH_POSITION = 0.55;

    public BufferMode bufferMode;
    public BufferPusherMode bufferPusherMode;
    private BufferPusherState bufferPusherState;
    private ElapsedTime bufferPushTime = new ElapsedTime();

    public BufferPusherState getBufferPusherState() {
        return bufferPusherState;
    }

    private Servo bufferServo;
    private Servo bufferPusherServo;
    private Rev2mDistanceSensorAsync bufferRingSensor;

    private Robot robot;

    private MovingStatistics ringSensorValues;

    public Buffer(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        bufferServo = hardwareMap.get(Servo.class, "bufferServo");
        bufferPusherServo = hardwareMap.get(Servo.class, "bufferPusherServo");
        bufferRingSensor = hardwareMap.get(Rev2mDistanceSensorAsync.class, "bufferRingSensor");

        bufferMode = BufferMode.DOWN;
        bufferPusherMode = BufferPusherMode.IDLE;
        bufferPusherState = BufferPusherState.IDLE;

        ringSensorValues = new MovingStatistics(2);
        ringSensorValues.add(bufferRingSensor.getDistance(DistanceUnit.MM));
    }

    public double lastSensorTime = 0;

    public static boolean SENSOR_ENABLE = true;
    public static double SENSOR_WAIT = 50;

    private ElapsedTime sensorTime = new ElapsedTime();

    public MovingStatistics avgSensorTime1 = new MovingStatistics(1);
    public MovingStatistics avgSensorTime10 = new MovingStatistics(10);
    public MovingStatistics avgSensorTime250 = new MovingStatistics(250);

    private static double getCurrentTime() {
        return System.nanoTime() / 1_000_000_000.0;
    }

    public static boolean IS_DISABLED = false;

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
                else if(getRingCount() == 2){
                    bufferServo.setPosition(BUFFER_RING3_POSITION);
                } else {
                    bufferMode = BufferMode.DOWN;
                }
                break;
            case OUTTAKE:
                bufferServo.setPosition(BUFFER_OUTTAKE_POSITION);
                break;
        }

        if (bufferMode != BufferMode.OUTTAKE) {
            bufferPusherMode = BufferPusherMode.IDLE;
            bufferPusherState = BufferPusherState.IDLE;
        }
        if (getRingCount() == 0) {
            bufferPusherMode = BufferPusherMode.IDLE;
        }

        switch (bufferPusherMode) {
            case PUSH_SINGLE:
            case PUSH_ALL:
                if(bufferPusherState == BufferPusherState.IDLE && robot.outtake.isReady()) {
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
                if (bufferPushTime.seconds() > 0.2) {
                    bufferPusherState = BufferPusherState.RETRACTING;
                }
                break;
            case RETRACTING:
                bufferPusherServo.setPosition(BUFFER_PUSHER_IDLE_POSITION);
                if (bufferPushTime.seconds() > 0.4) {
                    bufferPusherState = BufferPusherState.IDLE;

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
        double distance = ringSensorValues.getMean();
        if(distance > 120) {
            return 0;
        }
        if(distance > 95) {
            return 1;
        }
        if(distance > 70) {
            return 2;
        }
        return 3;
    }
}
