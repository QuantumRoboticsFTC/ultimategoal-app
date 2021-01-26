package eu.qrobotics.ultimategoal.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
    public static double BUFFER_RING1_POSITION = 0.57;
    public static double BUFFER_RING2_POSITION = 0.59;
    public static double BUFFER_RING3_POSITION = 0.61;
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
    private DistanceSensor bufferRingSensor;

    private Robot robot;

    public Buffer(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        bufferServo = hardwareMap.get(Servo.class, "bufferServo");
        bufferPusherServo = hardwareMap.get(Servo.class, "bufferPusherServo");
        bufferRingSensor = hardwareMap.get(DistanceSensor.class, "bufferRingSensor");

        bufferMode = BufferMode.DOWN;
        bufferPusherMode = BufferPusherMode.IDLE;
        bufferPusherState = BufferPusherState.IDLE;
    }

    @Override
    public void update() {
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
        double distance = bufferRingSensor.getDistance(DistanceUnit.MM);
        if(distance > 110) {
            return 0;
        }
        if(distance > 90) {
            return 1;
        }
        if(distance > 65) {
            return 2;
        }
        return 3;
    }
}
