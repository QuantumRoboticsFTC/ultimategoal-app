package eu.qrobotics.ultimategoal.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class RingStopper implements Subsystem {
    public enum RingStopperMode {
        DOWN,
        UP,
        INITIAL
    }

    public static double ARM_DOWN_POSITION = 0.62;
    public static double ARM_UP_POSITION = 0.20;
    public static double ARM_INITIAL_POSITION = 0.20;
    public boolean STARTED = false;
    public RingStopperMode ringStopperMode;

    private Servo ringStopperServo;
    public RingStopper(HardwareMap hardwareMap, Robot robot) {
        ringStopperServo = hardwareMap.get(Servo.class, "ringStopperServo");

        ringStopperMode = RingStopperMode.INITIAL;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if(IS_DISABLED) return;
            
        switch (ringStopperMode) {
            case DOWN:
                ringStopperServo.setPosition(ARM_DOWN_POSITION);
                break;
            case UP:
                ringStopperServo.setPosition(ARM_UP_POSITION);
                break;
            case INITIAL:
                ringStopperServo.setPosition(ARM_INITIAL_POSITION);
                break;
        }
    }
}
