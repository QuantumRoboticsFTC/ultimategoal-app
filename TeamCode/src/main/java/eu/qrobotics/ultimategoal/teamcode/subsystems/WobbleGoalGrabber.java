package eu.qrobotics.ultimategoal.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class WobbleGoalGrabber implements Subsystem {
    public enum WobbleGoalArmMode {
        DOWN,
        UP,
        INITIAL,
    }

    public enum WobbleGoalClawMode {
        OPEN,
        CLOSE,
    }

    public static double ARM_DOWN_POSITION = 0;
    public static double ARM_UP_POSITION = 0.55;
    public static double ARM_INITIAL_POSITION = 0.7;

    public static double CLAW_OPEN_POSITION = 0.25;
    public static double CLAW_CLOSE_POSITION = 0.6;

    public WobbleGoalArmMode wobbleGoalArmMode;
    public WobbleGoalClawMode wobbleGoalClawMode;

    private Servo wobbleGoalArmServo;
    private Servo wobbleGoalClawServo;

    public WobbleGoalGrabber(HardwareMap hardwareMap, Robot robot) {
        wobbleGoalArmServo = hardwareMap.get(Servo.class, "wobbleGoalArmServo");
        wobbleGoalClawServo = hardwareMap.get(Servo.class, "wobbleGoalClawServo");

        wobbleGoalArmMode = WobbleGoalArmMode.INITIAL;
        wobbleGoalClawMode = WobbleGoalClawMode.OPEN;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if(IS_DISABLED) return;
        switch (wobbleGoalArmMode) {
            case DOWN:
                wobbleGoalArmServo.setPosition(ARM_DOWN_POSITION);
                break;
            case UP:
                wobbleGoalArmServo.setPosition(ARM_UP_POSITION);
                break;
            case INITIAL:
                wobbleGoalArmServo.setPosition(ARM_INITIAL_POSITION);
                break;
        }

        switch (wobbleGoalClawMode) {
            case OPEN:
                wobbleGoalClawServo.setPosition(CLAW_OPEN_POSITION);
                break;
            case CLOSE:
                wobbleGoalClawServo.setPosition(CLAW_CLOSE_POSITION);
                break;
        }
    }
}
