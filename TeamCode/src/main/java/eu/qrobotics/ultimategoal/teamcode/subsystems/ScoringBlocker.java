package eu.qrobotics.ultimategoal.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ScoringBlocker implements Subsystem {
    public static double BLOCKER_MIN_POSITION = 0;
    public static double BLOCKER_MAX_POSITION = 1;
    public static double BLOCKER_SIDE_POSITION = 0.33;

    private Servo scoringBlockerServo;

    public double blockerPosition;
    private double prevBlockerPosition = -1;

    public ScoringBlocker(HardwareMap hardwareMap, Robot robot) {
        scoringBlockerServo = hardwareMap.get(Servo.class, "scoringBlockerServo");

        blockerPosition = BLOCKER_MIN_POSITION;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if(IS_DISABLED) return;

        blockerPosition = Math.max(BLOCKER_MIN_POSITION, Math.min(BLOCKER_MAX_POSITION, blockerPosition));

        if (blockerPosition != prevBlockerPosition) {
            scoringBlockerServo.setPosition(blockerPosition);
            prevBlockerPosition = blockerPosition;
        }
    }

    public void moveOut() {
        blockerPosition = Math.max(BLOCKER_SIDE_POSITION, blockerPosition);
    }

    public void moveIn() {
        blockerPosition = BLOCKER_MIN_POSITION;
    }
}
