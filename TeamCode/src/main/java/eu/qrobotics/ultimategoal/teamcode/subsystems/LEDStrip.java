package eu.qrobotics.ultimategoal.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class LEDStrip implements Subsystem {

    private RevBlinkinLedDriver blinkinLedDriver;
    private Robot robot;

    public LEDStrip(HardwareMap hardwareMap, Robot robot) {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "ledStrip");
        this.robot = robot;
    }

    public static boolean IS_DISABLED = false;

    private RevBlinkinLedDriver.BlinkinPattern lastPattern;
    private void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if(pattern != lastPattern) {
            blinkinLedDriver.setPattern(pattern);
            lastPattern = pattern;
        }
    }

    @Override
    public void update() {
        if(IS_DISABLED) return;

        boolean turretOutOfRange = !robot.outtake.isTurretInRange();
        int ringCount = robot.buffer.getRingCount();

        if(turretOutOfRange) {
            if(ringCount == 0) {
                setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
            }
            else if(ringCount == 1) {
                setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
            }
            else if(ringCount == 2) {
                setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
            }
            else if(ringCount == 3) {
                setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
            }
        }
        else {
            if(ringCount == 0) {
                setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
            }
            else if(ringCount == 1) {
                setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
            }
            else if(ringCount == 2) {
                setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
            }
            else if(ringCount == 3) {
                setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
            }
        }
    }
}
