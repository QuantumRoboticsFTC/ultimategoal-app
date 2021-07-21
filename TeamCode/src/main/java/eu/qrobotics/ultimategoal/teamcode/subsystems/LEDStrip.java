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

    @Override
    public void update() {
        if(IS_DISABLED) return;

        boolean notInLaunchZone = robot.drive.getPoseEstimate().getX() > 8;
        boolean turretOutOfRange = !robot.outtake.isTurretInRange();

        if(notInLaunchZone && turretOutOfRange)
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        else if(notInLaunchZone)
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        else if(turretOutOfRange)
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        else
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }
}
