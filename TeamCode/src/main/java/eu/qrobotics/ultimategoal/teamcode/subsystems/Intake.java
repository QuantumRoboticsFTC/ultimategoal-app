package eu.qrobotics.ultimategoal.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake implements Subsystem {
    public enum IntakeMode {
        IN,
        IN_SLOW,
        IDLE,
        OUT,
        OUT_SLOW,
    }

    public static double INTAKE_IN_SPEED = 1;
    public static double INTAKE_IN_SLOW_SPEED = 0.3;
    public static double INTAKE_IDLE_SPEED = 0;
    public static double INTAKE_OUT_SPEED = -0.8;
    public static double INTAKE_OUT_SLOW_SPEED = -0.4;

    public IntakeMode intakeMode;

    private DcMotor intakeMotor;

    private Robot robot;

    public Intake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMode = IntakeMode.IDLE;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if(IS_DISABLED) return;
        if(robot.buffer.bufferMode != Buffer.BufferMode.COLLECT && intakeMode == IntakeMode.IN) {
            intakeMode = IntakeMode.IDLE;
        }
        switch (intakeMode) {
            case IN:
                intakeMotor.setPower(INTAKE_IN_SPEED);
                break;
            case IN_SLOW:
                intakeMotor.setPower(INTAKE_IN_SLOW_SPEED);
                break;
            case IDLE:
                intakeMotor.setPower(INTAKE_IDLE_SPEED);
                break;
            case OUT:
                intakeMotor.setPower(INTAKE_OUT_SPEED);
                break;
            case OUT_SLOW:
                intakeMotor.setPower(INTAKE_OUT_SLOW_SPEED);
                break;
        }
    }
}
