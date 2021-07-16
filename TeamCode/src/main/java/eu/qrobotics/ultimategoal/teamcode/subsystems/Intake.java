package eu.qrobotics.ultimategoal.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake implements Subsystem {
    public enum IntakeMode {
        IN,
        IN_SLOW,
        IDLE,
        OUT,
        OUT_SLOW,
    }
    public enum IntakeStopperMode {
        DOWN,
        MID,
        UP,
    }

    public static double INTAKE_IN_SPEED = 1;
    public static double INTAKE_IN_SLOW_SPEED = 0.3;
    public static double INTAKE_IDLE_SPEED = 0;
    public static double INTAKE_OUT_SPEED = -0.8;
    public static double INTAKE_OUT_SLOW_SPEED = -0.4;

    public static double STOPPER_DOWN_POSITION = 0.905;
    public static double STOPPER_MID_POSITION = 0.4;
    public static double STOPPER_UP_POSITION = 0.2;

    public IntakeMode intakeMode;
    public IntakeStopperMode intakeStopperMode;

    private DcMotor intakeMotor;
    private Servo intakeStopper;

    private Robot robot;

    public Intake(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeStopper = hardwareMap.get(Servo.class, "intakeServo");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMode = IntakeMode.IDLE;
        intakeStopperMode = IntakeStopperMode.UP;
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
        switch (intakeStopperMode) {
            case DOWN:
                intakeStopper.setPosition(STOPPER_DOWN_POSITION);
                break;
            case MID:
                intakeStopper.setPosition(STOPPER_MID_POSITION);
                break;
            case UP:
                intakeStopper.setPosition(STOPPER_UP_POSITION);
                break;
        }
    }
}
