package eu.qrobotics.ultimategoal.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class Odometry extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public int leftEncoderBase;
    public int rightEncoderBase;
    public int frontEncoderBase;

    private DcMotor leftEncoder, rightEncoder, frontEncoder;

    public Odometry(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(-1.811, 7.755, 0), // left
                new Pose2d(-1.811, -7.755, 0), // right
                new Pose2d(-7.592, -1.496, Math.toRadians(90)) // front
        ));

        frontEncoder = hardwareMap.get(DcMotor.class, "rightFront");
        leftEncoder = hardwareMap.get(DcMotor.class, "leftRear");
        rightEncoder = hardwareMap.get(DcMotor.class, "leftFront");

        leftEncoderBase = leftEncoder.getCurrentPosition();
        rightEncoderBase = rightEncoder.getCurrentPosition();
        frontEncoderBase = frontEncoder.getCurrentPosition();
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(-(leftEncoder.getCurrentPosition() - leftEncoderBase)),
                encoderTicksToInches(-(rightEncoder.getCurrentPosition() - rightEncoderBase)),
                encoderTicksToInches(-(frontEncoder.getCurrentPosition() - frontEncoderBase))
        );
    }
}
