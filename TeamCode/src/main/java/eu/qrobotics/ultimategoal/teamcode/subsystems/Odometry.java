package eu.qrobotics.ultimategoal.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

import eu.qrobotics.ultimategoal.teamcode.util.Encoder;

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

    public static Pose2d FRONT_POSE = new Pose2d(1.7173,  1.7097, Math.toRadians(-90));
    public static Pose2d MIDDLE_POSE = new Pose2d(-4.2544, -2.4547, 0);
    public static Pose2d REAR_POSE = new Pose2d(-7.1765, -2.8346, Math.toRadians(-90));

    public static double X_MULTIPLIER = 1;
    public static double Y_MULTIPLIER = 1;

    private Encoder frontEncoder, middleEncoder, rearEncoder;

    public Odometry(HardwareMap hardwareMap) {
        super(Arrays.asList(
                FRONT_POSE, // left
                MIDDLE_POSE, // right
                REAR_POSE // front
        ));

        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        middleEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rearEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));

        frontEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(middleEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rearEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(middleEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rearEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}
