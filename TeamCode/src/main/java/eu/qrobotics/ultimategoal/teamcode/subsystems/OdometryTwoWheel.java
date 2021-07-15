package eu.qrobotics.ultimategoal.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import java.util.Arrays;
import java.util.List;

import eu.qrobotics.ultimategoal.teamcode.util.AxesSigns;
import eu.qrobotics.ultimategoal.teamcode.util.BNO055IMUUtil;
import eu.qrobotics.ultimategoal.teamcode.util.Encoder;

@Config
public class OdometryTwoWheel extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static Pose2d FRONT_POSE = new Pose2d(1.6873,  1.7097, Math.toRadians(-90));
    public static Pose2d MIDDLE_POSE = new Pose2d(-4.2544, -2.7047, 0);

    private Encoder frontEncoder, middleEncoder;
    private BNO055IMU imu;

    public static double MULTIPLIER_X = 1;
    public static double MULTIPLIER_Y = 1;

    public OdometryTwoWheel(HardwareMap hardwareMap) {
        super(Arrays.asList(
                FRONT_POSE, // left
                MIDDLE_POSE // right
        ));

        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        middleEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));

        frontEncoder.setDirection(Encoder.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
    }

    @Override
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * MULTIPLIER_Y,
                encoderTicksToInches(middleEncoder.getCurrentPosition()) * MULTIPLIER_X
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * MULTIPLIER_Y,
                encoderTicksToInches(middleEncoder.getCorrectedVelocity()) * MULTIPLIER_X
        );
    }
}
