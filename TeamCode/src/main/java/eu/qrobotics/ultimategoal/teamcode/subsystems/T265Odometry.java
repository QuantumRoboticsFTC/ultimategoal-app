package eu.qrobotics.ultimategoal.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.io.File;

import static java.lang.Math.PI;

@SuppressWarnings("FieldCanBeLocal")
public class T265Odometry implements Localizer {

    // Electronics
    private static T265Camera t265Cam;

    // Constants
    public final double ODOMETRY_COVARIANCE = 0.1;
    private final double INCH_TO_METER = 0.0254;
    private final double xOffset = -9;
    private final double yOffset = 2;

    // State Variables
    private double x, y, theta;
    public int confidence = 0;

    @SuppressLint("SdCardPath")
    private final String mapPath = "/data/user/0/com.qualcomm.ftcrobotcontroller/cache/map.bin";
    public boolean isEmpty = false;
    private boolean exportingMap = true;

    public T265Odometry(HardwareMap hardwareMap) {
        File file = new File(mapPath);
        if (!file.exists() || file.length() == 0) {
            isEmpty = true;
        }

//        Robot.log(isEmpty ? "T265 Localization Map Not Found" : "Found T265 Localization Map");
//        Robot.log(t265Cam == null ? "Instantiating new T265" : "Using static T265");

        if (t265Cam == null) {
            if (!isEmpty) {
                t265Cam = new T265Camera(new Transform2d(), ODOMETRY_COVARIANCE, mapPath, hardwareMap.appContext);
            } else {
                t265Cam = new T265Camera(new Transform2d(), ODOMETRY_COVARIANCE, hardwareMap.appContext);
            }
        }
        if(!t265Cam.isStarted()) {
            t265Cam.start();
        }
        setCameraPose(0, 0, 0);
    }

    public void startCam() {
        t265Cam.start();
    }

    public void exportMap() {
        if (exportingMap) {
            exportingMap = false;
            t265Cam.exportRelocalizationMap(mapPath);
        }
    }

    public void stopCam() {
        t265Cam.stop();
    }

    public void setCameraPose(double x, double y, double theta) {
        x -= -xOffset * Math.sin(theta) - yOffset * Math.cos(theta);
        y -= xOffset * Math.cos(theta) - yOffset * Math.sin(theta);

        t265Cam.setPose(new Pose2d(y * INCH_TO_METER, -x * INCH_TO_METER, new Rotation2d(theta)));
    }

    public void sendOdometryData(double vx, double vy, double theta, double w) {
        double r = Math.hypot(xOffset, yOffset);
        theta += Math.atan2(yOffset, xOffset) - PI/2;
        t265Cam.sendOdometry(vy + r * w * Math.sin(theta), -vx - r * w * Math.cos(theta));
    }

    public void updateCamPose() {
        T265Camera.CameraUpdate state = t265Cam.getLastReceivedCameraUpdate();

        Translation2d translation = new Translation2d(state.pose.getTranslation().getX() / INCH_TO_METER, state.pose.getTranslation().getY() / INCH_TO_METER);
        Rotation2d rotation = state.pose.getRotation();

        x = -translation.getY() - xOffset * Math.sin(theta) - yOffset * Math.cos(theta);
        y = translation.getX() + xOffset * Math.cos(theta) - yOffset * Math.sin(theta);
        theta = rotation.getRadians();

        if (state.confidence == T265Camera.PoseConfidence.High) {
            confidence = 3;
        } else if (state.confidence == T265Camera.PoseConfidence.Medium) {
            confidence = 2;
        } else if (state.confidence == T265Camera.PoseConfidence.Low) {
            confidence = 1;
        } else {
            confidence = 0;
        }
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public String confidenceColor() {
        switch (confidence) {
            case 3:
                return "green";
            case 2:
                return "yellow";
            case 1:
                return "orange";
            default :
                return "red";
        }
    }

    @NotNull
    @Override
    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseEstimate() {
        return new com.acmerobotics.roadrunner.geometry.Pose2d(x, y, theta);
    }

    @Override
    public void setPoseEstimate(@NotNull com.acmerobotics.roadrunner.geometry.Pose2d pose2d) {
        setCameraPose(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }

    @Nullable
    @Override
    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        updateCamPose();
    }

    public T265Camera.CameraUpdate getCameraUpdate() {
        return t265Cam.getLastReceivedCameraUpdate();
    }
}