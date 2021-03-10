package eu.qrobotics.ultimategoal.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class RingDetector {

    private OpenCvCamera camera;
    private boolean isUsingWebcam;
    private String webcamName;
    private HardwareMap hardwareMap;
    private RingPipeline ringPipeline;

    //The constructor is overloaded to allow the use of webcam instead of the phone camera
    public RingDetector(HardwareMap hMap) {
        hardwareMap = hMap;
    }

    public RingDetector(HardwareMap hMap, String webcamName) {
        hardwareMap = hMap;
        isUsingWebcam = true;
        this.webcamName = webcamName;
    }

    public void init() {
        //This will instantiate an OpenCvCamera object for the camera we'll be using
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        if (isUsingWebcam) {
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        //Set the pipeline the camera should use and start streaming
        camera.setPipeline(ringPipeline = new RingPipeline());
        camera.openCameraDeviceAsync(() -> camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));
    }

    public void close() {
        camera.closeCameraDeviceAsync(() -> {});
    }

    public void setTopRectangle(double topRectHeightPercentage, double topRectWidthPercentage) {
        ringPipeline.setTopRectHeightPercentage(topRectHeightPercentage);
        ringPipeline.setTopRectWidthPercentage(topRectWidthPercentage);
    }

    public void setBottomRectangle(double bottomRectHeightPercentage, double bottomRectWidthPercentage) {
        ringPipeline.setBottomRectHeightPercentage(bottomRectHeightPercentage);
        ringPipeline.setBottomRectWidthPercentage(bottomRectWidthPercentage);
    }

    public void setRectangleSize(int rectangleWidth, int rectangleHeight){
        ringPipeline.setRectangleHeight(rectangleHeight);
        ringPipeline.setRectangleWidth(rectangleWidth);
    }

    public Stack getStack() {
        if (Math.abs(ringPipeline.getTopAverage() - ringPipeline.getBottomAverage()) < ringPipeline.getThreshold() && (ringPipeline.getTopAverage() <= 100 && ringPipeline.getBottomAverage() <= 100)) {
            return Stack.FOUR;
        } else if (Math.abs(ringPipeline.getTopAverage() - ringPipeline.getBottomAverage()) < ringPipeline.getThreshold() && (ringPipeline.getTopAverage() >= 100 && ringPipeline.getBottomAverage() >= 100)) {
            return Stack.ZERO;
        } else {
            return Stack.ONE;
        }
    }

    public void setThreshold(int threshold) {
        ringPipeline.setThreshold(threshold);
    }

    public double getTopAverage() {
        return ringPipeline.getTopAverage();
    }

    public double getBottomAverage() {
        return ringPipeline.getBottomAverage();
    }

    public enum Stack {
        ZERO,
        ONE,
        FOUR,
    }

}