package eu.qrobotics.ultimategoal.teamcode.opmode.auto.cv;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvTrackerApiPipeline;

import java.util.Arrays;

@Config
public class RingDetector {

    private OpenCvCamera camera;
    private OpenCvTrackerApiPipeline trackerApiPipeline;
    private RingTracker ringTracker;
    private Point pt1, pt2;

    public static double THRESHOLD_FOUR = 4500000;
    public static double THRESHOLD_ONE = 1000000;

    public RingDetector(OpenCvCamera camera, Point pt1, Point pt2) {
        this.camera = camera;
        this.pt1 = pt1;
        this.pt2 = pt2;

        this.ringTracker = new RingTracker(pt1, pt2);
        this.trackerApiPipeline = new OpenCvTrackerApiPipeline();
        this.trackerApiPipeline.addTracker(ringTracker);
        camera.setPipeline(trackerApiPipeline);
    }

    public double getCount() {
        return average(this.ringTracker.getCount());
    }

    public Stack getStack() {
        double count = getCount();
        if (count > THRESHOLD_FOUR) {
            return Stack.FOUR;
        } else if (count > THRESHOLD_ONE) {
            return Stack.ONE;
        } else {
            return Stack.ZERO;
        }
    }

    public enum Stack {
        ZERO,
        ONE,
        FOUR,
    }

    @SuppressLint("NewApi")
    private double average(Scalar s) {
        if (s == null || s.val == null)
            return 0;
        return Arrays.stream(s.val).average().orElse(0);
    }
}