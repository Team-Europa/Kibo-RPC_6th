package jp.jaxa.iss.kibo.rpc.defaultapk;

import java.util.HashMap;
import java.util.Map;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.defaultapk.model.PointWithQuaternion;

public class Constants {
    static final PointWithQuaternion astronautPQ =
            new PointWithQuaternion(
                    new Point(11.143, -6.6707, 4.9654),
                    new Quaternion(0f, 0f, 0.707f, 0.707f));

    static final int visionThread_stoppingLatency = 20;
    static final long scanSleepMillis1 = 5000;
    static final long scanSleepMillis23 = 5000;
    static final long scanSleepMillis4 = 5000;

    public enum Cam {NAV, DOCK}

    static final PointWithQuaternion area1 = new PointWithQuaternion(
            new Point(10.925, -9.525,4.685),
            new Quaternion(0.5f, 0.5f, 0.5f, 0.5f));

    static final PointWithQuaternion area23 = new PointWithQuaternion(
            new Point(10.725, -8.275, 5.375),
            new Quaternion(-0.5f, 0.5f, 0.5f, 0.5f)
    );

    static final PointWithQuaternion area4 = new PointWithQuaternion(
            new Point(11.410, -6.875,4.685),
            new Quaternion(0.707f, 0f, 0f, 0.707f));

    static final org.opencv.core.Point area1Origin = new org.opencv.core.Point(4.82, 10.42);
    static final org.opencv.core.Point area2_3Origin = new org.opencv.core.Point(11.5, -9.25);
    static final org.opencv.core.Point area4Origin = new org.opencv.core.Point(5.57, -6.365);

    static final org.opencv.core.Point area1Max = new org.opencv.core.Point(5.57, 11.48);
    static final org.opencv.core.Point area2_3Max = new org.opencv.core.Point(10.3,-7.45);
    static final org.opencv.core.Point area4Max = new org.opencv.core.Point(4.32, -7.34);

    static final Double[] navCamDistFromCenter = {-0.0422, -0.0826, 0.1177};
    static final Double[] dockCamDistFromCenter = {-0.054, -0.0064, 0.1061};

    private static final double distanceToArea = 0.8;
    static final double targetZ_area23 = 3.76093 + distanceToArea;
    static final double targetY_area1 = -10.58 + distanceToArea;
    static final double targetX_area4 = 9.866984 + distanceToArea;
    static final PointWithQuaternion targetPQ_area4 = new PointWithQuaternion(new Point(targetX_area4, -6.8525, 4.945), new Quaternion(0f,0f,-1f,0f));
    static final PointWithQuaternion targetPQ_area3 = new PointWithQuaternion(new Point(10.925, -7.925, targetZ_area23), new Quaternion(0.5f, -0.5f, -0.5f, -0.5f));
    static final PointWithQuaternion targetPQ_area2 = new PointWithQuaternion(new Point(10.925, -8.875, targetZ_area23), new Quaternion(0.5f, -0.5f, -0.5f, -0.5f));
    static final PointWithQuaternion targetPQ_area1 = new PointWithQuaternion(new Point(10.95, targetY_area1, 5.195), new Quaternion(0f,0f,-1f,0f));

    public static final Map<Integer, Integer> TREASURE_MAPPING = new HashMap<>();
    static {
        TREASURE_MAPPING.put(4, 0);
        TREASURE_MAPPING.put(5, 1);
        TREASURE_MAPPING.put(6, 2);
    }

    public static final Map<Integer, Integer> LANDMARK_MAPPING = new HashMap<>();
    static {
        LANDMARK_MAPPING.put(0, 0);
        LANDMARK_MAPPING.put(1, 1);
        LANDMARK_MAPPING.put(2, 2);
        LANDMARK_MAPPING.put(3, 3);
        LANDMARK_MAPPING.put(7, 4);
        LANDMARK_MAPPING.put(8, 5);
        LANDMARK_MAPPING.put(9, 6);
        LANDMARK_MAPPING.put(10, 7);
    }
}