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

    static final PointWithQuaternion point1_1 = new PointWithQuaternion(new Point(10.425, -9.475,4.445), new Quaternion(0f, 0f, 0.707f, 0.707f));
    static final PointWithQuaternion point1_2 = new PointWithQuaternion(new Point(11.425, -9.475,5.4), new Quaternion(0f, 0f, 0.707f, 0.707f));
    static final PointWithQuaternion point2 = new PointWithQuaternion(new Point(11.425, -9.5, 5.4), new Quaternion(0f, 0f, 0.707f, 0.707f));
    static final PointWithQuaternion point2_2 = new PointWithQuaternion(new Point(10.925 , -8.45, 5.4), new Quaternion(0f, 0f, 0.707f, 0.707f));
    static final PointWithQuaternion point3_1 = new PointWithQuaternion(new Point(10.425, -7.5, 5.4), new Quaternion(-0.5f, 0.5f, 0.5f, 0.5f));
    static final PointWithQuaternion point3 = new PointWithQuaternion(new Point(10.425, -7.5, 5.4), new Quaternion(-0.5f, 0.5f, 0.5f, 0.5f));
    static final PointWithQuaternion point4_1 = new PointWithQuaternion(new Point(11.375, -7.3, 4.425), new Quaternion(0f,0f,-1f,0f));
    static final PointWithQuaternion point4_2 = new PointWithQuaternion(new Point(11.375, -6.35, 4.945), new Quaternion(0f,0f,-1f,0f));

    static final Double[] navCamDistFromCenter = {-0.0422, -0.0826, 0.1177};
    static final Double[] dockCamDistFromCenter = {-0.054, -0.0064, 0.1061};

    private static final double distanceToArea = 0.8725;
    static final double targetZ_area23 = 3.76093 + distanceToArea;
    static final double targetY_area1 = -10.58 + distanceToArea;
    static final double targetX_area4 = 9.866984 + distanceToArea;
    static final PointWithQuaternion targetPQ_area4 = new PointWithQuaternion(new Point(targetX_area4, -6.8525, 4.945), new Quaternion(0f,0f,-1f,0f));
    static final PointWithQuaternion targetPQ_area3 = new PointWithQuaternion(new Point(10.925, -7.925, targetZ_area23), new Quaternion(0.5f, 0.5f, -0.5f, 0.5f));
    static final PointWithQuaternion targetPQ_area2 = new PointWithQuaternion(new Point(10.925, -8.875, targetZ_area23), new Quaternion(0.5f, 0.5f, -0.5f, 0.5f));
    static final PointWithQuaternion targetPQ_area1 = new PointWithQuaternion(new Point(10.95, targetY_area1, 5.195), new Quaternion(0f, 0f, -0.707f, 0.707f));

    public static final Map<Integer, Integer> TREASURE_MAPPING = new HashMap<>();
    static {
        TREASURE_MAPPING.put(5, 0);
        TREASURE_MAPPING.put(6, 1);
        TREASURE_MAPPING.put(7, 2);
    }

    public static final Map<Integer, Integer> LANDMARK_MAPPING = new HashMap<>();
    static {
        LANDMARK_MAPPING.put(1, 0);
        LANDMARK_MAPPING.put(2, 1);
        LANDMARK_MAPPING.put(3, 2);
        LANDMARK_MAPPING.put(4, 3);
        LANDMARK_MAPPING.put(8, 4);
        LANDMARK_MAPPING.put(9, 5);
        LANDMARK_MAPPING.put(10, 6);
        LANDMARK_MAPPING.put(11, 7);
    }
}