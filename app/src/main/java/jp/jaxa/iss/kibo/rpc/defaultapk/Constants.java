package jp.jaxa.iss.kibo.rpc.defaultapk;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.defaultapk.model.PointWithQuaternion;

class Constants {
    static final PointWithQuaternion astronautPQ =
            new PointWithQuaternion(
                    new Point(11.143, -6.6707, 4.9654),
                    new Quaternion(0f, 0f, 0.707f, 0.707f));
    static final PointWithQuaternion point1_1 = new PointWithQuaternion(new Point(10.5, -9.45,4.5), new Quaternion(0f, 0f, 0.707f, 0.707f));
    static final PointWithQuaternion point1_2 = new PointWithQuaternion(new Point(11.4, -9.4,4.9), new Quaternion(0f, 0f, 0.707f, 0.707f));
    static final PointWithQuaternion point2 = new PointWithQuaternion(new Point(10.925, -9.5, 4.915), new Quaternion(0.5f, 0.5f, -0.5f, 0.5f));
    static final PointWithQuaternion point3 = new PointWithQuaternion(new Point(10.425, -7.4, 4.915), new Quaternion(0.5f, 0.5f, -0.5f, 0.5f));
    static final PointWithQuaternion point4_1 = new PointWithQuaternion(new Point(10.725, -7.35, 4.8), new Quaternion(0f,0f,-1f,0f));
    static final PointWithQuaternion point4_2 =
            new PointWithQuaternion(
                    new Point(11.143, -6.6707, 4.8),
                    new Quaternion(0f,0f,-1f,0f));

    static final Double[] navCamDistFromCenter = {-0.0422, -0.0826, 0.1177};
    static final Double[] dockCamDistFromCenter = {-0.054, -0.0064, 0.1061};

    private static final double distanceToArea = 0.775;
    static final double targetZ_area23 = 3.76093 + distanceToArea;
    static final double targetY_area1 = -10.58 + distanceToArea;
    static final double targetX_area4 = 9.866984 + distanceToArea;
    static final PointWithQuaternion targetPQ_area4 = new PointWithQuaternion(new Point(targetX_area4, -6.8525, 4.945), new Quaternion(0f,0f,-1f,0f));
    static final PointWithQuaternion targetPQ_area3 = new PointWithQuaternion(new Point(10.925, -7.925, targetZ_area23), new Quaternion(0.5f, 0.5f, -0.5f, 0.5f));
    static final PointWithQuaternion targetPQ_area2 = new PointWithQuaternion(new Point(10.925, -8.875, targetZ_area23), new Quaternion(0.5f, 0.5f, -0.5f, 0.5f));
    static final PointWithQuaternion targetPQ_area1 = new PointWithQuaternion(new Point(10.95, targetY_area1, 5.195), new Quaternion(0f, 0f, -0.707f, 0.707f));
}
