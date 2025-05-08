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

    static final String[] tresure_item = {"crystal", "diamond", "emerald"};
    static final String[] landmark_item =
            {"Treasure_box", "coin", "compass", "coral", "fossil", "key", "letter", "shell"};
}
