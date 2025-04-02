package jp.jaxa.iss.kibo.rpc.defaultapk;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.defaultapk.utils.PointWithQuaternion;

public class Constants {
    static final PointWithQuaternion astronautPQ =
            new PointWithQuaternion(
                    new Point(11.143, -6.6707, 4.9654),
                    new Quaternion(0f, 0f, 0.707f, 0.707f));
    static final PointWithQuaternion point1 = new PointWithQuaternion(new Point(10.95, -10,5.195), new Quaternion(0f, 0f, 0.707f, 0.707f));
    static final PointWithQuaternion point2 = new PointWithQuaternion(new Point(10.925, -7.925, 4.45), new Quaternion(0.5f, 0.5f, -0.5f, 0.5f));
    static final PointWithQuaternion point3 =
            new PointWithQuaternion(
                    new Point(11.143, -6.6707, 4.9654),
                    new Quaternion(0.5f, 0.5f, -0.5f, 0.5f));
}
