package jp.jaxa.iss.kibo.rpc.defaultapk.utils;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public class PointWithQuaternion {
    public Point point;
    public Quaternion quaternion;

    public PointWithQuaternion(Point point, Quaternion quaternion) {
        this.point = point;
        this.quaternion = quaternion;
    }
}
