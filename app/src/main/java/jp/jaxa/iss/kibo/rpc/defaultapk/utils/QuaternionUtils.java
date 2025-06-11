package jp.jaxa.iss.kibo.rpc.defaultapk.utils;

import android.util.Pair;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import org.opencv.core.Mat;

public class QuaternionUtils {

    public enum Axis { X, Y, Z }

    public static double calculateAngle(Quaternion q1, Quaternion q2) {
        double dotProduct = q1.getW() * q2.getW() + q1.getX() * q2.getX() + q1.getY() * q2.getY() + q1.getZ() * q2.getZ();
        double q1Magnitude = Math.sqrt(q1.getW() * q1.getW() + q1.getX() * q1.getX() + q1.getY() * q1.getY() + q1.getZ() * q1.getZ());
        double q2Magnitude = Math.sqrt(q2.getW() * q2.getW() + q2.getX() * q2.getX() + q2.getY() * q2.getY() + q2.getZ() * q2.getZ());
        return Math.acos(dotProduct / (q1Magnitude * q2Magnitude));
    }

    private static double[][] quaternionToRotationMatrix(Quaternion q) {
        double w = q.getW(), x = q.getX(), y = q.getY(), z = q.getZ();
        return new double[][] {
                {1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
                {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
                {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}
        };
    }

    public static Point getAbsolutePointByPQ(Double tx, Double ty, Double tz, Point localOrigin, Quaternion orientation) {
        double[][] R = quaternionToRotationMatrix(orientation);
        double[] localOriginMatrix = {localOrigin.getX(), localOrigin.getY(), localOrigin.getZ()};
        double[] localPositionMatrix = {tx, -ty, tz};
        double[] globalPosition = new double[3];
        for (int i = 0; i < 3; i++) {
            globalPosition[i] = localOriginMatrix[i];
            for (int j = 0; j < 3; j++) {
                globalPosition[i] += R[i][j] * localPositionMatrix[j];
            }
        }
        return new Point(globalPosition[0], globalPosition[1], globalPosition[2]);
    }

    public static Quaternion quaternionConjugate(Quaternion quaternion) {
        return new Quaternion(quaternion.getW(), -quaternion.getX(), -quaternion.getY(), -quaternion.getZ());
    }

    public static Point solveWorldPositionWithFixedAxis(
            Mat tvec,
            Point robotPosition,
            Quaternion robotOrientation,
            Pair<Axis, Double> axisfixConfig,
            Double[] camOffsetFromCenter
    ) {
        Axis axis = axisfixConfig.first;
        Double fixedValue = axisfixConfig.second;

        Point cameraWorldPos = getAbsolutePointByPQ(
                camOffsetFromCenter[0],
                camOffsetFromCenter[1],
                camOffsetFromCenter[2],
                robotPosition,
                robotOrientation
        );

        double[] tvecArr = new double[3];
        tvec.get(0, 0, tvecArr);
        double tx = tvecArr[2];       // camera z → robot x
        double ty = tvecArr[0];       // camera x → robot y
        double tz = -tvecArr[1];

        double[][] R = quaternionToRotationMatrix(robotOrientation);
        double[] direction = new double[3];
        double[] localVector = {tx, ty, tz};
        for (int i = 0; i < 3; i++) {
            direction[i] = 0;
            for (int j = 0; j < 3; j++) {
                direction[i] += R[i][j] * localVector[j];
            }
        }

        double cx = cameraWorldPos.getX();
        double cy = cameraWorldPos.getY();
        double cz = cameraWorldPos.getZ();

        double scale;
        switch (axis) {
            case X:
                scale = (fixedValue - cx) / direction[0];
                break;
            case Y:
                scale = (fixedValue - cy) / direction[1];
                break;
            case Z:
            default:
                scale = (fixedValue - cz) / direction[2];
                break;
        }

        double wx = cx + scale * direction[0];
        double wy = cy + scale * direction[1];
        double wz = cz + scale * direction[2];

        return new Point(wx, wy, wz);
    }

}
