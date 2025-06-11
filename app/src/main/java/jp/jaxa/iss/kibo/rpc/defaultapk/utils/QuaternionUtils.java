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

        // Compute camera position in world coordinates
        Point cameraWorldPos = getAbsolutePointByPQ(
                camOffsetFromCenter[0],
                camOffsetFromCenter[1],
                camOffsetFromCenter[2],
                robotPosition,
                robotOrientation
        );

        // Extract the translation vector from OpenCV (camera frame → robot frame)
        double[] tvecArr = new double[3];
        tvec.get(0, 0, tvecArr);
        double[] localVec = new double[]{tvecArr[2], tvecArr[0], -tvecArr[1]}; // camera z → robot x, camera x → robot y, y flipped

        // Convert direction to world coordinates using rotation matrix
        double[][] R = quaternionToRotationMatrix(robotOrientation);
        double[] direction = new double[3];
        for (int i = 0; i < 3; i++) {
            direction[i] = 0;
            for (int j = 0; j < 3; j++) {
                direction[i] += R[i][j] * localVec[j];
            }
        }

        // Normalize direction vector
        double dirNorm = Math.sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);
        for (int i = 0; i < 3; i++) direction[i] /= dirNorm;

        double cx = cameraWorldPos.getX();
        double cy = cameraWorldPos.getY();
        double cz = cameraWorldPos.getZ();

        // Estimate initial guess using simple intersection with fixed axis plane
        double px, py, pz;
        double scale;
        switch (axis) {
            case X:
                scale = (fixedValue - cx) / direction[0];
                px = fixedValue;
                py = cy + scale * direction[1];
                pz = cz + scale * direction[2];
                break;
            case Y:
                scale = (fixedValue - cy) / direction[1];
                px = cx + scale * direction[0];
                py = fixedValue;
                pz = cz + scale * direction[2];
                break;
            case Z:
            default:
                scale = (fixedValue - cz) / direction[2];
                px = cx + scale * direction[0];
                py = cy + scale * direction[1];
                pz = fixedValue;
                break;
        }

        // Gradient descent parameters
        double lr = 0.01;     // learning rate
        int maxIter = 100;
        double prevLoss = Double.MAX_VALUE;

        for (int iter = 0; iter < maxIter; iter++) {
            // Current direction from camera to point
            double dx = px - cx;
            double dy = py - cy;
            double dz = pz - cz;
            double norm = Math.sqrt(dx*dx + dy*dy + dz*dz);
            dx /= norm;
            dy /= norm;
            dz /= norm;

            // Loss: 1 - dot product (we want vectors to align)
            double dot = dx * direction[0] + dy * direction[1] + dz * direction[2];
            double loss = 1 - dot;

            // Numerical gradient (finite difference)
            double delta = 1e-4;
            double gradX = 0, gradY = 0;

            if (axis == Axis.X) {
                gradY = (lossAt(fixedValue, py + delta, pz, cx, cy, cz, direction) - loss) / delta;
                double gradZ = (lossAt(fixedValue, py, pz + delta, cx, cy, cz, direction) - loss) / delta;
                py -= lr * gradY;
                pz -= lr * gradZ;
            } else if (axis == Axis.Y) {
                gradX = (lossAt(px + delta, fixedValue, pz, cx, cy, cz, direction) - loss) / delta;
                double gradZ = (lossAt(px, fixedValue, pz + delta, cx, cy, cz, direction) - loss) / delta;
                px -= lr * gradX;
                pz -= lr * gradZ;
            } else {
                gradX = (lossAt(px + delta, py, fixedValue, cx, cy, cz, direction) - loss) / delta;
                gradY = (lossAt(px, py + delta, fixedValue, cx, cy, cz, direction) - loss) / delta;
                px -= lr * gradX;
                py -= lr * gradY;
            }

            // Early stopping
            if (Math.abs(prevLoss - loss) < 1e-6) break;
            prevLoss = loss;
        }

        return new Point(px, py, pz);
    }

    // Loss function: 1 - cosine similarity between estimated and target direction
    private static double lossAt(double x, double y, double z, double cx, double cy, double cz, double[] targetDir) {
        double dx = x - cx;
        double dy = y - cy;
        double dz = z - cz;
        double norm = Math.sqrt(dx*dx + dy*dy + dz*dz);
        dx /= norm;
        dy /= norm;
        dz /= norm;
        double dot = dx * targetDir[0] + dy * targetDir[1] + dz * targetDir[2];
        return 1 - dot;
    }

}
