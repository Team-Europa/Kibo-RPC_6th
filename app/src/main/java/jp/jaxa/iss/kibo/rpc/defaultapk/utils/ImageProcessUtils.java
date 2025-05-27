package jp.jaxa.iss.kibo.rpc.defaultapk.utils;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.imgproc.Imgproc;

public class ImageProcessUtils {
    public static Mat calibCamImg(Mat originalImg, CamParameter camParameter) {
        Mat calibrateImaged = new Mat();
        Calib3d.undistort(originalImg, calibrateImaged, camParameter.camMatrix, camParameter.camDistCoeffs);
        return calibrateImaged;
    }

    public static boolean areImgDiff(Mat image1, Mat image2) {
        long hash1 = imageHash(image1);
        long hash2 = imageHash(image2);

        return hash1 != hash2;
    }

    private static long imageHash(Mat image) {
        long hash;
        int width = image.cols();
        int height = image.rows();
        int sum = 0;

        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                sum += (int)image.get(i, j)[0];
            }
        }

        sum /= (width * height);
        hash = sum;

        return hash;
    }

    public static Mat getWarpItemImg(Mat originImg, Mat rvec, Mat tvec, Mat cameraMatrix, MatOfDouble doubleDistCoeffs){

        MatOfPoint3f itemBoardWorldPoint = new MatOfPoint3f(
                new Point3(-0.2325, 0.0375, 0),
                new Point3(-0.2325, -0.1125, 0),
                new Point3(-0.0325, -0.1125, 0),
                new Point3(-0.0325, 0.0375, 0));

        MatOfPoint2f itemBoardImagePoints = new MatOfPoint2f();

        Calib3d.projectPoints(itemBoardWorldPoint, rvec, tvec, cameraMatrix, doubleDistCoeffs, itemBoardImagePoints);

        org.opencv.core.Point[] points = itemBoardImagePoints.toArray();
        for (org.opencv.core.Point point : points) {
            if (point.x < 0 || point.x >= originImg.cols() || point.y < 0 || point.y >= originImg.rows()) {
                return null;
            }
        }

        int cmpp = 32;
        Mat frontView = new Mat(15 * cmpp, 20 * cmpp, CvType.CV_8UC3);

        MatOfPoint2f dstPoints = new MatOfPoint2f(
                new org.opencv.core.Point(0, 0),
                new org.opencv.core.Point(0, frontView.rows() - 1),
                new org.opencv.core.Point(frontView.cols() - 1, frontView.rows() - 1),
                new org.opencv.core.Point(frontView.cols() - 1, 0));


        Mat transformationMatrix = Imgproc.getPerspectiveTransform(itemBoardImagePoints, dstPoints);
        Imgproc.warpPerspective(originImg, frontView, transformationMatrix, frontView.size());
        return frontView;
    }
}
