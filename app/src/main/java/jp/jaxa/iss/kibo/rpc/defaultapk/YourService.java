package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.os.SystemClock;
import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.defaultapk.utils.PointWithQuaternion;


import static jp.jaxa.iss.kibo.rpc.defaultapk.Constants.*;

public class YourService extends KiboRpcService {

    private double[][]  navCamIntrinsicsMatrix;
    private double[][] dockCamIntrinsicsMatrix;

    private int saveImgNum= 0;

    @Override
    protected void runPlan1(){
        api.startMission();
        initCalibMatrix();
        Thread visionThread = new Thread(new Vision());
        visionThread.start();
        moveToWithRetry(point1,10);
        moveToWithRetry(point2,10);
        SystemClock.sleep(10000);
        moveToWithRetry(point3,10);
        moveToWithRetry(astronautPQ,10);
        visionThread.interrupt();
        SystemClock.sleep(3000);
        api.notifyRecognitionItem();
        api.takeTargetItemSnapshot();
    }

    @Override
    protected void runPlan2(){
        // write your plan 2 here
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here
    }

    class Vision implements Runnable {

        private ExecutorService executorService = Executors.newSingleThreadExecutor();

        @Override
        public void run() {
            Mat navImgPast = api.getMatNavCam();
            Mat dockImgPast = api.getMatDockCam();

            while (!Thread.currentThread().isInterrupted()) {
                Mat navImgNow = api.getMatNavCam();
                Mat dockImgNow = api.getMatDockCam();

                final Kinematics robotNowKinematics = api.getRobotKinematics();
                Long pastTime = System.currentTimeMillis();

                if (!areImgEqual(navImgPast, navImgNow)) {
                    PointWithQuaternion navImgShotPQ = new PointWithQuaternion(robotNowKinematics.getPosition(), robotNowKinematics.getOrientation());
                    navImgPast = navImgNow;

                    final Mat calibNavImg = calibImgWithMatrix(navImgNow, navCamIntrinsicsMatrix);

                    executorService.submit(new Runnable() {
                        @Override
                        public void run() {
                            scanItemFromMat(calibNavImg, navCamIntrinsicsMatrix[0], robotNowKinematics.getPosition(), robotNowKinematics.getOrientation());
                        }
                    });
                }

                if (!areImgEqual(dockImgPast, dockImgNow)) {
                    PointWithQuaternion dockImgShotPQ = new PointWithQuaternion(robotNowKinematics.getPosition(), robotNowKinematics.getOrientation());
                    dockImgPast = dockImgNow;

                    final Mat calibDockImg = calibImgWithMatrix(dockImgNow, dockCamIntrinsicsMatrix);

                    executorService.submit(new Runnable() {
                        @Override
                        public void run() {
                            scanItemFromMat(calibDockImg, navCamIntrinsicsMatrix[0], robotNowKinematics.getPosition(), QuaternionConjugate(robotNowKinematics.getOrientation()));
                        }
                    });
                }

                try {
                    long processingTime = System.currentTimeMillis() - pastTime;
                    long sleepTime = Math.max(100, 300 - processingTime);
                    Thread.sleep(sleepTime);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }

            executorService.shutdown();
        }

    }

    private void initCalibMatrix(){
        navCamIntrinsicsMatrix = api.getNavCamIntrinsics();
        dockCamIntrinsicsMatrix = api.getDockCamIntrinsics();
    }

    private Mat calibImgWithMatrix(Mat originalImg, double[][] calibMatrix) {
        Mat CameraMatrix = new Mat(3, 3 , CvType.CV_64F);
        Mat DistortionCoefficients = new Mat(1 , 5 , CvType.CV_64F);
        setCamCalib(calibMatrix[0], calibMatrix[1], CameraMatrix, DistortionCoefficients);

        Mat calibrateImaged = new Mat();

        Calib3d.undistort(
                originalImg,
                calibrateImaged,
                CameraMatrix,
                DistortionCoefficients
        );
        return calibrateImaged;
    }

    private void setCamCalib(double[] cameraDoubleMatrix, double[] distortionCoefficientsDoubleMatrix, Mat cameraMatrix, Mat distortionCoefficients) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cameraMatrix.put(i, j, cameraDoubleMatrix[i * 3 + j]);
            }
        }

        for (int i = 0; i < 1; i++) {
            for (int j = 0; j < 5; j++) {
                distortionCoefficients.put(i, j, distortionCoefficientsDoubleMatrix[j]);
            }
        }
    }

    public static boolean areImgEqual(Mat image1, Mat image2) {
        long hash1 = imageHash(image1);
        long hash2 = imageHash(image2);

        return hash1 == hash2;
    }

    private static long imageHash(Mat image) {
        long hash = 0;
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

    private void scanItemFromMat(Mat img, double[] camDoubleMatrix,Point point, Quaternion camQuaternion){
        Mat cameraMatrix = new Mat(3, 3 , CvType.CV_64F);//setup cameraMatrix for calibratedImg
        cameraMatrix.put(0,0, camDoubleMatrix[0]);
        cameraMatrix.put(0,1, 0);
        cameraMatrix.put(0,2, camDoubleMatrix[2]);
        cameraMatrix.put(1,0, 0);
        cameraMatrix.put(1,1, camDoubleMatrix[4]);
        cameraMatrix.put(1,2, camDoubleMatrix[5]);
        cameraMatrix.put(2,0, 0);
        cameraMatrix.put(2,1, 0);
        cameraMatrix.put(2,2, 1);


        Mat distCoeffs = new Mat(1 , 5 , CvType.CV_64F);//setup distCoeffs for calibratedImg
        distCoeffs.setTo(new Scalar(0.0));
        MatOfDouble doubleDistCoeffs = new MatOfDouble(0.0, 0.0, 0.0, 0.0, 0.0);

        List<Mat> arucoCorners = new ArrayList<>();
        Mat arucoIDs = new Mat();

        Aruco.detectMarkers(img, Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250), arucoCorners, arucoIDs);

        if(!arucoIDs.empty()) {
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(arucoCorners, 0.05f, cameraMatrix, distCoeffs, rvecs, tvecs);

            for (int i = 0; i < arucoIDs.rows(); i++) {
                int id = (int) arucoIDs.get(i, 0)[0]-100;
                if (id < 0 || id > 4) { continue; }
                Mat rvec = rvecs.row(i);
                Mat tvec = tvecs.row(i);

                Mat lostItemBoardImg = getWarpItemImg(img, rvec, tvec, cameraMatrix, doubleDistCoeffs);

                double[] tvecArray = tvec.get(0, 0);
                double tx = tvecArray[0];
                double ty = tvecArray[1];
                double tz = tvecArray[2];
                Point arucoWorldPos = getAbsolutePointByPQ(tx-0.0422, ty+0.0826, tz+0.1177, point, camQuaternion);

                Log.i("arucoWorldPos",id + ":  "+ arucoWorldPos);
                Log.i("robotPos",point + "//"+ camQuaternion);
                Log.i("arucoRelativePos",id + ":  "+ tvecArray[0] + ","+ tvecArray[1] + ","+ tvecArray[2] + ",");

                if(lostItemBoardImg != null){
                    api.saveMatImage(lostItemBoardImg,"image_" + saveImgNum + ".png");
                    saveImgNum++;
                }
            }
        }
    }

    private Mat getWarpItemImg(Mat originImg, Mat rvec, Mat tvec, Mat cameraMatrix, MatOfDouble doubleDistCoeffs){

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

        int cmpp = 30;
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

    private boolean moveToWithRetry(PointWithQuaternion pq, int loopMAX_time) {
        Point point = pq.point;
        Quaternion quaternion = pq.quaternion;
        Result result;
        final double MAX_THRESHOLD_Angle = 8.75;
        result = api.moveTo(point, quaternion, false);
        Quaternion currentQuaternion = api.getRobotKinematics().getOrientation();
        int loopCounter = 0;
        while (calculateAngle(currentQuaternion, quaternion) <= MAX_THRESHOLD_Angle && loopCounter < loopMAX_time) {
            result = api.moveTo(point, quaternion, false);
            ++loopCounter;
        }
        return result.hasSucceeded();
    }

    private static double calculateAngle(Quaternion q1, Quaternion q2) {
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

    private static Point getAbsolutePointByPQ(Double tx, Double ty, Double tz, Point localOrigin, Quaternion orientation) {
        // Rotation matrix converting
        double[][] R = quaternionToRotationMatrix(orientation);
        // Rotating and transforming
        double[] localOriginMatrix = {localOrigin.getX(), localOrigin.getY(), localOrigin.getZ()};
        double[] localPositionMatrix = {tx, ty, tz};
        double[] globalPosition = new double[3];
        for (int i = 0; i < 3; i++) {
            globalPosition[i] = localOriginMatrix[i];
            for (int j = 0; j < 3; j++) {
                globalPosition[i] += R[i][j] * localPositionMatrix[j];
            }
        }
        return new Point(globalPosition[0], globalPosition[1], globalPosition[2]);
    }

    private Quaternion QuaternionConjugate(Quaternion quaternion) {
        return new Quaternion(quaternion.getW(), -quaternion.getX(), -quaternion.getY(), -quaternion.getZ());
    }
}

