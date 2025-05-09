package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;

import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.core.Mat;
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
import jp.jaxa.iss.kibo.rpc.defaultapk.utils.CamParameter;
import jp.jaxa.iss.kibo.rpc.defaultapk.utils.ImageProcessUtils;
import jp.jaxa.iss.kibo.rpc.defaultapk.model.PointWithQuaternion;
import jp.jaxa.iss.kibo.rpc.defaultapk.utils.ItemDetectorUtils;
import jp.jaxa.iss.kibo.rpc.defaultapk.utils.QuaternionUtils;


import static jp.jaxa.iss.kibo.rpc.defaultapk.Constants.*;

public class YourService extends KiboRpcService {
    private CamParameter navCamParameter = new CamParameter();
    private CamParameter dockCamParameter = new CamParameter();
    private ItemDetectorUtils itemDetectorUtils;

    private int saveImgNum = 0;

    @Override
    protected void runPlan1(){
        itemDetectorUtils = new ItemDetectorUtils(getAssets());
        api.startMission();
        initCamParameter();
        Thread visionThread = new Thread(new Vision());
        visionThread.start();
        moveToWithRetry(point1_1,1);
        moveToWithRetry(point1_2,1);
        moveToWithRetry(point2,1);
        moveToWithRetry(point3,1);
        moveToWithRetry(point4_1,1);
        moveToWithRetry(point4_2,1);
        moveToWithRetry(astronautPQ,10);
        api.setAreaInfo(1,"compass",1);
        api.reportRoundingCompletion();
        api.notifyRecognitionItem();
        SystemClock.sleep(5000);
        visionThread.interrupt();
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
                long pastTime = System.currentTimeMillis();

                if (ImageProcessUtils.areImgDiff(navImgPast, navImgNow)) {
                    navImgPast = navImgNow;

                    final Mat calibNavImg = ImageProcessUtils.calibCamImg(navImgNow, navCamParameter);

                    executorService.submit(new Runnable() {
                        @Override
                        public void run() {
                            scanItemFromMat(calibNavImg, navCamParameter, robotNowKinematics.getPosition(), robotNowKinematics.getOrientation());
                        }
                    });
                }

                if (ImageProcessUtils.areImgDiff(dockImgPast, dockImgNow)) {
                    dockImgPast = dockImgNow;

                    final Mat calibDockImg = ImageProcessUtils.calibCamImg(dockImgNow, dockCamParameter);

                    executorService.submit(new Runnable() {
                        @Override
                        public void run() {
                            scanItemFromMat(calibDockImg, dockCamParameter, robotNowKinematics.getPosition(), QuaternionUtils.quaternionConjugate(robotNowKinematics.getOrientation()));
                        }
                    });
                }

                try {
                    long processingTime = System.currentTimeMillis() - pastTime;
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }

            executorService.shutdown();
        }

    }

    private void initCamParameter() {
        navCamParameter.initCamParameter(api.getNavCamIntrinsics(), navCamDistFromCenter);
        dockCamParameter.initCamParameter(api.getDockCamIntrinsics(), dockCamDistFromCenter);
    }

    private void scanItemFromMat(Mat img, CamParameter camParameter,Point point, Quaternion camQuaternion){
        List<Mat> arucoCorners = new ArrayList<>();
        Mat arucoIDs = new Mat();

        Aruco.detectMarkers(img, Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250), arucoCorners, arucoIDs);

        if(!arucoIDs.empty()) {
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(arucoCorners, 0.05f, camParameter.arUcoCalibCamMatrix, camParameter.zeroDistCoeffs, rvecs, tvecs);

            for (int i = 0; i < arucoIDs.rows(); i++) {
                int id = (int) arucoIDs.get(i, 0)[0]-100;
                if (id < 0 || id > 4) { continue; }
                Mat rvec = rvecs.row(i);
                Mat tvec = tvecs.row(i);

                Mat lostItemBoardImg = ImageProcessUtils.getWarpItemImg(img, rvec, tvec, camParameter.arUcoCalibCamMatrix, camParameter.zeroDoubleDistCoeffs);

                double[] tvecArray = tvec.get(0, 0);
                double tx = tvecArray[0];
                double ty = tvecArray[1];
                double tz = tvecArray[2];
                Point arucoWorldPos = QuaternionUtils.getAbsolutePointByPQ(tx + camParameter.tx, ty + camParameter.ty, tz + camParameter.tz, point, camQuaternion);

                Log.i("arucoWorldPos",id + ":  "+ arucoWorldPos);
                Log.i("robotPos",point + "//"+ camQuaternion);
                Log.i("arucoRelativePos",id + ":  "+ tvecArray[0] + ","+ tvecArray[1] + ","+ tvecArray[2] + ",");

                if(lostItemBoardImg != null){
                    api.saveMatImage(lostItemBoardImg,"image_" + saveImgNum + ".png");
                    saveImgNum++;

                    Imgproc.cvtColor(lostItemBoardImg, lostItemBoardImg, Imgproc.COLOR_GRAY2RGBA);
                    Bitmap lostItemBoardBitmap = Bitmap.createBitmap(lostItemBoardImg.width(), lostItemBoardImg.height(), Bitmap.Config.ARGB_8888);
                    Utils.matToBitmap(lostItemBoardImg, lostItemBoardBitmap);

                    itemDetectorUtils.detectTresureItem(lostItemBoardBitmap);
                    itemDetectorUtils.detectLandmarkItem(lostItemBoardBitmap);

                }else{Log.i("lostItemBoardImg","lostItemBoardImg is null");}
            }
        }
    }

    private boolean moveToWithRetry(PointWithQuaternion pq, int loopMAX_time) {
        Point point = pq.point;
        Quaternion quaternion = pq.quaternion;
        Result result;
        final double MAX_THRESHOLD_Angle = 8.75;
        result = api.moveTo(point, quaternion, false);
        Quaternion currentQuaternion = api.getRobotKinematics().getOrientation();
        int loopCounter = 0;
        while (QuaternionUtils.calculateAngle(currentQuaternion, quaternion) <= MAX_THRESHOLD_Angle && loopCounter < loopMAX_time) {
            result = api.moveTo(point, quaternion, false);
            ++loopCounter;
        }
        return result.hasSucceeded();
    }
}

