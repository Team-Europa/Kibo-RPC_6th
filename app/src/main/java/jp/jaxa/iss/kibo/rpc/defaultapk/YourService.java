package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;
import android.util.Pair;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.PriorityBlockingQueue;
import java.util.concurrent.TimeUnit;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.defaultapk.utils.CamParameter;
import jp.jaxa.iss.kibo.rpc.defaultapk.utils.Estimate;
import jp.jaxa.iss.kibo.rpc.defaultapk.utils.ImageProcessUtils;
import jp.jaxa.iss.kibo.rpc.defaultapk.model.PointWithQuaternion;
import jp.jaxa.iss.kibo.rpc.defaultapk.utils.ItemDetectorUtils;
import jp.jaxa.iss.kibo.rpc.defaultapk.utils.QuaternionUtils;

import static jp.jaxa.iss.kibo.rpc.defaultapk.Constants.*;
import static jp.jaxa.iss.kibo.rpc.defaultapk.utils.QuaternionUtils.quaternionConjugate;

public class YourService extends KiboRpcService {
    private CamParameter navCamParameter = new CamParameter();
    private CamParameter dockCamParameter = new CamParameter();
    private ItemDetectorUtils itemDetectorUtils;
    private QuaternionUtils quaternionUtils;
    private Estimate estimate;
    private Vision vision;
    private HashMap<Integer, Point> pointAIMMap = new HashMap<>();

    private final PriorityBlockingQueue<ScanTask> scanTaskQueue =
            new PriorityBlockingQueue<ScanTask>(11, new Comparator<ScanTask>() {
                @Override
                public int compare(ScanTask o1, ScanTask o2) {
                    return Integer.compare(o1.getPriority(), o2.getPriority());
                }
            });

    private int saveImgNum = 0;

    @Override
    protected void runPlan1(){
        itemDetectorUtils = new ItemDetectorUtils(getApplicationContext());
        vision = new Vision();
        Thread visionThread = new Thread(vision);

        api.startMission();
        estimate = new Estimate(getAssets());
        api.flashlightControlBack(0.05f);
        api.flashlightControlFront(0.05f);
        initCamParameter();

        visionThread.start();
        scanTask();

        vision.stopRunning();
        try{
            visionThread.join();
        } catch(InterruptedException e){
            Thread.currentThread().interrupt();
        }

        moveToWithRetry(astronautPQ,1);
        reportAreaInfoAndEndRounding();
        api.notifyRecognitionItem();
        String targetItem = recognizeTargetItem();
        endGameTask(itemDetectorUtils.getTargetArea(targetItem));
        api.shutdownFactory();
    }

    private void scanTask(){
        moveToWithRetry(area1, 10);
        SystemClock.sleep(scanSleepMillis);
        estimateAruco(Cam.DOCK, area1,1);

//        moveToWithRetry(area2, 10);
//        SystemClock.sleep(scanSleepMillis);
//        estimateAruco(Cam.NAV, area2,2);
//
//        moveToWithRetry(area3, 10);
//        SystemClock.sleep(scanSleepMillis);
//        estimateAruco(Cam.NAV, area3,3);

        moveToWithRetry(area23, 10);
        SystemClock.sleep(2 * scanSleepMillis);
        estimateAruco(Cam.NAV, area23, 2);
        estimateAruco(Cam.NAV, area23, 3);

        moveToWithRetry(area4, 10);
        SystemClock.sleep(scanSleepMillis);
        estimateAruco(Cam.DOCK, area4,4);
        SystemClock.sleep(1500);
    }

    private class ScanTask {
        private final Mat image;
        private final int areaId;
        private final int priority;

        ScanTask(Mat image, int areaId) {
            this.image = image;
            this.areaId = areaId;
            this.priority = itemDetectorUtils.getScanCountForArea(areaId);
        }

        int getPriority() {
            return priority;
        }

        void process() {
            try {
                itemDetectorUtils.scanItemBoard(image, areaId);
            } finally {
                image.release();
            }
        }
    }

    class Vision implements Runnable {
        private ExecutorService executorService = Executors.newSingleThreadExecutor();
        private volatile boolean running = true;

        public void stopRunning(){
            running = false;
        }

        public boolean isRunning() {
            return running;
        }

        @Override
        public void run() {
            Mat navImgPast = api.getMatNavCam();
            Mat dockImgPast = api.getMatDockCam();

            while (running && !Thread.currentThread().isInterrupted()) {
                final Mat navImgNow = api.getMatNavCam();
                final Mat dockImgNow = api.getMatDockCam();

                long pastTime = System.currentTimeMillis();

                if (ImageProcessUtils.areImgDiff(navImgPast, navImgNow)) {
                    navImgPast = navImgNow.clone();

                    final Mat calibNavImg = ImageProcessUtils.calibCamImg(navImgNow, navCamParameter);

                    executorService.submit(new Runnable() {
                        @Override
                        public void run() {
                            scanItemFromMat(calibNavImg, navCamParameter);
                        }
                    });
                }

                if (ImageProcessUtils.areImgDiff(dockImgPast, dockImgNow)) {
                    dockImgPast = dockImgNow.clone();

                    final Mat calibDockImg = ImageProcessUtils.calibCamImg(dockImgNow, dockCamParameter);

                    executorService.submit(new Runnable() {
                        @Override
                        public void run() {
                            scanItemFromMat(calibDockImg, dockCamParameter);
                        }
                    });
                }

                while (!scanTaskQueue.isEmpty()) {
                    ScanTask task = scanTaskQueue.poll();
                    if (task != null) {
                        task.process();
                    }
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
            try {
                if (!executorService.awaitTermination(visionThread_stoppingLatency, TimeUnit.SECONDS)){
                    executorService.shutdownNow();
                }
            } catch (InterruptedException e){
                executorService.shutdownNow();
            }
        }
    }

    private void initCamParameter() {
        navCamParameter.initCamParameter(api.getNavCamIntrinsics(), navCamDistFromCenter);
        dockCamParameter.initCamParameter(api.getDockCamIntrinsics(), dockCamDistFromCenter);
    }

    private void scanItemFromMat(Mat calibImg, CamParameter camParameter){
        List<Mat> arucoCorners = new ArrayList<>();
        Mat arucoIDs = new Mat();

        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        DetectorParameters parameters = DetectorParameters.create();

        parameters.set_adaptiveThreshWinSizeMin(3);
        parameters.set_adaptiveThreshWinSizeMax(23);
        parameters.set_adaptiveThreshConstant(5);
        parameters.set_minMarkerPerimeterRate(0.02f);
        parameters.set_maxMarkerPerimeterRate(4.0f);
        parameters.set_minDistanceToBorder(1);
        parameters.set_cornerRefinementMethod(Aruco.CORNER_REFINE_SUBPIX);
        parameters.set_cornerRefinementWinSize(5);
        parameters.set_cornerRefinementMaxIterations(100);
        parameters.set_cornerRefinementMinAccuracy(0.005f);

        Aruco.detectMarkers(calibImg, dictionary, arucoCorners, arucoIDs, parameters);

        if(!arucoIDs.empty()) {
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(arucoCorners, 0.05f, camParameter.arUcoCalibCamMatrix, camParameter.zeroDistCoeffs, rvecs, tvecs);

            for (int i = 0; i < arucoIDs.rows(); i++) {
                int id = (int) arucoIDs.get(i, 0)[0]-100;
                if (id < 1 || id > 4) { continue; }
                Mat rvec = rvecs.row(i);
                Mat tvec = tvecs.row(i);

                Mat lostItemBoardImg = ImageProcessUtils.getWarpItemImg(calibImg, rvec, tvec, camParameter.arUcoCalibCamMatrix, camParameter.zeroDoubleDistCoeffs);

                if(lostItemBoardImg != null){
                    api.saveMatImage(lostItemBoardImg,"image_" + saveImgNum + ".png");
                    saveImgNum++;

                    if (vision.isRunning()) {
                        scanTaskQueue.put(new ScanTask(lostItemBoardImg.clone(), id));
                    } else {
                        lostItemBoardImg.release();
                    }
                } else {
                    Log.i("lostItemBoardImg","lostItemBoardImg is null");
                }
            }
        }
    }

    private void estimateAruco(Cam cam, PointWithQuaternion robotPQ, Integer areaNum){
        Mat mat;
        CamParameter camParameter;

        if(cam == Cam.DOCK){
            mat = api.getMatDockCam();
            camParameter = dockCamParameter;
            robotPQ.quaternion = quaternionConjugate(robotPQ.quaternion);
        }else {
            mat = api.getMatNavCam();
            camParameter = navCamParameter;
        }

        List<Mat> arucoCorners = new ArrayList<>();
        Mat arucoIDs = new Mat();

        mat = ImageProcessUtils.calibCamImg(mat, camParameter);

        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        DetectorParameters parameters = DetectorParameters.create();

        parameters.set_adaptiveThreshWinSizeMin(3);
        parameters.set_adaptiveThreshWinSizeMax(23);
        parameters.set_adaptiveThreshConstant(5);
        parameters.set_minMarkerPerimeterRate(0.02f);
        parameters.set_maxMarkerPerimeterRate(4.0f);
        parameters.set_minDistanceToBorder(1);
        parameters.set_cornerRefinementMethod(Aruco.CORNER_REFINE_SUBPIX);
        parameters.set_cornerRefinementWinSize(5);
        parameters.set_cornerRefinementMaxIterations(100);
        parameters.set_cornerRefinementMinAccuracy(0.005f);

        Aruco.detectMarkers(mat, dictionary, arucoCorners, arucoIDs, parameters);

        if(!arucoIDs.empty()) {
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(arucoCorners, 0.05f, camParameter.arUcoCalibCamMatrix, camParameter.zeroDoubleDistCoeffs, rvecs, tvecs);

            for (int i = 0; i < arucoIDs.rows(); i++) {
                int id = (int) arucoIDs.get(i, 0)[0]-100;

//                if (id != areaNum) continue;

                Mat tvec = tvecs.row(i);
                Mat rvec = rvecs.row(i);

                MatOfPoint3f itemBoardWorldPoint = new MatOfPoint3f(new Point3(-0.1325, -0.0375, 0));
                MatOfPoint2f itemBoardImagePoints = new MatOfPoint2f();

                Calib3d.projectPoints(itemBoardWorldPoint, rvec, tvec, camParameter.camMatrix, camParameter.zeroDoubleDistCoeffs, itemBoardImagePoints);

                org.opencv.core.Point imagePoint = itemBoardImagePoints.toList().get(0);
                double x = imagePoint.x;
                double y = imagePoint.y;

                org.opencv.core.Point projectedPointRatio = estimate.computeProjectedPoint(mat, new org.opencv.core.Point(x,y), id);
                Log.i("EST","Estimated area " + id + " with ratio: (" + projectedPointRatio.x + ", " + projectedPointRatio.y + ").");
                org.opencv.core.Point targetOrigin, targetMax;

                switch (id){
                    case 1:
                        targetOrigin = area1Origin;
                        targetMax = area1Max;
                        break;
                    case 2:
                        targetOrigin = area2Origin;
                        targetMax = area2Max;
                        break;
                    case 3:
                        targetOrigin = area3Origin;
                        targetMax = area3Max;
                        break;
                    default:
                        targetOrigin = area4Origin;
                        targetMax = area4Max;
                }

                double projectedPointX = targetOrigin.x + ( (targetMax.x - targetOrigin.x ) * projectedPointRatio.x);
                double projectedPointY = targetOrigin.y + ( (targetMax.y - targetOrigin.y ) * projectedPointRatio.y);

                Point projectedPoint;

                switch (id){
                    case 1:
                        projectedPoint = new Point(projectedPointY, targetY_area1, projectedPointX);
                        break;
                    case 2: case 3:
                        projectedPoint = new Point(projectedPointX + 0.10, projectedPointY, targetZ_area23);
                        break;
                    default:
                        projectedPoint = new Point(targetX_area4, projectedPointY, projectedPointX); // offset ok
                        break;
                }

                pointAIMMap.put(id, projectedPoint);

                Mat lostItemBoardImg = ImageProcessUtils.getWarpItemImg(mat, rvec, tvec, camParameter.arUcoCalibCamMatrix, camParameter.zeroDoubleDistCoeffs);
                if (vision.isRunning()) {
                    scanTaskQueue.put(new ScanTask(lostItemBoardImg.clone(), id));
                } else {
                    lostItemBoardImg.release();
                }

                api.saveMatImage(mat,id+"AIM.png");
            }
        }
    }

    private void reportAreaInfoAndEndRounding() {
        for (int areaNum = 1; areaNum <= 4; areaNum++) {
            Pair<String, Integer> areaInfo = itemDetectorUtils.getMaxFreqLandmarkItemData(areaNum);
            if (areaInfo == null) {
                Log.i("Report", "areaInfo is null for areaNum: " + areaNum);
                continue;
            }

            if (areaInfo.first == null || areaInfo.second == null) {
                Log.i("Report", "areaInfo has null elements for areaNum: " + areaNum);
                continue;
            }

            api.setAreaInfo(areaNum, areaInfo.first, areaInfo.second);
        }
        api.reportRoundingCompletion();
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

    private String recognizeTargetItem(){
        while(true){
            Mat img = ImageProcessUtils.calibCamImg(api.getMatNavCam(), navCamParameter);
            List<Mat> arucoCorners = new ArrayList<>();
            Mat arucoIDs = new Mat();

            Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            DetectorParameters parameters = DetectorParameters.create();

            parameters.set_adaptiveThreshWinSizeMin(3);
            parameters.set_adaptiveThreshWinSizeMax(23);
            parameters.set_adaptiveThreshConstant(5);
            parameters.set_minMarkerPerimeterRate(0.02f);
            parameters.set_maxMarkerPerimeterRate(4.0f);
            parameters.set_minDistanceToBorder(1);
            parameters.set_cornerRefinementMethod(Aruco.CORNER_REFINE_SUBPIX);
            parameters.set_cornerRefinementWinSize(5);
            parameters.set_cornerRefinementMaxIterations(100);
            parameters.set_cornerRefinementMinAccuracy(0.005f);

            Aruco.detectMarkers(img, dictionary, arucoCorners, arucoIDs, parameters);

            if(!arucoIDs.empty()) {
                Mat rvecs = new Mat();
                Mat tvecs = new Mat();
                Aruco.estimatePoseSingleMarkers(arucoCorners, 0.05f, navCamParameter.arUcoCalibCamMatrix, navCamParameter.zeroDistCoeffs, rvecs, tvecs);

                for (int i = 0; i < arucoIDs.rows(); i++) {
                    int id = (int) arucoIDs.get(i, 0)[0]-100;
                    if (id !=0) { continue; }
                    Mat rvec = rvecs.row(i);
                    Mat tvec = tvecs.row(i);

                    Mat recognizeItemBoardImg = ImageProcessUtils.getWarpItemImg(img, rvec, tvec, navCamParameter.arUcoCalibCamMatrix, navCamParameter.zeroDoubleDistCoeffs);
                    Bitmap recognizeItemBoardBitmap = ImageProcessUtils.getBitmapFromMat(recognizeItemBoardImg);
                    String targetItem = itemDetectorUtils.detectRecognizedResult(recognizeItemBoardBitmap);
                    if(targetItem != null){ return targetItem; }
                }
            }
            SystemClock.sleep(100);
        }
    }

    private void endGameTask(Integer areaNum){
//        if(areaNum==null){areaNum = -1;}
//        switch (areaNum){
//            case 1:
//                moveToWithRetry(targetPQ_area1, 1);
//                break;
//            case 2:
//                moveToWithRetry(targetPQ_area2, 1);
//                break;
//            case 3:
//                moveToWithRetry(targetPQ_area3, 1);
//                break;
//            default:
//                moveToWithRetry(targetPQ_area4, 1);
//                break;
//        }
//

//        SystemClock.sleep(3500);
        Point targetP = pointAIMMap.get(areaNum);
        Quaternion targetQ;

        if(targetP != null){
            switch(areaNum){
                case 1:
                    targetQ = new Quaternion(0f, 0f, -0.707f, 0.707f);
                    break;
                case 2: case 3:
                    targetQ = new Quaternion(0.5f, -0.5f, -0.5f, -0.5f);
                    break;
                default:
                    targetQ = new Quaternion(0f,0f,-1f,0f);
            }
            Log.i("EST", "Estimated point: " + targetP + "; Quaternion:" + targetQ);
            moveToWithRetry(new PointWithQuaternion(targetP, targetQ), 1);
//            if(areaNum == 2||areaNum == 3){
//                moveToWithRetry(new PointWithQuaternion(target, new Quaternion(0.5f, 0.5f, -0.5f, 0.5f)),1);
//            }else if(areaNum == 1){
//                moveToWithRetry(new PointWithQuaternion(target, new Quaternion(0f, 0f, -0.707f, 0.707f)),5);
//            }else {
//                moveToWithRetry(new PointWithQuaternion(new Point(targetX_area4, robotPos.getY() - target.getY(), robotPos.getZ() - target.getX()), new Quaternion(0f,0f,-1f,0f)),1);
//            }
        }else{
            Log.e("EST", "Unable to reach Area" + areaNum + ", because the estimated point is null.");
        }
        api.takeTargetItemSnapshot();
    }
}