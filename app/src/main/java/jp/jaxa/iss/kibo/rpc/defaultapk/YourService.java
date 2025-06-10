package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;
import android.util.Pair;

import org.opencv.aruco.Aruco;
import org.opencv.core.Mat;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.PriorityBlockingQueue;
import java.util.concurrent.TimeUnit;

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
    private Vision vision;

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
        api.flashlightControlBack(0.05f);
        api.flashlightControlFront(0.05f);
        initCamParameter();

        visionThread.start();

        moveToWithRetry(point1_1, 1);
        moveToWithRetry(point1_2, 1);
        moveToWithRetry(point2,1);
        moveToWithRetry(point2_2,1);
        moveToWithRetry(point3_1,1);
        moveToWithRetry(point3,1);
        moveToWithRetry(point4_1,1);
        moveToWithRetry(astronautPQ,1);

        vision.stopRunning();
        try{
            visionThread.join();
        } catch(InterruptedException e){
            Thread.currentThread().interrupt();
        }

        reportAreaInfoAndEndRounding();
        api.notifyRecognitionItem();
        String targetItem = recognizeTargetItem();
        endGameTask(itemDetectorUtils.getTargetArea(targetItem));
        api.shutdownFactory();
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

    private void scanItemFromMat(Mat img, CamParameter camParameter, Point point, Quaternion camQuaternion){
        List<Mat> arucoCorners = new ArrayList<>();
        Mat arucoIDs = new Mat();

        Aruco.detectMarkers(img, Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250), arucoCorners, arucoIDs);

        if(!arucoIDs.empty()) {
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(arucoCorners, 0.05f, camParameter.arUcoCalibCamMatrix, camParameter.zeroDistCoeffs, rvecs, tvecs);

            for (int i = 0; i < arucoIDs.rows(); i++) {
                int id = (int) arucoIDs.get(i, 0)[0]-100;
                if (id < 1 || id > 4) { continue; }
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

            Aruco.detectMarkers(img, Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250), arucoCorners, arucoIDs);

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
        if(areaNum==null){areaNum = -1;}

        switch (areaNum){
            case 1:
                moveToWithRetry(targetPQ_area1, 5);
                break;
            case 2:
                moveToWithRetry(targetPQ_area2, 5);
                break;
            case 3:
                moveToWithRetry(targetPQ_area3, 5);
                break;
            default:
                moveToWithRetry(targetPQ_area4, 5);
                break;
        }

        Point robotPos = api.getRobotKinematics().getPosition();
        SystemClock.sleep(3500);
        Point error = calcArucoPos(ImageProcessUtils.calibCamImg(api.getMatNavCam(), navCamParameter), areaNum);
        if(error != null){
            if(areaNum == 2||areaNum == 3){
                moveToWithRetry(new PointWithQuaternion(new Point(robotPos.getX() + error.getX(), robotPos.getY() - error.getY(), targetZ_area23), new Quaternion(0.5f, 0.5f, -0.5f, 0.5f)),5);
            }else if(areaNum == 1){
                moveToWithRetry(new PointWithQuaternion(new Point(robotPos.getX() + error.getX(), targetY_area1, robotPos.getZ() + error.getY()), new Quaternion(0f, 0f, -0.707f, 0.707f)),5);
            }else {
                moveToWithRetry(new PointWithQuaternion(new Point(targetX_area4, robotPos.getY() - error.getX(), robotPos.getZ() + error.getY()), new Quaternion(0f,0f,-1f,0f)),5);
            }
        }

        api.takeTargetItemSnapshot();
    }

    private Point calcArucoPos(Mat img, Integer targetAreaNum) {
        List<Mat> arucoCorners = new ArrayList<>();
        Mat arucoIDs = new Mat();

        Aruco.detectMarkers(img, Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250), arucoCorners, arucoIDs);

        if(!arucoIDs.empty()) {
            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(arucoCorners, 0.05f, navCamParameter.arUcoCalibCamMatrix, navCamParameter.zeroDoubleDistCoeffs, rvecs, tvecs);

            for (int i = 0; i < arucoIDs.rows(); i++) {
                int id = (int) arucoIDs.get(i, 0)[0]-100;
                if(id != targetAreaNum){continue;}
                Mat tvec = tvecs.row(i);

                double[] tvecArray = tvec.get(0, 0);
                double tx = tvecArray[0];
                double ty = tvecArray[1];
                double tz = tvecArray[2];

                return new Point(tx, ty, tz);
            }
        }
        return null;
    }
}