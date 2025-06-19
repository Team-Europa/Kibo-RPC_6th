package jp.jaxa.iss.kibo.rpc.defaultapk.utils;

import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.features2d.BFMatcher;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.ORB;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import jp.jaxa.iss.kibo.rpc.defaultapk.model.BackgroundFeature;

public class Estimate {
    AssetManager assetManager;

    // 用來存放每個區域的背景特徵 (只存一張)
    private HashMap<Integer, BackgroundFeature> backgroundFeaturesMap = new HashMap<>();

    public Estimate(AssetManager assetManager){
        this.assetManager = assetManager;
        initbackgroundFeaturesMap();
    }

    public Point computeProjectedPoint(Mat input, Point inputPoint, Integer areaNum) {
        ORB orb = ORB.create(1000);

        MatOfKeyPoint inputKeypoints = new MatOfKeyPoint();
        Mat inputDescriptors = new Mat();
        orb.detectAndCompute(input, new Mat(), inputKeypoints, inputDescriptors);

        if (inputDescriptors.empty()) {
            return null; // 無法計算特徵
        }

        BackgroundFeature bg = backgroundFeaturesMap.get(areaNum);
        if (bg == null) return null;

        if (bg.descriptors.empty() || bg.keypoints == null || bg.keypoints.isEmpty()) {
            return null;
        }

        BFMatcher matcher = BFMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING, true);
        MatOfDMatch matches = new MatOfDMatch();
        matcher.match(bg.descriptors, inputDescriptors, matches);

        List<DMatch> matchList = matches.toList();
        if (matchList.size() < 10) return null;

        KeyPoint[] inputKeypointArray = inputKeypoints.toArray();

        List<Point> ptsBg = new ArrayList<>();
        List<Point> ptsInput = new ArrayList<>();
        for (DMatch m : matchList) {
            if (m.queryIdx < bg.keypoints.size() && m.trainIdx < inputKeypointArray.length) {
                ptsBg.add(bg.keypoints.get(m.queryIdx).pt);
                ptsInput.add(inputKeypointArray[m.trainIdx].pt);
            }
        }

        if (ptsBg.size() < 4 || ptsInput.size() < 4) return null;

        MatOfPoint2f srcPoints = new MatOfPoint2f();
        srcPoints.fromList(ptsInput);
        MatOfPoint2f dstPoints = new MatOfPoint2f();
        dstPoints.fromList(ptsBg);

        Mat H = Calib3d.findHomography(srcPoints, dstPoints, Calib3d.RANSAC, 5.0);
        if (H.empty()) return null;

        // 投影點轉換
        MatOfPoint2f src = new MatOfPoint2f(inputPoint);
        MatOfPoint2f dst = new MatOfPoint2f();
        Core.perspectiveTransform(src, dst, H);
        Point projected = dst.toArray()[0];

        // 回傳相對於整張背景圖的比例座標
        double ratioX;
        double ratioY;

        switch(areaNum){
            case 1:
                ratioX = projected.x / (bg.img.width() + 250);
                ratioY = projected.y / bg.img.height();
                break;
            case 2:
                ratioX = projected.x / bg.img.width();
                ratioY = projected.y / (bg.img.height() - 280);
                break;
            case 3:
                ratioX = projected.x / bg.img.width();
                ratioY = (projected.y - 280) / (bg.img.height() - 280);
                break;
            default:
                ratioX = projected.x / bg.img.height();
                ratioY = projected.y / bg.img.height();
        }

        return new Point(ratioX, ratioY);
    }

    private Mat loadMatFromAssets(String fileName) {
        Mat mat = new Mat();
        try (InputStream inputStream = assetManager.open(fileName)) {
            Bitmap bitmap = BitmapFactory.decodeStream(inputStream);
            if (bitmap != null) {
                Bitmap bmp32 = bitmap.copy(Bitmap.Config.ARGB_8888, true);
                Utils.bitmapToMat(bmp32, mat);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        return mat;
    }

    private void initbackgroundFeaturesMap(){
        for (int i = 1; i <= 4; i++) {
            Mat bgImage = loadMatFromAssets("Area" + i + ".png"); // 確認檔名副檔名
            if (bgImage.empty()) continue;

            ORB orb = ORB.create(1000);
            Mat gray = new Mat();
            Imgproc.cvtColor(bgImage, gray, Imgproc.COLOR_BGR2GRAY);

            MatOfKeyPoint keypoints = new MatOfKeyPoint();
            Mat descriptors = new Mat();
            orb.detectAndCompute(gray, new Mat(), keypoints, descriptors);

            List<KeyPoint> keypointList = keypoints.toList();

            BackgroundFeature bg = new BackgroundFeature(bgImage, keypointList, descriptors);
            backgroundFeaturesMap.put(i, bg);
        }
    }
}

