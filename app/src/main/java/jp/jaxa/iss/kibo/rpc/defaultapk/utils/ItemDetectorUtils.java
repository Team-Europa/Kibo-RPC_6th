package jp.jaxa.iss.kibo.rpc.defaultapk.utils;

import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.util.Log;

import jp.jaxa.iss.kibo.rpc.defaultapk.YOLO11Ncnn;
import jp.jaxa.iss.kibo.rpc.defaultapk.model.DetectionResult;
import jp.jaxa.iss.kibo.rpc.defaultapk.model.SegDetectionResult;

public class ItemDetectorUtils {
    private YOLO11Ncnn yolo11Ncnn = new YOLO11Ncnn();
    private AssetManager assetManager;
    private static final String[] tresure_item = {"crystal", "diamond", "emerald"};
    private static final String[] landmark_item =
            {"Treasure_box", "coin", "compass", "coral", "fossil", "key", "letter", "shell"};

    public ItemDetectorUtils(AssetManager assetManager) {
        this.assetManager = assetManager;
    }

    public void detectTresureItem(Bitmap bitmap){
        yolo11Ncnn.loadModel(assetManager, 0, 3, 0);
        DetectionResult[] results = yolo11Ncnn.detectObjects(bitmap);

        if (results != null && results.length > 0) {
            for (DetectionResult result : results) {
                Log.i("YOLO11", "Label: " + tresure_item[result.label] + ", Prob: " + result.prob +
                        ", Rect: (" + result.x + "," + result.y + "," + result.width + "," + result.height + ")");
            }
        } else {
            Log.w("YOLO11", "No objects detected");
        }
    }

    public void detectLandmarkItem(Bitmap bitmap){
        yolo11Ncnn.loadModel(assetManager, 1, 3, 0);
        SegDetectionResult[] results_seg = yolo11Ncnn.detectSegObjects(bitmap);

        if (results_seg != null && results_seg.length > 0) {
            Log.i("Detection", "Found " + results_seg.length + " objects");
            for (int j = 0; j < results_seg.length; j++) {
                SegDetectionResult r = results_seg[j];
                Log.i("Object-" + j, String.format("Label:%s Prob:%.2f Box:[%d,%d,%d,%d] MaskBytes:%d",
                        landmark_item[r.label],
                        r.prob,
                        r.x, r.y, r.width, r.height,
                        r.mask.length));
            }
        } else {
            Log.i("Detection", "No objects detected");
        }
    }

}
