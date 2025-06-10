package jp.jaxa.iss.kibo.rpc.defaultapk.utils;

import android.content.res.AssetManager;
import android.graphics.Bitmap;

import jp.jaxa.iss.kibo.rpc.defaultapk.YOLO11Ncnn;
import jp.jaxa.iss.kibo.rpc.defaultapk.model.DetectionResult;

public class YoloDetectorUtils {
    private YOLO11Ncnn yolo11Ncnn = new YOLO11Ncnn();
    private AssetManager assetManager;
    private static final String[] tresure_item = {"crystal", "diamond", "emerald"};

    public YoloDetectorUtils(AssetManager assetManager) { this.assetManager = assetManager; }

    public String detectRecognizedResult(Bitmap bitmap){
        yolo11Ncnn.loadModel(assetManager, 0, 3, 0);
         DetectionResult result = yolo11Ncnn.detectObjects(bitmap)[0];
        if(result !=null){ return  tresure_item[result.label]; }
        return null;
    }
}
