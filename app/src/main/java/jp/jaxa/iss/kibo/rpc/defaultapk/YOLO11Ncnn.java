package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.content.res.AssetManager;
import android.graphics.Bitmap;

import org.opencv.core.Mat;

import jp.jaxa.iss.kibo.rpc.defaultapk.model.DetectionResult;
import jp.jaxa.iss.kibo.rpc.defaultapk.model.SegDetectionResult;

public class YOLO11Ncnn
{
    static {
        System.loadLibrary("yolo11ncnn");
    }

    public native boolean loadModel(AssetManager mgr, int taskid, int modelid, int cpugpu);
    public native DetectionResult[] detectObjects(Bitmap bitmap);
    public native SegDetectionResult[] detectSegObjects(Mat mat);
}
