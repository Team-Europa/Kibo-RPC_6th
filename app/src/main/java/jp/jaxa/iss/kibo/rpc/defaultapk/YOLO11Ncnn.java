package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.content.res.AssetManager;
import android.graphics.Bitmap;

import jp.jaxa.iss.kibo.rpc.defaultapk.model.DetectionResult;

public class YOLO11Ncnn
{
    static {
        System.loadLibrary("yolo11ncnn");
    }

    public native boolean loadModel(AssetManager mgr, int taskid, int modelid, int cpugpu);
    public native DetectionResult[] detectObjects(Bitmap bitmap);
}
