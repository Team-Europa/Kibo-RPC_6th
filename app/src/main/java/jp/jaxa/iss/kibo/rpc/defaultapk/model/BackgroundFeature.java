package jp.jaxa.iss.kibo.rpc.defaultapk.model;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;

import java.util.List;

public class BackgroundFeature {
    public Mat img;
    public List<KeyPoint> keypoints;
    public Mat descriptors;

    public BackgroundFeature(Mat img, List<KeyPoint> keypoints, Mat descriptors) {
        this.img = img;
        this.keypoints = keypoints;
        this.descriptors = descriptors;
    }
}