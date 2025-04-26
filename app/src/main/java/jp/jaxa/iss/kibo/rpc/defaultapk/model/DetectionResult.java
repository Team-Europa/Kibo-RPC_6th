package jp.jaxa.iss.kibo.rpc.defaultapk.model;

public class DetectionResult {
    public int label;
    public float prob;
    public int x;
    public int y;
    public int width;
    public int height;

    public DetectionResult(int label, float prob, int x, int y, int width, int height) {
        this.label = label;
        this.prob = prob;
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
    }
}