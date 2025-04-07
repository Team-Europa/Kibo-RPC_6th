package jp.jaxa.iss.kibo.rpc.defaultapk.utils;

import org.opencv.core.CvType;
import org.opencv.core.Mat;


public class CamParameter {
    public Mat camMatrix;
    public Mat camDistCoeffs;
    public double[][] camIntrinsicsMatrix;
    public double tx;
    public double ty;
    public double tz;

    public void initCamParameter(double[][] camIntrinsicsMatrix, Double[] distFromCenter) {
        this.camIntrinsicsMatrix = camIntrinsicsMatrix;
        tx = distFromCenter[0];
        ty = distFromCenter[1];
        tz = distFromCenter[2];

        camMatrix = new Mat(3, 3 , CvType.CV_64F);
        camDistCoeffs = new Mat(1 , 5 , CvType.CV_64F);
        setCamCalib(camIntrinsicsMatrix[0], camIntrinsicsMatrix[1], camMatrix, camDistCoeffs);
    }

    private void setCamCalib(double[] cameraDoubleMatrix, double[] distortionCoefficientsDoubleMatrix, Mat cameraMatrix, Mat distortionCoefficients) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cameraMatrix.put(i, j, cameraDoubleMatrix[i * 3 + j]);
            }
        }

        for (int i = 0; i < 1; i++) {
            for (int j = 0; j < 5; j++) {
                distortionCoefficients.put(i, j, distortionCoefficientsDoubleMatrix[j]);
            }
        }
    }
}
