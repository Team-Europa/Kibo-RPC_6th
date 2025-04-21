package jp.jaxa.iss.kibo.rpc.defaultapk.utils;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Scalar;


public class CamParameter {
    public Mat camMatrix;
    public Mat camDistCoeffs;
    private double[][] camIntrinsicsMatrix;
    public double tx;
    public double ty;
    public double tz;
    public Mat zeroDistCoeffs;
    public MatOfDouble zeroDoubleDistCoeffs;
    public Mat arUcoCalibCamMatrix;

    public void initCamParameter(double[][] camIntrinsicsMatrix, Double[] distFromCenter) {
        this.camIntrinsicsMatrix = camIntrinsicsMatrix;
        tx = distFromCenter[0];
        ty = distFromCenter[1];
        tz = distFromCenter[2];

        camMatrix = new Mat(3, 3 , CvType.CV_64F);
        camDistCoeffs = new Mat(1 , 5 , CvType.CV_64F);
        Mat zeroDistCoeffs = new Mat(1 , 5 , CvType.CV_64F);//setup distCoeffs for calibratedImg
        zeroDistCoeffs.setTo(new Scalar(0.0));
        zeroDoubleDistCoeffs = new MatOfDouble(0.0, 0.0, 0.0, 0.0, 0.0);

        setCamCalib(camIntrinsicsMatrix[0], camIntrinsicsMatrix[1], camMatrix, camDistCoeffs);
        setArUcoCalibCamMatrix();
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

    private void setArUcoCalibCamMatrix() {
        Mat cameraMatrix = new Mat(3, 3 , CvType.CV_64F);//setup cameraMatrix for calibratedImg
        cameraMatrix.put(0,0, camIntrinsicsMatrix[0][0]);
        cameraMatrix.put(0,1, 0);
        cameraMatrix.put(0,2, camIntrinsicsMatrix[0][2]);
        cameraMatrix.put(1,0, 0);
        cameraMatrix.put(1,1, camIntrinsicsMatrix[0][4]);
        cameraMatrix.put(1,2, camIntrinsicsMatrix[0][5]);
        cameraMatrix.put(2,0, 0);
        cameraMatrix.put(2,1, 0);
        cameraMatrix.put(2,2, 1);
    }
}
