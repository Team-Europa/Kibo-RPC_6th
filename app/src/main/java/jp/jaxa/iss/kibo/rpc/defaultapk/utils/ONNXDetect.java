package jp.jaxa.iss.kibo.rpc.defaultapk.utils;

import android.content.Context;
import android.content.res.AssetManager;
import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import ai.onnxruntime.OnnxTensor;
import ai.onnxruntime.OrtEnvironment;
import ai.onnxruntime.OrtException;
import ai.onnxruntime.OrtSession;
import jp.jaxa.iss.kibo.rpc.defaultapk.model.DetectionResult;

class ONNXDetect {
    private final Context context;
    private final OrtEnvironment env;
    private OrtSession deimSession;
    private final AssetManager assetManager;

    ONNXDetect(Context context) {
        this.context = context;
        this.assetManager = context.getAssets();
        env = OrtEnvironment.getEnvironment();
        try {
            loadModel();
        } catch (Exception e) {
            Log.e("ItemDetectorUtils", "Model loading failed", e);
        }
    }

    public DetectionResult[] detect(Mat mat, float confidenceThreshold) {
        List<DetectionResult> results = new ArrayList<>();
        if (deimSession == null) {
            Log.e("ONNXDetect", "Model not loaded");
            return new DetectionResult[0];
        }

        int origWidth = mat.width();
        int origHeight = mat.height();

        float[] inputTensorData = preprocessMatToFloatArray(mat, 640, 640);
        if (inputTensorData == null) {
            Log.i("ONNXDetect", "Preprocess failed");
            return new DetectionResult[0];
        }

        long[] imageShape = {1, 3, 640, 640};
        long[][] origSizes = {{origHeight, origWidth}};

        try (
                OnnxTensor imageTensor = OnnxTensor.createTensor(env, FloatBuffer.wrap(inputTensorData), imageShape);
                OnnxTensor origTensor = OnnxTensor.createTensor(env, origSizes)
        ) {
            Map<String, OnnxTensor> inputs = new HashMap<>();
            inputs.put("images", imageTensor);
            inputs.put("orig_target_sizes", origTensor);

            try (OrtSession.Result outputs = deimSession.run(inputs)) {
                Object labelsObj = outputs.get(0).getValue();
                Object boxesObj = outputs.get(1).getValue();
                Object scoresObj = outputs.get(2).getValue();

                if (labelsObj instanceof long[][]
                        && boxesObj instanceof float[][][]
                        && scoresObj instanceof float[][]) {

                    long[][] labels = (long[][]) labelsObj;
                    float[][][] boxes = (float[][][]) boxesObj;
                    float[][] scores = (float[][]) scoresObj;

                    for (int batch = 0; batch < scores.length; batch++) {
                        for (int i = 0; i < scores[batch].length; i++) {
                            float score = scores[batch][i];
                            if (score < confidenceThreshold) continue;

                            long label = labels[batch][i];
                            float[] box = boxes[batch][i];

                            int x = (int) box[0];
                            int y = (int) box[1];
                            int width = (int) (box[2] - box[0]);
                            int height = (int) (box[3] - box[1]);

                            results.add(new DetectionResult(
                                    (int) label,
                                    score,
                                    x, y, width, height
                            ));
                        }
                    }
                } else {
                    throw new IllegalStateException("Unexpected output types");
                }
            }
        } catch (OrtException e) {
            Log.e("ONNXDetect", "Inference failed", e);
        }
        return results.toArray(new DetectionResult[0]);
    }


    private float[] preprocessMatToFloatArray(Mat mat, int targetWidth, int targetHeight) {
        Mat resized = new Mat();
        Imgproc.resize(mat, resized, new Size(targetWidth, targetHeight));

        // 轉成float陣列，並做channel reorder和normalize（依你的模型需求）
        int channels = resized.channels();
        int imgSize = targetWidth * targetHeight;
        float[] floatData = new float[channels * imgSize];

        byte[] data = new byte[(int) (resized.total() * resized.channels())];
        resized.get(0, 0, data);

        // 假設BGR轉RGB，且normalize到0~1
        for (int i = 0; i < imgSize; i++) {
            int b = data[i * 3] & 0xFF;
            int g = data[i * 3 + 1] & 0xFF;
            int r = data[i * 3 + 2] & 0xFF;

            // RGB順序
            floatData[i] = r / 255.0f;
            floatData[i + imgSize] = g / 255.0f;
            floatData[i + imgSize * 2] = b / 255.0f;
        }
        return floatData;
    }


    private void loadModel() {
        try {
            String modelPath = copyAssetToFile("DEIM.onnx");

            if (modelPath != null) {
                deimSession = env.createSession(modelPath, new OrtSession.SessionOptions());
            } else {
                Log.i("DEIM_ONNX", "Failed to load onnx file.");
            }
        } catch (Exception e) {
            Log.i("DEIM", "loadModel failed", e);
        }
    }

    private String copyAssetToFile(String assetName) {
        File file = new File(context.getFilesDir(), assetName);
        if (!file.exists()) {
            try {
                InputStream is = assetManager.open(assetName);
                FileOutputStream fos = new FileOutputStream(file);
                byte[] buffer = new byte[4096];
                int read;
                while ((read = is.read(buffer)) != -1) {
                    fos.write(buffer, 0, read);
                }
                fos.close();
                is.close();
            } catch (Exception e) {
                Log.e("ItemDetectorUtils", "copyAssetToFile failed for " + assetName, e);
                return null;
            }
        }
        return file.getAbsolutePath();
    }
}
