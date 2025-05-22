package jp.jaxa.iss.kibo.rpc.defaultapk.utils;

import android.content.Context;
import android.content.res.AssetManager;
import android.util.Log;
import android.util.Pair;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.util.*;

import ai.onnxruntime.*;

import jp.jaxa.iss.kibo.rpc.defaultapk.model.DetectionResult;
import jp.jaxa.iss.kibo.rpc.defaultapk.model.SegDetectionResult;

public class ItemDetectorUtils {
    private final AssetManager assetManager;
    private final Context context;
    private final OrtEnvironment env;
    private OrtSession treasureSession;
    private OrtSession landMarkSession;

    private static final String[] treasure_items = {"crystal", "diamond", "emerald"};
    private static final String[] landmark_items = {
            "treasure_box", "coin", "compass", "coral",
            "fossil", "key", "treasure_boxtreasure_box", "shell"
    };

    private final Map<Integer, List<Pair<String, Integer>>> areaLandmarkDatas = new HashMap<>();
    private final Map<Integer, List<String>> areaTreasureDatas = new HashMap<>();

    public ItemDetectorUtils(Context context) {
        this.context = context;
        this.assetManager = context.getAssets();
        env = OrtEnvironment.getEnvironment();
        try {
            loadModel();
        } catch (Exception e) {
            Log.e("ItemDetectorUtils", "Model loading failed", e);
        }
    }

    private void loadModel() {
        try {
            String modelPath1 = copyAssetToFile("treasureDEIM.onnx");
            String modelPath2 = copyAssetToFile("landMarkDEIM.onnx");

            if (modelPath1 != null) {
                treasureSession = env.createSession(modelPath1, new OrtSession.SessionOptions());
            } else {
                Log.e("ItemDetectorUtils", "Failed to load treasure model file.");
            }
            if (modelPath2 != null) {
                landMarkSession = env.createSession(modelPath2, new OrtSession.SessionOptions());
            } else {
                Log.e("ItemDetectorUtils", "Failed to load landmark model file.");
            }
        } catch (Exception e) {
            Log.e("ItemDetectorUtils", "loadModel failed", e);
        }
    }

    public void detectTreasureItem(Mat image, Integer areaNum) {
        try {
            float[][][] input = preprocessMat(image, 640, 480);
            OnnxTensor inputTensor = OnnxTensor.createTensor(env, input);

            OrtSession.Result result = treasureSession.run(
                    Collections.singletonMap(
                            treasureSession.getInputNames().iterator().next(),
                            inputTensor
                    )
            );

            float[][] output = (float[][]) result.get(0).getValue(); // 假設輸出為 [1, class_count]
            int maxIndex = argmax(output[0]);
            String detected = treasure_items[maxIndex];

            List<String> treasureList = areaTreasureDatas.get(areaNum);
            if (treasureList == null) {
                treasureList = new ArrayList<>();
                areaTreasureDatas.put(areaNum, treasureList);
            }
            treasureList.add(detected);

            Log.i("ONNX", "Treasure Detected: " + detected + " at area " + areaNum);
        } catch (Exception e) {
            Log.e("ItemDetectorUtils", "detectTreasureItem failed", e);
        }
    }

    public void detectLandmarkItem(Mat image, Integer areaNum) {
        try {
            float[][][] input = preprocessMat(image, 224, 224); // 根據模型尺寸修改
            OnnxTensor inputTensor = OnnxTensor.createTensor(env, input);

            OrtSession.Result result = landMarkSession.run(
                    Collections.singletonMap(
                            landMarkSession.getInputNames().iterator().next(),
                            inputTensor
                    )
            );
            float[][] output = (float[][]) result.get(0).getValue(); // 假設輸出為 [1, class_count]

            int maxIndex = argmax(output[0]);
            float maxProb = output[0][maxIndex];

            String label = landmark_items[maxIndex];
            int count = (int) (maxProb * 100); // 假設比例表示可信度

            putVisionData(areaNum, new Pair<>(label, count));

            Log.i("ONNX", "Landmark Detected: " + label + " (prob = " + maxProb + ")");
        } catch (Exception e) {
            Log.e("ItemDetectorUtils", "detectLandmarkItem failed", e);
        }
    }

    public String detectRecognizedResult(Mat image) {
        try {
            float[][][] input = preprocessMat(image, 640, 480);
            OnnxTensor inputTensor = OnnxTensor.createTensor(env, input);

            OrtSession.Result result = treasureSession.run(
                    Collections.singletonMap(
                            treasureSession.getInputNames().iterator().next(),
                            inputTensor
                    )
            );
            float[][] output = (float[][]) result.get(0).getValue();
            int maxIndex = argmax(output[0]);

            return treasure_items[maxIndex];
        } catch (Exception e) {
            Log.e("ItemDetectorUtils", "detectRecognizedResult failed", e);
            return null;
        }
    }

    public Integer getTargetArea(String targetTreasure) {
        int maxCount = 0;
        Integer resultKey = null;

        for (Map.Entry<Integer, List<String>> entry : areaTreasureDatas.entrySet()) {
            int count = Collections.frequency(entry.getValue(), targetTreasure);
            if (count > maxCount) {
                maxCount = count;
                resultKey = entry.getKey();
            }
        }

        return resultKey;
    }

    public Pair<String, Integer> getMaxFreqLandmarkItemData(Integer areaNum) {
        List<Pair<String, Integer>> visionDataList = areaLandmarkDatas.get(areaNum);
        if (visionDataList == null || visionDataList.isEmpty()) return null;

        Map<String, Integer> freq = new HashMap<>();
        for (Pair<String, Integer> p : visionDataList) {
            String key = p.first;
            freq.put(key, freq.getOrDefault(key, 0) + 1);
        }

        String maxLabel = null;
        int maxValue = 0;
        for (Map.Entry<String, Integer> entry : freq.entrySet()) {
            if (entry.getValue() > maxValue) {
                maxValue = entry.getValue();
                maxLabel = entry.getKey();
            }
        }

        List<Pair<String, Integer>> filteredPairs = new ArrayList<>();
        for (Pair<String, Integer> p : visionDataList) {
            if (p.first.equals(maxLabel)) {
                filteredPairs.add(p);
            }
        }

        Map<Integer, Integer> integerFreq = new HashMap<>();
        for (Pair<String, Integer> p : filteredPairs) {
            int val = p.second;
            integerFreq.put(val, integerFreq.getOrDefault(val, 0) + 1);
        }

        int mostFrequentInt = 0;
        int maxFreq = 0;
        for (Map.Entry<Integer, Integer> entry : integerFreq.entrySet()) {
            if (entry.getValue() > maxFreq) {
                maxFreq = entry.getValue();
                mostFrequentInt = entry.getKey();
            }
        }

        return new Pair<>(maxLabel, mostFrequentInt);
    }

    private void putVisionData(Integer areaNum, Pair<String, Integer> visionData) {
        List<Pair<String, Integer>> list = areaLandmarkDatas.get(areaNum);
        if (list == null) {
            list = new ArrayList<>();
            areaLandmarkDatas.put(areaNum, list);
        }
        list.add(visionData);
    }

    private int argmax(float[] array) {
        int maxIdx = 0;
        for (int i = 1; i < array.length; i++) {
            if (array[i] > array[maxIdx]) maxIdx = i;
        }
        return maxIdx;
    }

    private float[][][] preprocessMat(Mat image, int width, int height) {
        Mat resized = new Mat();
        Imgproc.resize(image, resized, new Size(width, height));
        resized.convertTo(resized, CvType.CV_32FC3, 1.0 / 255);

        float[][][] input = new float[1][3][width * height];
        int index = 0;
        for (int y = 0; y < resized.rows(); y++) {
            for (int x = 0; x < resized.cols(); x++) {
                double[] pixel = resized.get(y, x);
                input[0][0][index] = (float) pixel[0]; // B
                input[0][1][index] = (float) pixel[1]; // G
                input[0][2][index] = (float) pixel[2]; // R
                index++;
            }
        }
        return input;
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