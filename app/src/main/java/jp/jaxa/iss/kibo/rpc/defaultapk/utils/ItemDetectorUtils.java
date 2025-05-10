package jp.jaxa.iss.kibo.rpc.defaultapk.utils;

import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.util.Log;
import android.util.Pair;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import jp.jaxa.iss.kibo.rpc.defaultapk.YOLO11Ncnn;
import jp.jaxa.iss.kibo.rpc.defaultapk.model.DetectionResult;
import jp.jaxa.iss.kibo.rpc.defaultapk.model.SegDetectionResult;

public class ItemDetectorUtils {
    private YOLO11Ncnn yolo11Ncnn = new YOLO11Ncnn();
    private AssetManager assetManager;
    private static final String[] tresure_item = {"crystal", "diamond", "emerald"};
    private static final String[] landmark_item =
            {"treasure_box", "coin", "compass", "coral", "fossil", "key", "letter", "shell"};

    private Map<Integer, List<Pair<String, Integer>>> areaLandmarkDatas = new HashMap<>();

    public ItemDetectorUtils(AssetManager assetManager) {
        this.assetManager = assetManager;
    }

    public void detectTresureItem(Bitmap bitmap, Integer areaNum){
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

    public void detectLandmarkItem(Bitmap bitmap, Integer areaNum) {
        yolo11Ncnn.loadModel(assetManager, 1, 3, 0);

        SegDetectionResult[] segResults = yolo11Ncnn.detectSegObjects(bitmap);

        if (segResults == null || segResults.length == 0) {
            Log.i("Detection", "No objects detected");
            return;
        }

        Log.i("Detection", "Found " + segResults.length + " objects");

        Map<String, LabelStats> labelStatsMap = new HashMap<>();
        String highestProbLabel = null;
        float maxProb = -1f;

        for (SegDetectionResult result : segResults) {
            String label = landmark_item[result.label];
            float prob = result.prob;

            LabelStats stats = labelStatsMap.getOrDefault(label, new LabelStats());
            stats.count++;
            if (prob > stats.maxProb) {
                stats.maxProb = prob;
            }
            labelStatsMap.put(label, stats);

            if (prob > maxProb) {
                maxProb = prob;
                highestProbLabel = label;
            }

            Log.i("Detection", String.format(
                    "Label:%s Prob:%.2f Box:[%d,%d,%d,%d] MaskBytes:%d",
                    label, prob, result.x, result.y,
                    result.width, result.height, result.mask.length
            ));
        }

        int highestScoreLabelCount = highestProbLabel != null ?
                labelStatsMap.get(highestProbLabel).count : 0;

        putVisionData(areaNum, new Pair<>(highestProbLabel, highestScoreLabelCount));
    }

    private static class LabelStats {
        int count = 0;
        float maxProb = -1f;
    }

    public Pair<String, Integer> getMaxFreqItemData(Integer areaNum) {
        List<Pair<String, Integer>> visionDataList = areaLandmarkDatas.get(areaNum);

        if (visionDataList == null || visionDataList.isEmpty()) { return null; }

        Map<String, Integer> frequencyMap = new HashMap<>();
        for (Pair<String, Integer> pair : visionDataList) {
            frequencyMap.put(pair.first, frequencyMap.getOrDefault(pair.first, 0) + 1);
        }

        String mostFrequentString = null;
        int maxFrequency = 0;
        for (Map.Entry<String, Integer> entry : frequencyMap.entrySet()) {
            if (entry.getValue() > maxFrequency) {
                maxFrequency = entry.getValue();
                mostFrequentString = entry.getKey();
            }
        }

        List<Pair<String, Integer>> filteredPairs = new ArrayList<>();
        for (Pair<String, Integer> pair : visionDataList) {
            if (pair.first.equals(mostFrequentString)) {
                filteredPairs.add(pair);
            }
        }

        Map<Integer, Integer> integerFrequencyMap = new HashMap<>();
        for (Pair<String, Integer> pair : filteredPairs) {
            integerFrequencyMap.put(pair.second, integerFrequencyMap.getOrDefault(pair.second, 0) + 1);
        }

        int mostFrequentInteger = 0;
        int maxIntegerFrequency = 0;
        for (Map.Entry<Integer, Integer> entry : integerFrequencyMap.entrySet()) {
            if (entry.getValue() > maxIntegerFrequency) {
                maxIntegerFrequency = entry.getValue();
                mostFrequentInteger = entry.getKey();
            }
        }

        return new Pair<>(mostFrequentString, mostFrequentInteger);
    }

    private void putVisionData(Integer areaNum, Pair<String, Integer> visionData) {
        List<Pair<String, Integer>> list = areaLandmarkDatas.get(areaNum);
            if (list == null) {
            list = new ArrayList<>();
        }
        list.add(visionData);
        areaLandmarkDatas.put(areaNum, list);
    }
}
