package jp.jaxa.iss.kibo.rpc.defaultapk.utils;

import android.content.Context;
import android.util.Log;
import android.util.Pair;

import org.opencv.core.Mat;

import java.util.*;

import jp.jaxa.iss.kibo.rpc.defaultapk.model.DetectionResult;

import static jp.jaxa.iss.kibo.rpc.defaultapk.Constants.*;

public class ItemDetectorUtils {
    private ONNXDetect onnxDetect;

    private static final String[] treasure_items = {"crystal", "diamond", "emerald"};
    private static final String[] landmark_items = {
            "treasure_box", "coin", "compass", "coral",
            "fossil", "key", "treasure_boxtreasure_box", "shell"};

    private final Map<Integer, List<Pair<String, Integer>>> areaLandmarkDatas = new HashMap<>();
    private final Map<Integer, List<String>> areaTreasureDatas = new HashMap<>();

    public ItemDetectorUtils(Context context) {
        onnxDetect = new ONNXDetect(context);
    }

    private void detectTreasureItem(DetectionResult[] treasureDetectionResults, Integer areaNum){
        if (treasureDetectionResults != null && treasureDetectionResults.length > 0) {
            List<String> treasureList = areaTreasureDatas.get(areaNum);
            if (treasureList == null) {
                treasureList = new ArrayList<>();
                areaTreasureDatas.put(areaNum, treasureList);
            }

            treasureList.add(treasure_items[treasureDetectionResults[0].label]);

            for (DetectionResult result : treasureDetectionResults) {
                Log.i("DEIM_treasure", "Label: " + treasure_items[result.label] + ", Prob: " + result.prob +
                        ", Rect: (" + result.x + "," + result.y + "," + result.width + "," + result.height + ")");
            }
        } else {
            Log.w("DEIM_treasure", "No objects detected");
        }
    }

    private void detectLandmarkItem(DetectionResult[] landMarkDetectionResults, Integer areaNum) {

        if (landMarkDetectionResults == null || landMarkDetectionResults.length == 0) {
            Log.i("Detection", "No objects detected");
            return;
        }

        Log.i("Detection", "Found " + landMarkDetectionResults.length + " objects");

        Map<String, LabelStats> labelStatsMap = new HashMap<>();
        String highestProbLabel = null;
        float maxProb = -1f;

        for (DetectionResult result : landMarkDetectionResults) {
            String label = landmark_items[result.label];
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

            Log.i("DEIM_landmark", "Label: " + landmark_items[result.label] + ", Prob: " + result.prob +
                    ", Rect: (" + result.x + "," + result.y + "," + result.width + "," + result.height + ")");
        }

        int highestScoreLabelCount = highestProbLabel != null ?
                labelStatsMap.get(highestProbLabel).count : 0;

        putVisionData(areaNum, new Pair<>(highestProbLabel, highestScoreLabelCount));
    }

    public String detectRecognizedResult(Mat mat){
        DetectionResult[] detectionResults = onnxDetect.detect(mat,0.5f);
        DetectionResult result = reMappingTreasureData(detectionResults)[0];
        if(result !=null){ return  treasure_items[result.label]; }
        return null;
    }

    private static class LabelStats {
        int count = 0;
        float maxProb = -1f;
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

    public void scanItemBoard(Mat mat, Integer areaNum){
        DetectionResult[] detectionResults = onnxDetect.detect(mat,0.85f);

        DetectionResult[] treasureResults = reMappingTreasureData(detectionResults);
        detectTreasureItem(treasureResults, areaNum);

        DetectionResult[] landmarkResults = reMappingLandmarkData(detectionResults);
        detectLandmarkItem(landmarkResults, areaNum);
    }

    private DetectionResult[] reMappingTreasureData(DetectionResult[] detectionResults) {
        return reMapAndFilter(detectionResults, TREASURE_MAPPING);
    }

    private DetectionResult[] reMappingLandmarkData(DetectionResult[] detectionResults) {
        return reMapAndFilter(detectionResults, LANDMARK_MAPPING);
    }

    private DetectionResult[] reMapAndFilter(DetectionResult[] detectionResults, Map<Integer, Integer> mapping) {
        List<DetectionResult> resultList = new ArrayList<>();
        for (DetectionResult dr : detectionResults) {
            if (dr != null && mapping.containsKey(dr.label)) {
                resultList.add(new DetectionResult(
                        mapping.get(dr.label),
                        dr.prob,
                        dr.x,
                        dr.y,
                        dr.width,
                        dr.height
                ));
            }
        }
        return resultList.toArray(new DetectionResult[0]);
    }
}