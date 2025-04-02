package jp.jaxa.iss.kibo.rpc.defaultapk.utils;

import android.util.Pair;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AreasItemData {
    private Map<Integer, List<Pair<String, Integer>>> areaVisionDataMap = new HashMap<>();;

    public void putVisionData(Integer areaNum, Pair<String, Integer> visionData) {
        List<Pair<String, Integer>> list = areaVisionDataMap.get(areaNum);
        if (list == null) {
            list = new ArrayList<>();
        }
        list.add(visionData);
        areaVisionDataMap.put(areaNum, list);
    }

    public Pair<String, Integer> getMaxFreqItemData(Integer areaNum) {
        List<Pair<String, Integer>> visionDataList = areaVisionDataMap.get(areaNum);

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
}

