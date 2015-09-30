package edu.cmu.ri.airboat.server;

import com.madara.KnowledgeBase;
import com.madara.KnowledgeRecord;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

/**
 * @author jjb
 */
public class HysteresisFilter implements DatumListener {

    HashMap<SENSOR_TYPE,List<Double>> heightsHashMap; // 5 numbers, using P squared algorithm for calculating median without storing values
    HashMap<SENSOR_TYPE,List<Integer>> markersHashMap;
    HashMap<SENSOR_TYPE,List<Double>> desiredMarkersHashMap;

    HashMap<SENSOR_TYPE,Boolean> convergedHashMap;
    HashMap<SENSOR_TYPE,Long> dataToKBCount; // number of data points pushed into knowledge base

    KnowledgeBase knowledge;
    LutraMadaraContainers containers;
    boolean dwelling;

    final double percentile = 0.5;
    final double[] increment = new double[] {0, percentile/2.0, percentile, (1.0 + percentile)/2.0, 1};
    final double[] defaultDesiredMarkers = new double[] {1, 1+2*percentile, 1+4*percentile, 3+2*percentile, 5};
    //final int[] defaultMarkers = new int[] {1, 2, 3, 4, 5};

    HysteresisFilter(KnowledgeBase knowledge, LutraMadaraContainers containers) {
        this.knowledge = knowledge;
        this.containers = containers;
        dwelling = false;
        heightsHashMap = new HashMap<>();
        markersHashMap = new HashMap<>();
        desiredMarkersHashMap = new HashMap<>();
        convergedHashMap = new HashMap<>();
        dataToKBCount = new HashMap<>();
        for (SENSOR_TYPE type : SENSOR_TYPE.environmental) {
            convergedHashMap.put(type,!type.hysteresis); // non-hysteresis sensors are inherently converged
        }
    }

    @Override
    public void newDatum(Datum datum) {
        SENSOR_TYPE type = datum.getType();
        String logString = datum.toString();
        if (!isConverged(type)) {
            logString = logString + " -- WARNING: MAY HAVE HYSTERESIS";
            filter(datum);
        }
        else {
            datum.toKnowledgeBase(); //push into knowledge base
            incrementCount(datum.getType());
        }
        // TODO: save logString to file
    }

    public void filter(Datum datum) {
        if (containers.distToDest.get() < containers.sufficientProximity.get()) {
            if (!dwelling) {
                dwelling = true;
                resetAll();
            }
            checkForConvergence(datum);
        }
        else {
            dwelling = false;
            convergedHashMap.put(datum.getType(), false);
        }
    }

    void checkForConvergence(Datum datum) {
        List<Double> entry = heightsHashMap.get(datum.getType());
        if (entry.size() < 5) {
            entry.add(datum.getZ().getEntry(0,0));
            return;
        }
        if (entry.size() == 5) { // need to sort initial list of five values in ASCENDING order
            Collections.sort(entry);
            return;
        }

        boolean converged = false;
        // TODO: P squared method for median without storing values
        



        if (converged) {
            convergedHashMap.put(datum.getType(), true);
        } else {
            convergedHashMap.put(datum.getType(), false);
        }
    }


    void resetAll() {
        for (HashMap.Entry<SENSOR_TYPE, List<Double>> entry : heightsHashMap.entrySet()) {
            resetSpecific(entry.getKey());
        }
    }
    void resetSpecific(SENSOR_TYPE type) {
        heightsHashMap.get(type).clear();
        markersHashMap.get(type).clear();
        desiredMarkersHashMap.get(type).clear();
        for (int i = 0; i < 5; i++) {
            markersHashMap.get(type).add(i);
            desiredMarkersHashMap.get(type).add(defaultDesiredMarkers[i]);
        }
    }
    void allUnconverged() {
        for (HashMap.Entry<SENSOR_TYPE, Boolean> entry : convergedHashMap.entrySet()) {
            entry.setValue(false);
        }
    }
    boolean isConverged(SENSOR_TYPE type) {
        return convergedHashMap.get(type);
    }
    void incrementCount(SENSOR_TYPE type) {
        dataToKBCount.put(type,dataToKBCount.get(type) + 1);
    }


}
