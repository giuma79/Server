package edu.cmu.ri.airboat.server;

import com.madara.KnowledgeBase;
import com.madara.KnowledgeRecord;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * @author jjb
 */
public class HysteresisFilter implements DatumListener {

    HashMap<SENSOR_TYPE,List<Double>> datumHashMap;
    HashMap<SENSOR_TYPE,Boolean> convergedHashMap;
    HashMap<SENSOR_TYPE,Long> dataToKBCount; // number of data points pushed into knowledge base
    KnowledgeBase knowledge;
    LutraMadaraContainers containers;
    boolean dwelling;
    final int MIN_NUMBER_OF_SAMPLES = 5;

    HysteresisFilter(KnowledgeBase knowledge, LutraMadaraContainers containers) {
        this.knowledge = knowledge;
        this.containers = containers;
        dwelling = false;
        datumHashMap = new HashMap<>();
        convergedHashMap = new HashMap<>();
        dataToKBCount = new HashMap<>();
    }

    public void filter() {
        if (containers.distToDest.get() < containers.sufficientProximity.get()) {
            if (dwelling) {
                checkForConvergence();
            }
            else { // you are now near your goal but you weren't dwelling already, clear the datum list
                dwelling = true;
                clearAll(); // clear all the datum lists
            }
        }
        else {
            dwelling = false;
            clearAll(); // you aren't near your goal, clear all the datum lists
        }
    }

    @Override
    public void newDatum(Datum datum) {
        SENSOR_TYPE type = datum.getType();
        if (type.hysteresis && dwelling) {
            if (!convergedHashMap.get(datum.getType())) {
                datumHashMap.get(datum.getType()).add(datum.getZ().getEntry(0, 0));
            }
            else {
                datum.toKnowledgeBase(); // push into knowledge base
            }
        }
        else {
            datum.toKnowledgeBase(); //push into knowledge base
        }
        // TODO: log all environmental data, perhaps include possible hysteresis warning flag?
    }

    void checkForConvergence() {
        // iterate over the hashmap. if it isn't already converged, check for convergence

        for (HashMap.Entry<SENSOR_TYPE, List<Double>> entry : datumHashMap.entrySet()) {
            if (!convergedHashMap.get(entry.getKey())) { // not already converged
                if (entry.getValue().size() >= MIN_NUMBER_OF_SAMPLES) {
                    Double[] values = new Double[entry.getValue().size()];
                    entry.getValue().toArray(values);
                    boolean converged = false;

                    // TODO: run tests of values, fit some kind of function (exponential?) to the vector of data, make sure the slope is less than a cutoff? Need to normalize data to make this universal?

                    if (converged) {
                        convergedHashMap.put(entry.getKey(), true);
                        clearSpecific(entry.getKey()); // clear specific - start pushing good data into knowledge base instead of these lists
                    }
                }
                else {
                    convergedHashMap.put(entry.getKey(), false);
                }
            }
        }
    }

    void clearAll() {
        datumHashMap.clear();
    }

    void clearSpecific(SENSOR_TYPE type) {
        datumHashMap.get(type).clear();
    }

    void allUnconverged() {
        for (HashMap.Entry<SENSOR_TYPE, Boolean> entry : convergedHashMap.entrySet()) {
            entry.setValue(false);
        }
    }


}
