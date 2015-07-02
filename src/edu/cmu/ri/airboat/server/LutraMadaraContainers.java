package edu.cmu.ri.airboat.server;

import com.gams.variables.Self;
import com.madara.KnowledgeBase;
import com.madara.KnowledgeRecord;
import com.madara.containers.DoubleVector;
import com.madara.containers.Double;
import com.madara.containers.NativeDoubleVector;

import org.apache.commons.math.linear.MatrixUtils;
import org.apache.commons.math.linear.RealMatrix;

/**
 * @author jjb
 */
public class LutraMadaraContainers {

    // Pass in knowledge base
    // Takes care of all the SetName's for all the code at once - no more hard coding strings in multiple locations!
    // Then just pass this object around in the constructors for things like the EFK and controller

    KnowledgeBase knowledge;
    Double distToDest;
    Double sufficientProximity;
    Double peakVelocity;
    Double accel;
    Double decel;
    DoubleVector x;

    Self self;

    public LutraMadaraContainers(KnowledgeBase knowledge, Self self) {
        this.knowledge = knowledge;
        this.self = self;
        distToDest = new Double();
        distToDest.setName(knowledge,".distToDest");
        sufficientProximity = new Double();
        sufficientProximity.setName(knowledge,".sufficientProximity");
        peakVelocity = new Double();
        peakVelocity.setName(knowledge, ".peakVelocity");
        accel = new Double();
        accel.setName(knowledge, ".accel");
        decel = new Double();
        decel.setName(knowledge, ".decel");
        x = new DoubleVector();
        x.setName(knowledge, ".x");
    }

    public void freeAll() {
        distToDest.free();
        sufficientProximity.free();
        peakVelocity.free();
        accel.free();
        decel.free();
        x.free();
    }

    public RealMatrix NDV_to_RM(NativeDoubleVector NDV) {
        // NativeDoubleVector to RealMatrix column conversion
        // Useful for pulling out self.device.home, .location, .dest, and .source
        KnowledgeRecord KR = NDV.toRecord();
        double[] DA = KR.toDoubleArray();
        RealMatrix result = MatrixUtils.createColumnRealMatrix(DA);
        return result;
    }

    public double[] NDV_to_DA(NativeDoubleVector NDV) {
        // NativeDoubleVector to double[] conversion
        // Useful for pulling out self.device.home, .location, .dest, and .source
        KnowledgeRecord KR = NDV.toRecord();
        double[] result = KR.toDoubleArray();
        return result;
    }

}
