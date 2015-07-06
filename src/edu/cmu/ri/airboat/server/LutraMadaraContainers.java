package edu.cmu.ri.airboat.server;

import com.gams.variables.Self;
import com.madara.KnowledgeBase;
import com.madara.KnowledgeRecord;
import com.madara.containers.Double;
import com.madara.containers.DoubleVector;
import com.madara.containers.NativeDoubleVector;
import com.madara.containers.Integer;

import org.apache.commons.math.linear.MatrixUtils;
import org.apache.commons.math.linear.RealMatrix;

enum THRUST_TYPES {
    VECTORED(0), DIFFERENTIAL(1);
    private final long value;
    THRUST_TYPES(long value) { // constructor allows for special types in the enum
        this.value = value;
    }
    public final long getLongValue() { // need a get function to access the values
        return value;
    }
}

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
    Integer executingProfile; // == 1 if controller is currently executing a velocity profile, == 0 otherwise
    Integer thrustType;
    DoubleVector motorCommands;
    final double defaultSufficientProximity = 3.0;
    final double defaultPeakVelocity = 2.0;
    final double defaultAccelTime = 5.0;
    final double defaultDecelTime = 5.0;
    final double maxAccel = 1.0; // no more than X m/s^2 capable at full power
    final double minAccel = 0.1; // no less than X m/s^2, or motor doesn't respond

    Self self;

    public LutraMadaraContainers(KnowledgeBase knowledge, Self self, THRUST_TYPES thrustType) {
        this.knowledge = knowledge;
        this.self = self;
        this.self.device.dest.resize(3);
        this.self.device.home.resize(3);
        this.self.device.location.resize(3);
        this.self.device.source.resize(3);
        distToDest = new Double();
        distToDest.setName(knowledge,".distToDest");
        sufficientProximity = new Double();
        sufficientProximity.setName(knowledge, ".sufficientProximity");
        sufficientProximity.set(defaultSufficientProximity);
        peakVelocity = new Double();
        peakVelocity.setName(knowledge, ".peakVelocity");
        peakVelocity.set(defaultPeakVelocity);
        accel = new Double();
        accel.setName(knowledge, ".accelTime");
        accel.set(defaultAccelTime);
        decel = new Double();
        decel.setName(knowledge, ".decelTime");
        decel.set(defaultDecelTime);
        x = new DoubleVector();
        x.setName(knowledge, ".x");
        executingProfile = new Integer();
        executingProfile.setName(knowledge, ".executingProfile");
        executingProfile.set(0);
        this.thrustType = new Integer();
        this.thrustType.setName(knowledge, ".thrustType");
        this.thrustType.set(thrustType.getLongValue());
        motorCommands = new DoubleVector();
        motorCommands.setName(knowledge,".motorCommands");
    }

    public void freeAll() {
        distToDest.free();
        sufficientProximity.free();
        peakVelocity.free();
        accel.free();
        decel.free();
        x.free();
        executingProfile.free();
    }

    public void restoreDefaults() {
        sufficientProximity.set(defaultSufficientProximity);
        peakVelocity.set(defaultPeakVelocity);
        accel.set(defaultAccelTime);
        decel.set(defaultDecelTime);
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
