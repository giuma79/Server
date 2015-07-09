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

enum TELEOPERATION_TYPES {
    NONE(0), GUI_WP(1), GUI_MS(2), RC(3);
    // NONE --> control loops are always active (always try to arrive at and stay at current destination unless algorithm overrides)
    // GUI_WP --> user selects waypoint(s) in a GUI. Boat controls arrival, but the boat does nothing after it arrives
    // GUI_MS --> user is sending motor signals to the boats via a GUI. Boat control loops completely disabled
    // RC --> user is sending motor signals to the boats via a radio controller. Boat control loops completely disabled
    private final long value;
    TELEOPERATION_TYPES(long value) { this.value = value; }
    public final long getLongValue() { return value; }
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
    Integer thrustType; // see THRUST_TYPES enum
    Integer teleopStatus; // see TELEOPERATION_TYPES enum
    Integer gpsInitialized; // == 1 if the first GPS lock has come in
    Integer compassInitialized; // == 1 if the first compass measurement has come in
    Integer localized; // == 1 if both GPS and compass are initialized
    DoubleVector motorCommands;
    final double defaultSufficientProximity = 3.0;
    final double defaultPeakVelocity = 2.0;
    final double defaultAccelTime = 5.0;
    final double defaultDecelTime = 5.0;
    final double maxAccel = 1.0; // no more than X m/s^2 capable at full power
    final double minAccel = 0.1; // no less than X m/s^2, or motor doesn't respond
    final long defaultTeleopStatus = 0L;
    final double controlHz = 25.0; // frequency of control loop and sending the corresponding JSON commands

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
        teleopStatus = new Integer();
        teleopStatus.setName(knowledge,".teleopStatus");
        teleopStatus.set(defaultTeleopStatus);
        gpsInitialized = new Integer();
        gpsInitialized.setName(knowledge,".gpsInitialized");
        gpsInitialized.set(0);
        compassInitialized = new Integer();
        compassInitialized.setName(knowledge,".compassInitialized");
        compassInitialized.set(0);
        localized = new Integer();
        localized.setName(knowledge,".localized");
        localized.set(0);
    }

    public void freeAll() {
        distToDest.free();
        sufficientProximity.free();
        peakVelocity.free();
        accel.free();
        decel.free();
        x.free();
        executingProfile.free();
        teleopStatus.free();
        gpsInitialized.free();
        compassInitialized.free();
        localized.free();
    }

    public void restoreDefaults() {
        sufficientProximity.set(defaultSufficientProximity);
        peakVelocity.set(defaultPeakVelocity);
        accel.set(defaultAccelTime);
        decel.set(defaultDecelTime);
        teleopStatus.set(defaultTeleopStatus);
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

    public double velocityTowardGoal() {
        // calculate the boat's current velocity along the line between its current location and the goal
        RealMatrix initialV = MatrixUtils.createRealMatrix(2,1);
        initialV.setEntry(0,0,this.x.get(3)*Math.cos(this.x.get(2)) - this.x.get(5));
        initialV.setEntry(1,0,this.x.get(3)*Math.sin(this.x.get(2)) - this.x.get(6));
        RealMatrix xd = NDV_to_RM(self.device.dest).subtract(NDV_to_RM(self.device.home));
        RealMatrix x = MatrixUtils.createRealMatrix(2,1);
        x.setEntry(0, 0, this.x.get(0));
        x.setEntry(1,0,this.x.get(1));
        RealMatrix xError = xd.getSubMatrix(0,1,0,0).subtract(x);
        RealMatrix xErrorNormalized = xError.scalarMultiply(1 / RMO.norm2(xError));
        double v = RMO.dot(initialV, xErrorNormalized); // initial speed in the direction of the goal
        return v;
    }

}
