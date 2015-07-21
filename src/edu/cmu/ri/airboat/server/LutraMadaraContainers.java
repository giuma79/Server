package edu.cmu.ri.airboat.server;

import com.gams.utility.Position;
import com.gams.variables.Self;

import com.madara.KnowledgeBase;
import com.madara.KnowledgeRecord;
import com.madara.UpdateSettings;
import com.madara.containers.Double;
import com.madara.containers.Integer;
import com.madara.containers.DoubleVector;
import com.madara.containers.NativeDoubleVector;
import com.madara.containers.String;

import org.apache.commons.math.linear.MatrixUtils;
import org.apache.commons.math.linear.RealMatrix;
import org.jscience.geography.coordinates.LatLong;
import org.jscience.geography.coordinates.UTM;
import org.jscience.geography.coordinates.crs.ReferenceEllipsoid;

import javax.measure.unit.NonSI;
import javax.measure.unit.SI;

import edu.cmu.ri.crw.data.Utm;
import edu.cmu.ri.crw.data.UtmPose;
import robotutils.Pose3D;
import robotutils.Quaternion;

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
    // GUI_MS --> user is sending motor signals to the boats via a GUI (w/ joystick). Boat control loops completely disabled
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
    java.lang.String prefix;
    UpdateSettings settings; // used to force a global variable to not broadcast as if it were local
    // allows other agents to change this value but does not broadcast changes made locally
    // e.g. i want to teleoperate the boat by changing the motor commands directly from the GUI agent

    Double distToDest;
    Double sufficientProximity;
    Double peakVelocity;
    Double accel;
    Double decel;
    DoubleVector x;
    DoubleVector latLong;
    Integer longitudeZone;
    String latitudeZone; // a single character (see UTM) http://jscience.org/api/org/jscience/geography/coordinates/UTM.html
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
        this.prefix = java.lang.String.format("device.%d.",this.self.id.get());
        this.settings = new UpdateSettings();
        settings.setTreatGlobalsAsLocals(true);

        this.self.device.dest.resize(3);
        this.self.device.home.resize(3);
        this.self.device.location.resize(3);
        this.self.device.source.resize(3);
        distToDest = new Double();
        distToDest.setName(knowledge, prefix + "distToDest");
        distToDest.setSettings(settings);
        sufficientProximity = new Double();
        sufficientProximity.setName(knowledge,prefix + "sufficientProximity");
        sufficientProximity.setSettings(settings);
        sufficientProximity.set(defaultSufficientProximity);
        peakVelocity = new Double();
        peakVelocity.setName(knowledge, prefix + "peakVelocity");
        peakVelocity.setSettings(settings);
        peakVelocity.set(defaultPeakVelocity);
        accel = new Double();
        accel.setName(knowledge, prefix + "accelTime");
        accel.setSettings(settings);
        accel.set(defaultAccelTime);
        decel = new Double();
        decel.setName(knowledge, prefix + "decelTime");
        decel.setSettings(settings);
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
        motorCommands.setName(knowledge,prefix + "motorCommands");
        motorCommands.setSettings(settings);
        motorCommands.resize(2);
        teleopStatus = new Integer();
        teleopStatus.setName(knowledge, prefix + "teleopStatus");
        teleopStatus.setSettings(settings);
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
        latLong = new DoubleVector();
        latLong.setName(knowledge,prefix + "latLong");
        latLong.resize(2);
        longitudeZone = new Integer();
        longitudeZone.setName(knowledge, prefix + "longitudeZone");
        longitudeZone.setSettings(settings);
        latitudeZone = new String();
        latitudeZone.setName(knowledge, prefix + "latitudeZone");
        latitudeZone.setSettings(settings);

        settings.free(); // don't need this object past the initialization
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
        motorCommands.free();
        latLong.free();
        longitudeZone.free();
        latitudeZone.free();
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
        double[] DA = NDV_to_DA(NDV);
        RealMatrix result = MatrixUtils.createColumnRealMatrix(DA);
        return result;
    }

    public double[] NDV_to_DA(NativeDoubleVector NDV) {
        // NativeDoubleVector to double[] conversion
        // Useful for pulling out self.device.home, .location, .dest, and .source
        // e.g. double[] home = containers.NDV_to_DA(self.device.home);
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
        RealMatrix x = MatrixUtils.createRealMatrix(2, 1);
        x.setEntry(0, 0, this.x.get(0));
        x.setEntry(1,0,this.x.get(1));
        RealMatrix xError = xd.getSubMatrix(0,1,0,0).subtract(x);
        RealMatrix xErrorNormalized = xError.scalarMultiply(1 / RMO.norm2(xError));
        double v = RMO.dot(initialV, xErrorNormalized); // initial speed in the direction of the goal
        return v;
    }

    public double[] UTMPoseToLocalXY(UtmPose utm) {
        double[] result = new double[] {0.0,0.0};
        double[] home = NDV_to_DA(self.device.home);
        result[0] = utm.pose.getX() - home[0];
        result[1] = utm.pose.getY() - home[1];
        return result;
    }

    public double[] PositionToLocalXY(Position position) {
        double [] result = new double[] {0.0,0.0};
        double[] home = NDV_to_DA(self.device.home);
        // Convert from lat/long to UTM coordinates
        UTM utmLoc = UTM.latLongToUtm(
                LatLong.valueOf(position.getX(), position.getY(), NonSI.DEGREE_ANGLE),
                ReferenceEllipsoid.WGS84);

        // Convert to UTM data structure
        Pose3D pose = new Pose3D(utmLoc.eastingValue(SI.METER),
                utmLoc.northingValue(SI.METER), position.getZ(),
                Quaternion.fromEulerAngles(0, 0, 0));
        Utm origin = new Utm(utmLoc.longitudeZone(),
                utmLoc.latitudeZone() > 'O');
        UtmPose utm = new UtmPose(pose, origin);
        result = UTMPoseToLocalXY(utm);
        return result;
    }

    public LatLong LocalXYToLatLong() {
        UTM utm = LocalXYToUTM();
        LatLong latLong = UTM.utmToLatLong(utm, ReferenceEllipsoid.WGS84);
        return latLong;
    }

    public UTM LocalXYToUTM() {
        double easting = x.get(0) + self.device.home.get(0);
        double northing = x.get(1) + self.device.home.get(1);
        int longitudeZone = (int)this.longitudeZone.get();
        char latitudeZone = this.latitudeZone.get().charAt(0);
        UTM utm = UTM.valueOf(longitudeZone, latitudeZone, easting, northing, SI.METER);
        return utm;
    }



}
