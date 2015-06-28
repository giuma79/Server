package edu.cmu.ri.airboat.server;

import android.util.Log;

import com.gams.controllers.BaseController;
import com.gams.platforms.BasePlatform;
import com.gams.platforms.DebuggerPlatform;
import com.gams.platforms.PlatformStatusEnum;
import com.gams.utility.Position;
import com.gams.utility.Axes;
import com.madara.EvalSettings;
import com.madara.KnowledgeBase;
import com.madara.threads.BaseThread;
import com.madara.threads.Threader;

import org.jscience.geography.coordinates.LatLong;
import org.jscience.geography.coordinates.UTM;
import org.jscience.geography.coordinates.crs.CoordinatesConverter;
import org.jscience.geography.coordinates.crs.ReferenceEllipsoid;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.measure.quantity.Angle;
import javax.measure.unit.NonSI;
import javax.measure.unit.SI;

import edu.cmu.ri.crw.data.Utm;
import edu.cmu.ri.crw.data.UtmPose;
import robotutils.Pose3D;
import robotutils.Quaternion;

/**
 * GAMS BasePlatform implementation
 *
 * @author nbb
 *
 */
//public class LutraPlatform extends BasePlatform {
public class LutraPlatform extends DebuggerPlatform {

    KnowledgeBase knowledge;
    BoatEKF boatEKF;
    BoatMotionController boatMotionController;
    Threader threader;
    class FilterAndControllerThread extends BaseThread {
        @Override
        public void run() {
            if (boatEKF.isGPSInitialized && boatEKF.isCompassInitialized) {
                boatEKF.predict();
                boatMotionController.control();
                //String threadID = String.format(" -- thread # %d", Thread.currentThread().getId());
                //Log.w("jjb", "FilterAndControllerThread iteration" + threadID);
            }
        }
    }

    // Speed profile following --> updated each time profile.move() is called ***
    /*
    * 1) Assume that the boat is already pointed straight at the target (no trajectory planning)
    * 2) Using estimate of current velocity along that ideal line between current point and target point
     *     as initial speed, establish a trapezoidal speed profile.
    * 3) Parallel, independent, and simple PID control on rotation, just like in current POINT_AND_SHOOT.
     *     Any rotation velocity will be superimposed on top of speed commands.
    * 4) P-PI cascaded position and velocity control to follow the trapezoidal velocity profile
    * 5) The actions will be suboptimal for sure, but the trapezoid will be updated several times a second,
     *     and as the boat nears a straight-away, the boat approaches the ideal
    *
    * Alternatively, you could do tank-steering moves at the way points. Point right at the goal from the
     * beginning. Or at least wait until you are pointing within +/- 30 degrees before starting the trapezoid.
     * That seems like a much easier alternative. Otherwise you'll need trajectory generation.
    */
    List<java.lang.Double> velocityProfile;
    double sustainedSpeed;
    double accelTime; // t1
    double sustainTime; // t2
    double finishTime; // t3
    double moveDistance;

    ////////////////////////////////////////////////////////////////////////
    String ipAddress;

    // Delay for sending modified fields
    EvalSettings evalSettings;

    public LutraPlatform(KnowledgeBase knowledge) {
        this.knowledge = knowledge;
        evalSettings = new EvalSettings();
        evalSettings.setDelaySendingModifieds(true);
        threader = new Threader(knowledge);
        boatEKF = new BoatEKF(knowledge);
        boatMotionController = new BoatMotionController(knowledge);
    }

    @Override
    public void init(BaseController controller) {
        super.init(controller);

        //threader.run(100.0, "FilterAndController", new FilterAndControllerThread()); // --> causes a crash
    }

    public void start() {
        threader.run(100.0, "FilterAndController", new FilterAndControllerThread()); // --> also causes a crash
    }


    /**
     * Analyzes the platform.
     *
     * @return status information (@see Status)
     *
     */
    public int analyze() {

        return PlatformStatusEnum.OK.value();
    }

    /**
     *
     *
     */
    public int rotate(Axes axes) {
        return PlatformStatusEnum.OK.value();
    }


    /**
     * Returns the position accuracy in meters
     *
     * @return position accuracy
     *
     */
    public double getPositionAccuracy() {
        return 5.0;
    }

    public double getGpsAccuracy() {
        return 5.0;
    }

    /**
     * Returns the current GPS position
     *
     */
    public Position getPosition() {
        Position position = new Position(self.device.location.get(0), self.device.location.get(1), self.device.location.get(2));
        return position;
    }

    public int home() {
        return PlatformStatusEnum.OK.value();
    }
    public int land() {
        return PlatformStatusEnum.OK.value();
    }

    /**
     * Initializes a move to the target position. This should be a non-blocking
     * call.
     *
     * @param target the new position to move to
     * @param proximity the minimum distance between current position and target
     * position that terminates the move.
     * @return status information (@see Status)
     *
     */
    public int move(Position target, double proximity) {

        // Convert from lat/long to UTM coordinates
        UTM utmLoc = UTM.latLongToUtm(
                LatLong.valueOf(target.getX(), target.getY(), NonSI.DEGREE_ANGLE),
                ReferenceEllipsoid.WGS84);

        // Convert to UTM data structure
        Pose3D pose = new Pose3D(utmLoc.eastingValue(SI.METER),
                utmLoc.northingValue(SI.METER), target.getZ(),
                Quaternion.fromEulerAngles(0, 0, 0));
        Utm origin = new Utm(utmLoc.longitudeZone(),
                utmLoc.latitudeZone() > 'O');
        UtmPose utm = new UtmPose(pose, origin);

        // Write destination and source to device id path
        self.device.dest.set(0, target.getX());
        self.device.dest.set(1, target.getY());
        self.device.dest.set(2, target.getZ());
        self.device.source.set(0, self.device.location.get(0));
        self.device.source.set(1, self.device.location.get(1));
        self.device.source.set(2, self.device.location.get(2));

        return PlatformStatusEnum.OK.value();
    }

    /**
     * Get sensor radius
     *
     * @return minimum radius of all available sensors for this platform
     */
    public double getMinSensorRange() {
        return 0.0;
    }

    /**
     * Gets the movement speed
     *
     * @return movement speed
     *
     */
    public double getMoveSpeed() {
        return 10.0;
    }

    /**
     * Gets the unique id of the platform. This should be an alphanumeric id
     * that can be part of a MADARA variable name. Specifically, this is used in
     * the variable expansion of .platform.{yourid}.
     *
     *
     * @return the id of the platform (alphanumeric only: no spaces!)
     *
     */
    public java.lang.String getId() {
        // Get IP address
        return ipAddress;
    }

    /**
     * Gets the name of the platform
     *
     * @return the name of the platform
     *
     */
    public java.lang.String getName() {
        return "Lutra";
    }

    /**
     * Gets results from the platform's sensors. This should be a non-blocking
     * call.
     *
     * @return 1 if moving, 2 if arrived, 0 if an error occurred
     *
     */
    public int sense() {
        return PlatformStatusEnum.OK.value();
    }

    /**
     * Sets move speed
     *
     * @param speed new speed in meters/second
     *
     */
    public void setMoveSpeed(double speed) {
    }
    public int takeoff() {
        return PlatformStatusEnum.OK.value();
    }
    public void stopMove() {
    }

    /**
     * Conversion method that takes a UTMPose and a variable name in MADARA and
     * set the variable in the provided knowledge base to the given UtmPose.
     *
     * @param knowledge a knowledge base that will be updated
     * @param knowledgePath the name of the variable in the knowledge base that
     * should be updated
     * @param utmPose the UTM pose that the knowledge base will be updated with
     */
    public void setUtmPose(KnowledgeBase knowledge, String knowledgePath, UtmPose utmPose) {

        // Write pose to ip address path
        knowledge.set(knowledgePath + ".x", utmPose.pose.getX(), evalSettings);
        knowledge.set(knowledgePath + ".y", utmPose.pose.getY(), evalSettings);
        knowledge.set(knowledgePath + ".z", utmPose.pose.getZ(), evalSettings);
        knowledge.set(knowledgePath + ".roll", utmPose.pose.getRotation().toRoll(), evalSettings);
        knowledge.set(knowledgePath + ".pitch", utmPose.pose.getRotation().toPitch(), evalSettings);
        knowledge.set(knowledgePath + ".yaw", utmPose.pose.getRotation().toYaw(), evalSettings);
        knowledge.set(knowledgePath + ".zone", utmPose.origin.zone, evalSettings);
        knowledge.set(knowledgePath + ".hemisphere", utmPose.origin.isNorth ? "North" : "South", evalSettings);

        // Write pose to device location path
        UTM utm = UTM.valueOf(utmPose.origin.zone, utmPose.origin.isNorth ? 'N' : 'S', utmPose.pose.getX(), utmPose.pose.getY(), SI.METRE);
        CoordinatesConverter<UTM, LatLong> utmToLatLong = UTM.CRS.getConverterTo(LatLong.CRS);
        LatLong latLong = utmToLatLong.convert(utm);

        self.device.location.set(0, latLong.latitudeValue(NonSI.DEGREE_ANGLE));
        self.device.location.set(1, latLong.longitudeValue(NonSI.DEGREE_ANGLE));
        self.device.location.set(2, utmPose.pose.getZ());
    }

    public void shutdown() {

        // stop threads
        threader.terminate();

        boatEKF.stop();

        // Free MADARA containers
        threader.free();
    }
}
