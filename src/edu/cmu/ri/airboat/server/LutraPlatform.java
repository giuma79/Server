package edu.cmu.ri.airboat.server;

import com.gams.controllers.BaseController;
import com.gams.platforms.BasePlatform;
import com.gams.platforms.PlatformStatusEnum;
import com.gams.utility.Position;
import com.gams.utility.Axes;
import com.madara.EvalSettings;
import com.madara.KnowledgeBase;
import com.madara.threads.BaseThread;
import com.madara.threads.Threader;

import org.apache.commons.math.linear.MatrixUtils;
import org.apache.commons.math.linear.RealMatrix;
import org.jscience.geography.coordinates.LatLong;
import org.jscience.geography.coordinates.UTM;

import javax.measure.unit.NonSI;

import edu.cmu.ri.crw.data.UtmPose;


/**
 * GAMS BasePlatform implementation
 *
 * @author jjb
 *
 */
public class LutraPlatform extends BasePlatform {
//public class LutraPlatform extends DebuggerPlatform {

    KnowledgeBase knowledge;

    BoatEKF boatEKF;
    BoatMotionController boatMotionController;
    Threader threader;
    boolean homeSet = false;
    VelocityProfileListener velocityProfileListener;
    LutraMadaraContainers containers;
    THRUST_TYPES thrustType;
    Long t;


    class FilterAndControllerThread extends BaseThread {
        @Override
        public void run() {
            //if (boatEKF.isGPSInitialized && boatEKF.isCompassInitialized) {
            if (containers.localized.get() == 1) {
                if (!homeSet) {
                    homeSet = true;
                    knowledge.print();
                }
                boatEKF.predict();
                boatMotionController.control();
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////
    String ipAddress;

    // Delay for sending modified fields
    EvalSettings evalSettings;

    public LutraPlatform(KnowledgeBase knowledge, THRUST_TYPES thrustType) {
        this.knowledge = knowledge;
        evalSettings = new EvalSettings();
        evalSettings.setDelaySendingModifieds(true);
        threader = new Threader(knowledge);
        this.thrustType = thrustType;
    }

    @Override
    public void init(BaseController controller) {
        super.init(controller);
        containers = new LutraMadaraContainers(knowledge,self,thrustType); // has to occur AFTER super.init, or "self" will be null
        boatEKF = new BoatEKF(knowledge,containers); // has to occur AFTER containers() b/c it needs "self"
        boatMotionController = new BoatMotionController(knowledge,boatEKF,containers);
        velocityProfileListener = boatMotionController;
    }

    public void start() {
        threader.run(containers.controlHz, "FilterAndController", new FilterAndControllerThread());
    }


    /**
     * Analyzes the platform.
     *
     * @return status information (@see Status)
     *
     */
    public int analyze() {

        //TODO: Should make an algorithm to follow a chain of waypoints by changing the platform destination when you get near the current destination
        //TODO: Algorithm that simply listens to the GUI for changes to its Madara variables
        /*
        * Look at distance from current location to desired location.
        * If this distance is above some threshold, call move() to generate velocity profile.
        * Generation of this profile will cause the controller to execute a P-PI cascade.
        * If this distance is below the threshold, do nothing (a simple PID will be running to
        *   automatically enforce station keeping).
        *
        * Platform doesn't get to change its destination. Algorithm or a user does that. Each iteration
        *   in the MAPE loop, the platform just looks where it is and where its destination is.
        */

        t = System.currentTimeMillis();
        /*///////////////////////////////////////////
        if ((containers.localized.get() == 1) && (containers.executingProfile.get() != 1)) {
            double[] goal = new double[]{10, 10};
            moveLocalXY(goal, 1.5);
        }
        *////////////////////////////////////////////
        if ((containers.distToDest.get() > containers.sufficientProximity.get()) &&
                                           (containers.executingProfile.get() != 1)) {

            createProfile(containers.sufficientProximity.get(), containers.peakVelocity.get(),0);
        }

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

    void moveLocalXY(double[] localTarget, double proximity) {
        // need to reset motion controller PID error accumulators, start from zero for a new goal
        boatMotionController.zeroErrors();

        // moving in local X,Y requires recreation of Utm global X,Y using current home container Utm

        double[] home = containers.NDV_to_DA(self.device.home);
        self.device.dest.set(0,localTarget[0]+home[0]);
        self.device.dest.set(1,localTarget[1]+home[1]);
        self.device.source.set(0, self.device.location.get(0) + home[0]);
        self.device.source.set(1, self.device.location.get(1) + home[1]);
        containers.sufficientProximity.set(proximity);
    }

    public int move(Position target, double proximity) {
        moveLocalXY(containers.PositionToLocalXY(target), proximity);
        return PlatformStatusEnum.OK.value();
    }

    public int move(UTM utm, double proximity) {
        moveLocalXY(containers.UTMToLocalXY(utm), proximity);
        return PlatformStatusEnum.OK.value();
    }

    public void createProfile(double proximity, double sustainedSpeed, double finalSpeed) {
        /*////////////////////////////////
        * There are two primary controllers. The first is simple PID on position error, called
        *   "station keeping", or "dwelling". The second is a P-PI position-velocity cascade.
        * This second controller requires a velocity profile to follow.
        *
        * createProfile is called to generate a new velocity profile from current position to a new position.
        *
        */
        final int timeSteps = 100;
        RealMatrix velocityProfile = MatrixUtils.createRealMatrix(timeSteps, 3); // t, vel., pos.
        /*
        RealMatrix initialV = MatrixUtils.createRealMatrix(2,1);
        initialV.setEntry(0,0,containers.x.get(3)*Math.cos(containers.x.get(2)) - containers.x.get(5));
        initialV.setEntry(1,0,containers.x.get(3)*Math.sin(containers.x.get(2)) - containers.x.get(6));
        RealMatrix xd = containers.NDV_to_RM(containers.self.device.dest).subtract(containers.NDV_to_RM(containers.self.device.home));
        RealMatrix x = MatrixUtils.createRealMatrix(2,1);
        x.setEntry(0, 0, containers.x.get(0));
        x.setEntry(1,0,containers.x.get(1));
        RealMatrix xError = xd.getSubMatrix(0,1,0,0).subtract(x);
        RealMatrix xErrorNormalized = xError.scalarMultiply(1 / RMO.norm2(xError));
        double v0 = RMO.dot(initialV, xErrorNormalized); // initial speed in the direction of the goal
        */
        double v0 = containers.velocityTowardGoal();
        double vs = sustainedSpeed;
        double vf = finalSpeed;
        double t0 = t.doubleValue()/1000.0;
        double ta = containers.defaultAccelTime;
        double a = (vs-v0)/ta;
        double[] clippedAccel = clipAccel(a,ta);
        a = clippedAccel[0];
        ta = clippedAccel[1];
        double td = containers.defaultDecelTime;
        double d = (vf-vs)/td;
        clippedAccel = clipAccel(d,td);
        d = clippedAccel[0];
        td = clippedAccel[1];
        double L = containers.distToDest.get();
        double ts = 1/vs*(L - 0.5*a*ta*ta - v0*ta - 0.5*d*td*td - vs*td);
        double tf = ta+td+ts;
        t0 = 0; // need the curve to start at relative zero
        /*
        velocityProfile.setEntry(0,0,0);
        velocityProfile.setEntry(0,1,v0);
        velocityProfile.setEntry(1,0,ta);
        velocityProfile.setEntry(1,1,vs);
        velocityProfile.setEntry(2,0,ta+ts);
        velocityProfile.setEntry(2,1,vs);
        velocityProfile.setEntry(3,0,tf);
        velocityProfile.setEntry(3,1,vf);
        */

        //TODO: create desired location column for the profile

        RealMatrix timeMatrix = RMO.linspace(t0,tf,velocityProfile.getRowDimension());
        velocityProfile.setColumn(0, timeMatrix.getColumn(0));
        double dT = L;
        for (int i = 0; i < velocityProfile.getRowDimension(); i++) {
            double T = velocityProfile.getEntry(i,0);
            double vT = 0;
            if (T < ta) { // acceleration period
                vT = v0 + T*a;
            }
            else if ((T > ta) && T < ta+ts) {
                vT = vs;
            }
            else if (T > ta+ts) {
                vT = vs + (T-(ta+ts))*d;
            }
            velocityProfile.setEntry(i, 1, vT);
            if (i == 0) {
                dT = L;
            }
            else if (i == velocityProfile.getRowDimension()-1) {
                dT = 0;
            }
            else {
                dT = dT - (velocityProfile.getEntry(i-1,1)+vT)/2
                        *(velocityProfile.getEntry(i,0)-velocityProfile.getEntry(i-1,0));
            }
            velocityProfile.setEntry(i, 2, dT);
        }

        velocityProfileListener.newProfile(velocityProfile,proximity);
        ///////////////////////////////////
    }

    double[] clipAccel(double a, double t) {
        if (Math.abs(a) > containers.maxAccel) { // need lower acceleration, higher time
            // a --> maxAccel, a = maxAccel/a*a
            // ta --> a/maxAccel*ta
            a = containers.maxAccel;
            t = t*containers.maxAccel/a;
        }
        else if (Math.abs(a) < containers.minAccel) { // need higher acceleration, lower time
            a = containers.minAccel;
            t = t*containers.minAccel/a;
        }
        double[] result = new double[]{a,t};
        return result;
    }

    @Override
    public double getAccuracy() {
        return 0;
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

        // move local .x localization state into device.id.location
        // remember to add in device.id.home because .x is about (0,0)
        double[] home = containers.NDV_to_DA(self.device.home);
        self.device.location.set(0,containers.x.get(0) + home[0]);
        self.device.location.set(1,containers.x.get(1) + home[1]);
        self.device.location.set(2,containers.x.get(2));

        LatLong latLong = containers.LocalXYToLatLong();
        containers.latLong.set(0,latLong.latitudeValue(NonSI.DEGREE_ANGLE));
        containers.latLong.set(1,latLong.longitudeValue(NonSI.DEGREE_ANGLE));

        knowledge.print();


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

        /*
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
        */
    }

    public void shutdown() {

        boatEKF.shutdown();
        boatMotionController.shutdown();

        // stop threads
        threader.terminate();

        // free any containers
        containers.freeAll();

        // Free threader
        threader.free();
    }
}
