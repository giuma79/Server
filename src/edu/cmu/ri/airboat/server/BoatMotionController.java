package edu.cmu.ri.airboat.server;

import android.util.Log;

import com.madara.KnowledgeBase;

import org.apache.commons.math.linear.MatrixUtils;
import org.apache.commons.math.linear.RealMatrix;

/**
 * @author jjb
 */
public class BoatMotionController implements VelocityProfileListener {

    int stateSize;
    RealMatrix x;
    RealMatrix xd;
    RealMatrix profile; // current velocity profile to follow
    RealMatrix xError;
    RealMatrix xErrorOld; // used for derivative control
    KnowledgeBase knowledge;
    Long t;
    Long tOld;
    double dt;
    LutraMadaraContainers containers;
    final double headingErrorThreshold = 20.0*Math.PI/180.0;
    final double[][] simplePIDGains = new double[][]{{0.2,0,0.3},{0.5,0.5,0.5}}; // rows: position, heading; cols: P,I,D
    final double[] PPIGains = new double[]{1.0,1.0,1.0}; // cols: Pos-P, Vel-P, Vel-I
    double PPIErrorAccumulator; // just velocity error
    double[] simplePIDErrorAccumulator; // cols: x,y,th
    public static final double SAFE_DIFFERENTIAL_THRUST = 0.4;
    public static final double SAFE_VECTORED_THRUST = 0.6;
    double headingSignal;
    double distanceSignal;
    DatumListener datumListener;


    public BoatMotionController(KnowledgeBase knowledge, BoatEKF boatEKF, LutraMadaraContainers containers) {
        this.knowledge = knowledge;
        this.containers = containers;
        this.stateSize = boatEKF.stateSize;
        x = MatrixUtils.createRealMatrix(this.stateSize,1);
        xError = MatrixUtils.createRealMatrix(4,1); // [x y th v]
        xErrorOld = MatrixUtils.createRealMatrix(4,1); // [x y th v]
        xd = MatrixUtils.createRealMatrix(2,1); // [x y]
        PPIErrorAccumulator = 0;
        simplePIDErrorAccumulator = new double[]{0,0,0};
        t = System.currentTimeMillis();
        datumListener = boatEKF;
    }

    public void control() {
        updateFromKnowledgeBase();
        xErrorOld = xError.copy();
        tOld = t;
        t = System.currentTimeMillis();
        dt = (t.doubleValue()-tOld.doubleValue())/1000.0;

        // current position error
        xError.setSubMatrix(xd.getSubMatrix(0, 1, 0, 0).subtract(x.getSubMatrix(0, 1, 0, 0)).getData(), 0, 0);
        // current heading error
        // Error magnitude must be <= 180 degrees. Wrap the error into [0,180]
        double angleToGoal = Math.atan2(xError.getEntry(1,0),xError.getEntry(0,0));
        double angleError = x.getEntry(2,0) - angleToGoal;
        while (Math.abs(angleError) > Math.PI) {
            angleError = angleError - Math.signum(angleError)*2*Math.PI;
        }
        xError.setEntry(2, 0, angleError);

        // if you are within a sufficient distance from the goal, stop following velocity profiles
        containers.distToDest.set(RMO.distance(x.getSubMatrix(0, 1, 0, 0), xd.getSubMatrix(0, 1, 0, 0)));
        if (containers.distToDest.get() < containers.sufficientProximity.get()) {
            containers.executingProfile.set(0);
            // reset PPICascade accumulated error variables
            PPIErrorAccumulator = 0;
        }

        String distanceString = String.format("Distance from x to xd = %5.3e",containers.distToDest.get());
        Log.w("jjb",distanceString);

        // determine which controller to use, simple PID or P-PI pos./vel. cascade
        if (containers.executingProfile.get() == 1) {
            //TODO: find current velocity error using velocity profile
            // Need to find where we hoped to be on the velocity profile curve
            PPICascade(xError);
        }
        else {
            simplePID(xError);
        }

        motorCommands();
    }

    void simplePID(RealMatrix xError) {
        // Operate on x,y, and theta concurrently.
        // The boat's heading should converge to the direction of water flow (i.e. fx,fy)
        // That way the boat just needs to go straight forward to stay on the right spot

        for (int i = 0; i < 3; i++) {
            simplePIDErrorAccumulator[i] += xError.getEntry(i,0);
        }
        RealMatrix xErrorDiff = xError.subtract(xErrorOld).scalarMultiply(1.0/dt);

        // HEADING
        // P
        // I
        // D
        headingSignal = simplePIDGains[1][0]*xError.getEntry(2,0) +
                        simplePIDGains[1][1]*simplePIDErrorAccumulator[2] +
                        simplePIDGains[1][2]*xErrorDiff.getEntry(2,0);

        // POSITION
        // P
        // I
        // D
        /*
        double distanceSignalX = simplePIDGains[0][0]*xError.getEntry(0,0) +
                simplePIDGains[0][1]*simplePIDErrorAccumulator[0] +
                simplePIDGains[0][2]*xErrorDiff.getEntry(0,0);
        double distanceSignalY = simplePIDGains[0][0]*xError.getEntry(1,0) +
                simplePIDGains[0][1]*simplePIDErrorAccumulator[1] +
                simplePIDGains[0][2]*xErrorDiff.getEntry(1,0);
        distanceSignal = distanceSignalX*distanceSignalX + distanceSignalY*distanceSignalY;
        */
        //double distance = Math.pow(xError.getEntry(0,0),2) + Math.pow(xError.getEntry(1,0),2);
        distanceSignal = simplePIDGains[0][0]*1.0;
    }

    void PPICascade(RealMatrix xError) {
        // Operate in two phases
        // If the theta error is above some threshold, focus purely on that, ignoring the velocity profile
        // If the theta error is below that threshold, execute the P-PI cascade on the velocity profile
        // As long as the theta error remains below that threshold, you assume the controller is solving
        //   a 1-D problem, modulating the velocity independently of a simple PID that is correcting theta error

        // Determine which phase you are in based on current theta error
        // PID for heading is always occurring
        if (Math.abs(xError.getEntry(2, 0)) < headingErrorThreshold) {

        }

        double vd = 0, wd= 0; // the desired velocities


        double[] motorSignals = desiredVelocityToMotorSignalMap(vd, wd);
        distanceSignal = motorSignals[0];
        headingSignal = motorSignals[1];
    }

    double[] desiredVelocityToMotorSignalMap(double v, double w) {
        double[] result = new double[2];
        //TODO: build the steady state map
        // map from steady state velocity and % of safe motor signal
        // map from steady state rotational velocity and % of safe motor-momentarm couple
        // ***After finding the necessary motor commands for thrust and rotation, need to combine them
        //      and scale them down to create the desired result
        result[0] = 0;
        result[1] = 0;
        return result;
    }

    double[] motorSignalToVelocityInverseMap(double m0, double m1) {
        double[] result = new double[2];
        // TODO: invert the steady state map
        // map from two motor signals to forward velocity and rotation velocity
        result[0] = 0; // forward velocity
        result[1] = 0; // rotation velocity
        return result;
    }

    void motorCommands() {
        // send motor signals
        if (containers.thrustType.get() == THRUST_TYPES.DIFFERENTIAL.getLongValue()) {
            containers.motorCommands.set(0,map(clip(distanceSignal - headingSignal,-1,1)
                    ,-1, 1,-SAFE_DIFFERENTIAL_THRUST, SAFE_DIFFERENTIAL_THRUST));
            containers.motorCommands.set(1,map(clip(distanceSignal + headingSignal,-1,1)
                    ,-1, 1,-SAFE_DIFFERENTIAL_THRUST, SAFE_DIFFERENTIAL_THRUST));

        }
        else if (containers.thrustType.get() == THRUST_TYPES.VECTORED.getLongValue()) {
            containers.motorCommands.set(0,map(clip(distanceSignal,-1,1)
                    ,-1,1,-SAFE_VECTORED_THRUST,SAFE_VECTORED_THRUST));
            containers.motorCommands.set(1,clip(headingSignal, -1, 1));
        }

        double[] velocities = motorSignalToVelocityInverseMap(containers.motorCommands.get(0),containers.motorCommands.get(1));
        // send intended steady state velocities to the localization filter
        RealMatrix z = MatrixUtils.createRealMatrix(2,1);
        z.setEntry(0,0,velocities[0]);
        z.setEntry(1,0,velocities[1]);
        RealMatrix R = MatrixUtils.createRealMatrix(2,2); // all zeros b/c this is a "perfect" sensor
        t = System.currentTimeMillis();
        Datum datum = new Datum(SENSOR_TYPES.MOTOR,t,z,R);
        datumListener.newDatum(datum);
    }

    void updateFromKnowledgeBase() {
        // remember to subtract device.{.id}.home from the destination so xd is centered about (0,0) like x
        xd = containers.NDV_to_RM(containers.self.device.dest).subtract(containers.NDV_to_RM(containers.self.device.home));

        // update current state
        for (int i = 0; i < stateSize; i++) {
            x.setEntry(i,0,containers.x.get(i));
        }

        Log.w("jjb","xd = " + RMO.realMatrixToString(xd));
        Log.w("jjb","x = " + RMO.realMatrixToString(x));
    }

    public void newProfile(RealMatrix profile, double sufficientProximity) {
        this.profile = profile;
        containers.executingProfile.set(1);
        // set basic PID error accumulators to zero
        simplePIDErrorAccumulator = new double[]{0,0,0};
    }

    public void shutdown() {
    }

    double clip(double value,double min, double max) {
        double result = value;
        if (value > max) { result = max; }
        if (value < min) { result =  min; }
        return result;
    }

    double map(double input, double input_min, double input_max,
                             double output_min, double output_max) {
        return (input - input_min) / (input_max - input_min)
                * (output_max - output_min) + output_min;
    }



}
