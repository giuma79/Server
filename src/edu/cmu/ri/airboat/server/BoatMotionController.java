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
    RealMatrix xErrorDiff; // used for derivative control
    KnowledgeBase knowledge;
    Long t;
    Long tOld;
    double dt;
    double t0; // time at the start of the velocity profile
    boolean t0set;
    LutraMadaraContainers containers;
    final double headingErrorThreshold = 20.0*Math.PI/180.0;
    final double[][] simplePIDGains = new double[][]{{0.2,0,0.3},{0.5,0.5,0.5}}; // rows: position, heading; cols: P,I,D
    final double[] PPIGains = new double[]{1.0,1.0,1.0}; // cols: Pos-P, Vel-P, Vel-I
    double PPIErrorAccumulator; // [Pos-P*(pos error) + vel error] accumulation
    double[] simplePIDErrorAccumulator; // cols: x,y,th
    public static final double SAFE_DIFFERENTIAL_THRUST = 0.4;
    public static final double SAFE_VECTORED_THRUST = 0.6;
    double headingSignal;
    double thrustSignal;
    DatumListener datumListener;
    VelocityMotorMap velocityMotorMap;


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
        velocityMotorMap = new VelocityMotorMap(containers);
    }

    public void zeroErrors() {
        simplePIDErrorAccumulator = new double[] {0.0,0.0,0.0};
        PPIErrorAccumulator = 0.0;
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
        double angleToGoal = Math.atan2(xError.getEntry(1,0),xError.getEntry(0,0));
        double angleError = x.getEntry(2,0) - angleToGoal;
        // Error magnitude must be <= 180 degrees. Wrap the error into [0,180]
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

        // Heading PID control is always operating!
        for (int i = 0; i < 3; i++) {
            simplePIDErrorAccumulator[i] += xError.getEntry(i,0)*dt;
        }
        xErrorDiff = xError.subtract(xErrorOld).scalarMultiply(1.0/dt);
        headingSignal = simplePIDGains[1][0]*xError.getEntry(2,0) + // P
                simplePIDGains[1][1]*simplePIDErrorAccumulator[2] + // I
                simplePIDGains[1][2]*xErrorDiff.getEntry(2,0); // D

        if (containers.teleopStatus.get() == TELEOPERATION_TYPES.NONE.getLongValue()) {
            /////////////////////////////////////////////////////////////////////////////////////
            // determine which controller to use, simple PID or P-PI pos./vel. cascade
            if (containers.executingProfile.get() == 1) {
                PPICascade();
            } else {
                simplePID();
            }
            motorCommands();
            /////////////////////////////////////////////////////////////////////////////////////
        }
        else { // some form of teleoperation is occurring, so don't accumulate error and don't try to control anything
            zeroErrors();
        }

    }

    void simplePID() {
        // Operate on x,y, and theta concurrently.
        // The boat's heading should converge to the direction of water flow (i.e. fx,fy)
        // That way the boat just needs to go straight forward to stay on the right spot

        // POSITION
        // P
        // I
        // D
        /*
        double thrustSignalX = simplePIDGains[0][0]*xError.getEntry(0,0) +
                simplePIDGains[0][1]*simplePIDErrorAccumulator[0] +
                simplePIDGains[0][2]*xErrorDiff.getEntry(0,0);
        double thrustSignalY = simplePIDGains[0][0]*xError.getEntry(1,0) +
                simplePIDGains[0][1]*simplePIDErrorAccumulator[1] +
                simplePIDGains[0][2]*xErrorDiff.getEntry(1,0);
        thrustSignal = thrustSignalX*thrustSignalX + thrustSignalY*thrustSignalY;
        */
        //double distance = Math.pow(xError.getEntry(0,0),2) + Math.pow(xError.getEntry(1,0),2);
        thrustSignal = simplePIDGains[0][0]*1.0;
    }

    void PPICascade() {
        // Operate in two phases
        // If the theta error is above some threshold, focus purely on that, ignoring the velocity profile
        // If the theta error is below that threshold, execute the P-PI cascade on the velocity profile
        // As long as the theta error remains below that threshold, you assume the controller is solving
        //   a 1-D problem, modulating the velocity independently of a simple PID that is correcting theta error

        // Determine which phase you are in based on current theta error
        // PID for heading is always occurring
        double vd; // the desired velocity
        double dd; // the desired distance from target

        if (Math.abs(xError.getEntry(2, 0)) < headingErrorThreshold) {
            if (!t0set) {
                t0 = t.doubleValue()/1000.0;
                t0set = true;
            }
            double tRelative = t.doubleValue()/1000.0 - t0;
            // linear interpolate desired velocity
            vd = RMO.interpolate1D(profile,tRelative,1);
            dd = RMO.interpolate1D(profile,tRelative,2);

            // use actual velocity towards goal (use velocityTowardGoal()), actual distance from goal (distToDest container) to generate errors
            double vError = vd - containers.velocityTowardGoal();
            double dError = dd - containers.distToDest.get();
            //TODO: determine thrustSignal based on P-PI error signals
            PPIErrorAccumulator += PPIGains[0]*dd + vd;
            thrustSignal = PPIGains[1]*(PPIGains[0]*dd + vd) + PPIGains[2]*PPIErrorAccumulator;
        }
        else {
            //TODO: perhaps make this exponentially decay each iteration instead?
            thrustSignal *= 0.5;
        }
    }

    void motorCommands() {
        // send motor signals
        double signal0 = 0;
        double signal1 = 0;
        if (containers.thrustType.get() == THRUST_TYPES.DIFFERENTIAL.getLongValue()) {
            signal0 = map(clip(thrustSignal - headingSignal,-1,1)
                    ,-1, 1,-SAFE_DIFFERENTIAL_THRUST, SAFE_DIFFERENTIAL_THRUST);
            signal1 = map(clip(thrustSignal + headingSignal,-1,1)
                    ,-1, 1,-SAFE_DIFFERENTIAL_THRUST, SAFE_DIFFERENTIAL_THRUST);
        }
        else if (containers.thrustType.get() == THRUST_TYPES.VECTORED.getLongValue()) {
            signal0 = map(clip(thrustSignal,-1,1),-1,1,-SAFE_VECTORED_THRUST,SAFE_VECTORED_THRUST);
            signal1 = clip(headingSignal, -1, 1);
        }


        containers.motorCommands.set(0, signal0);
        containers.motorCommands.set(1, signal1);

        //TODO: after the velocity maps are built, use them to treat expected velocities as a sensor
        //TODO: alternatively, gather JSON's from arduino and put that through the map as the faux sensor
        /*
        double[] velocities = velocityMotorMap.Signal_to_VW(signal0,signal1);
        // send intended steady state velocities to the localization filter
        RealMatrix z = MatrixUtils.createRealMatrix(2,1);
        z.setEntry(0,0,velocities[0]);
        z.setEntry(1,0,velocities[1]);
        RealMatrix R = MatrixUtils.createRealMatrix(2,2); // all zeros b/c this is a "perfect" sensor
        t = System.currentTimeMillis();
        Datum datum = new Datum(SENSOR_TYPES.MOTOR,t,z,R);
        datumListener.newDatum(datum);
        */
    }

    void updateFromKnowledgeBase() {
        // remember to subtract device.{.id}.home from the destination so xd is centered about (0,0) like x
        xd = containers.NDV_to_RM(containers.self.device.dest).subtract(containers.NDV_to_RM(containers.self.device.home));

        // update current state
        for (int i = 0; i < stateSize; i++) {
            x.setEntry(i,0,containers.x.get(i));
        }

        Log.w("jjb","xd = " + RMO.realMatrixToString(xd));
        //Log.w("jjb","x = " + RMO.realMatrixToString(x));
    }

    public void newProfile(RealMatrix profile, double sufficientProximity) {
        this.profile = profile;
        containers.executingProfile.set(1);
        simplePIDErrorAccumulator = new double[]{0,0,0}; // set basic PID error accumulators to zero
        t0set = false;
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
