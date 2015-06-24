package edu.cmu.ri.airboat.server;

import android.util.Log;

import org.apache.commons.math.linear.LUDecompositionImpl;
import org.apache.commons.math.linear.MatrixUtils;
import org.apache.commons.math.linear.RealMatrix;

import com.madara.KnowledgeBase;
import com.madara.containers.DoubleVector;

/**
 * @author jjb
 */
public class BoatEKF implements DatumListener {

    // Things that update explicitly via Listener
    RealMatrix z; // new measurement to include
    RealMatrix H; // dz/dx associated with z
    RealMatrix R; // covariance associated with z

    int stateSize = 8; // [x,y,theta,v,omega,cx,cy,ct]
    double[] initial_x = new double[stateSize];
    RealMatrix x = MatrixUtils.createColumnRealMatrix(initial_x); // state
    RealMatrix P = MatrixUtils.createRealIdentityMatrix(stateSize); // stateCov
    RealMatrix K; // Kalman gain
    //RealMatrix Q = MatrixUtils.createRealIdentityMatrix(stateSize); // growth of uncertainty with time
    RealMatrix QBase = MatrixUtils.createRealIdentityMatrix(stateSize); // growth of uncertainty with time
    RealMatrix G = MatrixUtils.createRealIdentityMatrix(stateSize); // coordinate transformation of uncertainty
    RealMatrix Phi = MatrixUtils.createRealIdentityMatrix(stateSize); // state transition (x_{k+1} = Phi*x_{k})
    RealMatrix Phi_k =  MatrixUtils.createRealIdentityMatrix(stateSize); // (I + F*dt), propagation of uncertainty (P_{k+1} = Phi_k*P_{k}*Phi_k' + GQG')

    Long t; // current time
    final double ROLLBACK_LIMIT = 1.0; // seconds allowed for a rollback before measurements are just abandoned

    boolean isGPSInitialized = false;
    boolean isCompassInitialized = false;

    KnowledgeBase knowledge;
    DoubleVector x_KB;

    public BoatEKF(KnowledgeBase knowledge) {
        t = java.lang.System.currentTimeMillis();
        this.knowledge = knowledge;
        x_KB = new DoubleVector();
        x_KB.setName(knowledge, ".x");

        // default Q and P (use other constructor to override these)
        QBase.scalarMultiply(0.01);
        P.scalarMultiply(0.1);
        P.setEntry(0,0,1.0); // default GPS covariance is much larger than other parts of state
        P.setEntry(1,1,1.0); // default GPS covariance is much larger than other parts of state

        Log.w("jjb", "BoatEKF has been constructed");
    }

    public BoatEKF(KnowledgeBase knowledge, RealMatrix x, RealMatrix P, RealMatrix QBase) {
        this(knowledge);
        this.x = x;
        this.P = P;
        this.QBase = QBase;
    }

    public void stop() {
        x_KB.free();
    }

    public synchronized void updateKnowledgeBase() {
        //Log.d("jjb","---------------------------------");
        for (int i = 0; i < stateSize; i++) {
            String message = String.format("x(%d) = %f",i,x.getEntry(i,0));
            //Log.d("jjb",message);
            x_KB.set(i, x.getEntry(i,0));
        }
        //String message = String.format("x(%d) = %f",2,x.getEntry(2,0));
        //Log.d("jjb",message);
        //Log.d("jjb","---------------------------------");
    }

    @Override
    public synchronized void newDatum(Datum datum) {

        String threadID = String.format(" -- thread # %d",Thread.currentThread().getId());
        Log.w("jjb","received datum z = " + datum.getZ().toString() + threadID);

        //String timeString = String.format("EKF.t BEFORE = %d",t);
        //Long old_t = t;

        // update z and R
        z = datum.getZ();
        R = datum.getR();
        predict();

        //timeString = timeString + String.format(",  EKF.t AFTER = %d",t);
        //timeString = timeString + String.format(",  datum.t = %d",datum.getTimestamp());
        //timeString = timeString + String.format(",  EKF.dt = %d,  datum LAG = %d",t-old_t,t-datum.getTimestamp());
        //Log.w("jjb",timeString);


        // warning if datum timestamp is too far away from filter's current time
        if ((datum.getTimestamp().doubleValue() - t.doubleValue())*1000.0 > ROLLBACK_LIMIT) {
            String warning = String.format(
                    "WARNING: %s sensor is more than %f seconds AHEAD filter",datum.typeString(),ROLLBACK_LIMIT);
            //System.out.println(warning);
            Log.w("datum timestamp",warning);
        }
        else if ((datum.getTimestamp().doubleValue() - t.doubleValue())*1000.0 < -ROLLBACK_LIMIT) {
            String warning = String.format(
                    "WARNING: %s sensor is more than %f seconds BEHIND filter",datum.typeString(),ROLLBACK_LIMIT);
            //System.out.println(warning);
            Log.w("datum timestamp",warning);
        }


        // first GPS and compass (i.e. positions) datum are put into state directly
        // velocity parts of state are initialized at zero
        if (!isGPSInitialized) {
            if (datum.isType(SENSOR_TYPES.GPS)) {
                x.setEntry(0,0,z.getEntry(0,0));
                x.setEntry(1,0,z.getEntry(1,0));
                isGPSInitialized = true;


                Log.w("jjb", "GPS is now initialized");

                return;
            }
        }

        if (!isCompassInitialized) {
            if (datum.isType(SENSOR_TYPES.COMPASS)) {
                x.setEntry(2,0,z.getEntry(0,0));
                isCompassInitialized = true;

                Log.w("jjb", "Compass is now initialized");

                return;
            }
        }

        // given datum.type, construct H
        setH(datum);
        sensorUpdate();

        String PString = "[";
        for (int i = 0; i < stateSize; i++) {
            for (int j = 0; j < stateSize-1; j++) {
                PString = PString + String.format("%4.3e   ", P.getEntry(i, j));
            }
            PString = PString + String.format("%4.3e\n",P.getEntry(i,stateSize-1));
        }
        PString = PString + "]";
        Log.d("jjb","P = " + PString);
    }

    private synchronized void setH(Datum datum) {
        RealMatrix newH = MatrixUtils.createRealMatrix(z.getRowDimension(),stateSize);
        if (datum.getType() == SENSOR_TYPES.GPS) {
            newH.setEntry(0,0,1.0);
            newH.setEntry(1,1,1.0);
        }
        else if (datum.getType() ==  SENSOR_TYPES.COMPASS) {
            newH.setEntry(0,2,1.0);
        }
        else if (datum.getType() == SENSOR_TYPES.GYRO) {
            newH.setEntry(0,4,1.0);
        }
        else if (datum.getType() == SENSOR_TYPES.IMU) {

        }

        this.H = newH;
    }
    public synchronized void sensorUpdate() {
        // compute kalman gain = PH'inv(HPH'+R)
        RealMatrix Ktemp = H.multiply(P.multiply(H.transpose())).add(R);
        LUDecompositionImpl decomp = new LUDecompositionImpl(Ktemp);
        Ktemp = decomp.getSolver().getInverse();
        Ktemp = (P.multiply(H.transpose())).multiply(Ktemp);
        K = Ktemp.copy();

        //Log.w("jjb","P = " + P.toString());
        //Log.w("jjb","H = " + H.toString());
        //Log.w("jjb","K = " + K.toString());

        // compute innovation (dz), remember in EKF, dz = z - h(x), not z - Hx
        // z - Hx will work for simple measurements, where H is just ones
        // So z - Hx will work for GPS, compass, and gyro. Not sure about IMU just yet.
        RealMatrix dz = MatrixUtils.createRealMatrix(z.getRowDimension(),1);
        dz = z.subtract(H.multiply(x));

        // compute innovation covariance S = HPH' + R
        RealMatrix S = H.multiply(P.multiply(H.transpose()));

        // check "validation gate" - calculate Mahalanobis distance d = sqrt(dz'*inv(S)*dz)
        decomp = new LUDecompositionImpl(S);
        RealMatrix dtemp = ((dz.transpose()).multiply(decomp.getSolver().getInverse())).multiply(dz);
        double d = dtemp.getEntry(0,0);

        // if Mahalanobis distance is below threshold, update state estimate, x_{k+1} = x_{k} + K*(dz)
        // and update state covariance P_{k+1} = (I - KH)P
        String a = String.format("Mahalanobis distance = %f",d);
        Log.w("jjb", a);
        if (d > 3) {
            Log.w("jjb","WARNING, Mahalanobis distance is > 3. Measurement will be ignored.");
        }
        else {
            x = x.add(K.multiply(dz));
            P = ((MatrixUtils.createRealIdentityMatrix(P.getRowDimension())).subtract(K.multiply(H))).multiply(P);
        }

        updateKnowledgeBase();
    }

    public synchronized void predict() {

        //String threadID = String.format(" -- thread # %d",Thread.currentThread().getId());
        //Log.w("jjb","BoatEKF.predict()" + threadID);

        double dt = timeStep();
        String a = String.format("dt = %f",dt);
        Log.d("jjb",a);

        // update Q with dt
        RealMatrix Q = QBase.scalarMultiply(dt);

        double s = Math.cos(x.getEntry(2, 0));
        double c = Math.sin(x.getEntry(2, 0));
        double v = x.getEntry(3,0);

        // Update Phi and Phi_k with current state and dt
        Phi.setEntry(0,3,dt*c);
        Phi.setEntry(0,5,dt);
        Phi.setEntry(1,3,dt*s);
        Phi.setEntry(1,6,dt);
        Phi.setEntry(2,3,dt);
        Phi.setEntry(2,7,dt);

        Phi_k.setEntry(0,2,-dt*v*s);
        Phi_k.setEntry(0,3,dt*c);
        Phi_k.setEntry(0,5,dt);
        Phi_k.setEntry(1,2,dt*v*c);
        Phi_k.setEntry(1,3,dt*s);
        Phi_k.setEntry(1,6,dt);
        Phi_k.setEntry(2,4,dt);
        Phi_k.setEntry(2,7,dt);

        // Update G with current state
        G.setEntry(0,0,c);
        G.setEntry(0,1,-s);
        G.setEntry(1,0,s);
        G.setEntry(1,1,c);

        // Update state and state covariance
        x = Phi.multiply(x);
        P = (Phi_k.multiply(P).multiply(Phi_k.transpose())).add(G.multiply(Q).multiply(G.transpose()));

        updateKnowledgeBase();
    }


    public synchronized double timeStep() {
        // Update dt
        Long old_t = t;
        t = java.lang.System.currentTimeMillis();
        return (t.doubleValue() - old_t.doubleValue())/1000.0;
    }

    public synchronized boolean rollBack(Long old_t) {
        // roll back the state of the filter
        double secondsBack = (old_t.doubleValue() - t.doubleValue()) / 1000.0;

        // return true if roll back occurred, return false if roll back is too far
        if (secondsBack < ROLLBACK_LIMIT) {
            // do stuff

            return true;
        }
        else {
            return false;
        }
    }

}
