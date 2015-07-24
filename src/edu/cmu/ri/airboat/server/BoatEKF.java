package edu.cmu.ri.airboat.server;

import android.util.Log;

import org.apache.commons.math.linear.MatrixUtils;
import org.apache.commons.math.linear.RealMatrix;
import org.jscience.geography.coordinates.LatLong;

import com.madara.KnowledgeBase;

import javax.measure.unit.NonSI;


/**
 * @author jjb
 */
public class BoatEKF implements DatumListener {

    // Things that update explicitly via Listener
    RealMatrix z; // new measurement to include
    RealMatrix H; // dz/dx associated with z
    RealMatrix R; // covariance associated with z

    public final int stateSize = 7; // state is global except for expected-motor velocities
    // state = [x y theta Vm Wm fx fy]

    double[] initial_x = new double[stateSize];
    RealMatrix x = MatrixUtils.createColumnRealMatrix(initial_x); // state
    RealMatrix P = MatrixUtils.createRealIdentityMatrix(stateSize); // stateCov
    RealMatrix K; // Kalman gain
    RealMatrix QBase = MatrixUtils.createRealIdentityMatrix(stateSize); // growth of uncertainty with time
    RealMatrix G = MatrixUtils.createRealIdentityMatrix(stateSize); // coordinate transformation of uncertainty
    RealMatrix Phi = MatrixUtils.createRealIdentityMatrix(stateSize); // state transition (x_{k+1} = Phi*x_{k})
    RealMatrix Phi_k =  MatrixUtils.createRealIdentityMatrix(stateSize); // (I + F*dt), propagation of uncertainty (P_{k+1} = Phi_k*P_{k}*Phi_k' + GQG')

    Long t; // current time
    final double ROLLBACK_LIMIT = 1.0; // seconds allowed for a rollback before measurements are just abandoned

    KnowledgeBase knowledge;
    LutraMadaraContainers containers;

    public BoatEKF(KnowledgeBase knowledge,LutraMadaraContainers containers) {
        t = java.lang.System.currentTimeMillis();
        this.knowledge = knowledge;
        this.containers = containers;
        containers.x.resize(stateSize);

        // default Q and P (use other constructor to override these)
        QBase = QBase.scalarMultiply(0.01);
        P = P.scalarMultiply(0.1);
        P.setEntry(0,0,1.0); // default GPS covariance is much larger than other parts of state
        P.setEntry(1, 1, 1.0); // default GPS covariance is much larger than other parts of state
    }

    public BoatEKF(KnowledgeBase knowledge, LutraMadaraContainers containers, RealMatrix x, RealMatrix P, RealMatrix QBase) {
        this(knowledge,containers);
        this.x = x;
        this.P = P;
        this.QBase = QBase;
    }

    public void shutdown() {
    }

    public synchronized void updateKnowledgeBase() {
        for (int i = 0; i < stateSize; i++) {
            containers.x.set(i, x.getEntry(i, 0));
        }
    }

    @Override
    public synchronized void newDatum(Datum datum) {

        //String threadID = String.format(" -- thread # %d",Thread.currentThread().getId());
        //Log.w("jjb","received datum z = " + datum.getZ().toString() + threadID);

        if (!(datum.isType(SENSOR_TYPES.GYRO) ||
              datum.isType(SENSOR_TYPES.COMPASS) ||
              datum.isType(SENSOR_TYPES.MOTOR)))      {
            String datum_data = RMO.realMatrixToString(datum.getZ());
            String datum_info = String.format("Received %s, z = %s",datum.typeString(),datum_data);
            Log.w("jjb",datum_info);
        }


        //String timeString = String.format("EKF.t BEFORE = %d",t);
        //Long old_t = t;

        // update z and R
        if (datum.isType(SENSOR_TYPES.GPS)) { // subtract home so localization is centered around (0,0)
            RealMatrix _z = datum.getZ();
            RealMatrix home_RM = containers.NDV_to_RM(containers.self.device.home);
            z = _z.subtract(home_RM.getSubMatrix(0,1,0,0));
        }
        else {
            z = datum.getZ();
        }

        R = datum.getR();

        // warning if datum timestamp is too far away from filter's current time
        if ((datum.getTimestamp().doubleValue() - t.doubleValue())/1000.0 > ROLLBACK_LIMIT) {
            String warning = String.format(
                    "WARNING: %s sensor is more than %f seconds AHEAD of filter",datum.typeString(),ROLLBACK_LIMIT);
            Log.w("jjb",warning);
        }
        else if ((datum.getTimestamp().doubleValue() - t.doubleValue())/1000.0 < -ROLLBACK_LIMIT) {
            String warning = String.format(
                    "WARNING: %s sensor is more than %f seconds BEHIND of filter",datum.typeString(),ROLLBACK_LIMIT);
            Log.w("jjb",warning);
        }


        // first GPS and compass (i.e. positions) datum are put into state directly
        // velocity parts of state are initialized at zero
        if (containers.gpsInitialized.get() == 0) {
            if (datum.isType(SENSOR_TYPES.GPS)) {
                double[] _z = new double[] {z.getEntry(0,0),z.getEntry(1,0),x.getEntry(2,0)};

                for (int i = 0; i < 3; i++) {
                    containers.self.device.location.set(i,_z[i]);
                    containers.self.device.home.set(i,_z[i]);
                    containers.self.device.dest.set(i,_z[i]);
                    containers.self.device.source.set(i,_z[i]);
                }

                x.setEntry(0,0,0);
                x.setEntry(1,0,0);
                containers.x.set(0,0);
                containers.x.set(1,0);
                containers.gpsInitialized.set(1);

                LatLong latLong = containers.LocalXYToLatLong();
                containers.latLong.set(0,latLong.latitudeValue(NonSI.DEGREE_ANGLE));
                containers.latLong.set(1,latLong.longitudeValue(NonSI.DEGREE_ANGLE));

                if (containers.compassInitialized.get() == 1) {
                    containers.localized.set(1);
                }

                Log.w("jjb", "GPS is now initialized");
                return;
            }
        }

        if (containers.compassInitialized.get() == 0) { // TODO: is there a way to have the compass update even if you don't have a GPS lock?
            if (datum.isType(SENSOR_TYPES.COMPASS)) {
                double[] x_array = containers.NDV_to_DA(containers.self.device.location);
                double[] _z = new double[] {x_array[0],x_array[1],z.getEntry(0,0)};

                for (int i = 0; i < 3; i++) {
                    containers.self.device.location.set(i,_z[i]);
                    containers.self.device.home.set(i,_z[i]);
                    containers.self.device.dest.set(i,_z[i]);
                    containers.self.device.source.set(i,_z[i]);
                }

                x.setEntry(2,0,z.getEntry(0,0));
                containers.x.set(2,z.getEntry(0,0));
                containers.compassInitialized.set(1);
                if (containers.gpsInitialized.get() == 1) {
                    containers.localized.set(1);
                }

                Log.w("jjb", "Compass is now initialized");
                return;
            }
        }

        if (containers.localized.get() == 0) {
            timeStep();
            return;
        }

        predict();

        //timeString = timeString + String.format(",  EKF.t AFTER = %d",t);
        //timeString = timeString + String.format(",  datum.t = %d",datum.getTimestamp());
        //timeString = timeString + String.format(",  EKF.dt = %d,  datum LAG = %d",t-old_t,t-datum.getTimestamp());
        //Log.w("jjb",timeString);

        // given datum.type, construct H
        setH(datum);
        sensorUpdate();

    }



    private synchronized void setH(Datum datum) {
        double s = Math.sin(x.getEntry(2, 0));
        double c = Math.cos(x.getEntry(2, 0));
        double v = x.getEntry(3, 0);
        double fx = x.getEntry(5, 0);
        double fy = x.getEntry(6, 0);
        //double ft = x.getEntry(7,0);

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
            //newH.setEntry(0,7,-1.0);
        }
        else if (datum.getType() == SENSOR_TYPES.IMU) {
            newH.setEntry(0,2,fx*s-fy*c);
            newH.setEntry(0,3,1.0);
            newH.setEntry(0,5,-c);
            newH.setEntry(0,6,-s);
            newH.setEntry(1,2,fx*c+fy*s);
            newH.setEntry(1,5,s);
            newH.setEntry(1,6,c);
        }
        else if (datum.getType() == SENSOR_TYPES.MOTOR) {
            newH.setEntry(0,3,1.0);
            //newH.setEntry(1,4,1.0);
        }
        else if (datum.getType() == SENSOR_TYPES.DGPS) {
            newH.setEntry(0,2,-v*s);
            newH.setEntry(0,3,c);
            newH.setEntry(0,5,-1.0);
            newH.setEntry(1,2,v*c);
            newH.setEntry(1,3,s);
            newH.setEntry(1,6,-1.0);
        }

        this.H = newH;
    }
    public synchronized void sensorUpdate() {
        // compute kalman gain = PH'inv(HPH'+R)
        RealMatrix Ktemp = H.multiply(P).multiply(H.transpose()).add(R);
        Ktemp = RMO.inverse(Ktemp);
        Ktemp = P.multiply(H.transpose()).multiply(Ktemp);
        K = Ktemp.copy();

        //Log.w("jjb","P = " + P.toString());
        //Log.w("jjb","H = " + H.toString());
        //Log.w("jjb","K = " + K.toString());

        // compute innovation (dz), remember in EKF, dz = z - h(x), not z - Hx
        // z - Hx will work for simple measurements, where H is just ones
        // So z - Hx will work for GPS, compass, and gyro. Not sure about IMU just yet.
        RealMatrix dz = MatrixUtils.createRealMatrix(z.getRowDimension(), 1);
        dz = z.subtract(H.multiply(x));

        // compute innovation covariance S = HPH' + R
        RealMatrix S = H.multiply(P).multiply(H.transpose());

        // check "validation gate" - calculate Mahalanobis distance d = sqrt(dz'*inv(S)*dz)
        RealMatrix dtemp = dz.transpose().multiply(RMO.inverse(S)).multiply(dz);
        double d = Math.sqrt(dtemp.getEntry(0,0));

        // if Mahalanobis distance is below threshold, update state estimate, x_{k+1} = x_{k} + K*(dz)
        // and update state covariance P_{k+1} = (I - KH)P
        //String a = String.format("Mahalanobis distance = %f",d);
        //Log.w("jjb", a);
        //if (d > 3.0) {
        //    Log.w("jjb","WARNING, Mahalanobis distance is > 3. Measurement will be ignored.");
        //}
        //else {
            x = x.add(K.multiply(dz));
            P = (MatrixUtils.createRealIdentityMatrix(P.getRowDimension()).subtract(K.multiply(H))).multiply(P);
        //}

        updateKnowledgeBase();

        //String PString = RMO.realMatrixToString(P);
        //Log.w("jjb","P = " + PString);
    }

    public synchronized void predict() {

        //String threadID = String.format(" -- thread # %d",Thread.currentThread().getId());
        //Log.w("jjb","BoatEKF.predict()" + threadID);

        double dt = timeStep();
        String a = String.format("dt = %f",dt);
        Log.d("jjb",a);

        // update Q with dt
        RealMatrix Q = QBase.scalarMultiply(dt);

        double s = Math.sin(x.getEntry(2, 0));
        double c = Math.cos(x.getEntry(2, 0));
        double v = x.getEntry(3,0);

        // Update Phi and Phi_k with current state and dt
        Phi.setEntry(0,3,dt*c);
        Phi.setEntry(0,5,-dt);
        Phi.setEntry(1,3,dt*s);
        Phi.setEntry(1,6,-dt);
        Phi.setEntry(2,4,dt);
        //Phi.setEntry(2,7,-dt);

        Phi_k.setEntry(0,2,-dt*v*s);
        Phi_k.setEntry(0,3,dt*c);
        Phi_k.setEntry(0,5,-dt);
        Phi_k.setEntry(1,2,dt*v*c);
        Phi_k.setEntry(1,3,dt*s);
        Phi_k.setEntry(1,6,-dt);
        Phi_k.setEntry(2,4,dt);
        //Phi_k.setEntry(2,7,-dt);

        // Update G with current state
        //G.setEntry(0,0,c);
        //G.setEntry(0,1,-s);
        //G.setEntry(1,0,s);
        //G.setEntry(1,1,c);

        // Update state and state covariance
        x = Phi.multiply(x);
        P = (Phi_k.multiply(P).multiply(Phi_k.transpose())).add(G.multiply(Q).multiply(G.transpose()));

        updateKnowledgeBase();
    }


    public synchronized double timeStep() {
        // Update dt
        Long old_t = t;
        t = System.currentTimeMillis();
        return (t.doubleValue() - old_t.doubleValue())/1000.0;
    }

    /*
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
    */

}
