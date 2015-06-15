package edu.cmu.ri.airboat.server;

import org.apache.commons.math.linear.RealMatrix;

/**
 * @author jjb
 */
public class BoatEKF implements DatumListener {

    // Things that update explicitly via Listener
    RealMatrix u; // current control action --> motion controller must add Async_EKF as a listener, call controlEvent()
    RealMatrix z; // new measurement to include --> AirboatService must add Async_EKF as a listener, call measurementEvent()
    RealMatrix H; // dz/dx associated with z --> ditto
    RealMatrix R; // covariance associated with z --> ditto

    RealMatrix x; // state
    RealMatrix P; // stateCov
    RealMatrix K; // Kalman gain
    RealMatrix F; // d_xdot/d_x
    RealMatrix Q; // growth of uncertainty with time
    RealMatrix G; // coordinate transformation of uncertainty
    RealMatrix Phi; // state transition (x_{k+1} = Phi*x_{k})
    RealMatrix Phi_k; // (I + F*dt), propagation of uncertainty (P_{k+1} = Phi_k*P_{k}*Phi_k' + GQG')

    Long t; // current time
    double ROLLBACK_LIMIT = 1.0; // seconds allowed for a rollback before measurements are just abandoned

    public EKF() {
        t = java.lang.System.currentTimeMillis();
    }

    public EKF(RealMatrix x, RealMatrix P, RealMatrix Q) {
        this();
        this.x = x;
        this.P = P;
        this.Q = Q;
    }

    @Override
    public void newDatum(Datum datum) {
        // update z and R
        // given datum.type, construct H

        update();
    }

    public void predict() {
        double dt = timeStep();

        // Update Phi and Phi_k with current state and dt

        // Update G with current state

        // Update state and state covariance
        x = Phi.multiply(x);
        P = (Phi_k.multiply(P).multiply(Phi_k.transpose())).add(G.multiply(Q).multiply(G.transpose()));
    }
    public void update() {
        predict();

        // compute kalman gain
        // compute innovation (dz), remember in EKF, dz = z - h(x), not z - Hx
        // compute innovation covariance S = HPH' + R

        // check "validation gate" - calculate Mahalanobis distance d = sqrt(dz'*inv(S)*dz)

        // if Mahalanobis distance is below threshold, update state estimate, x_{k+1} = x_{k} + K*(dz)
        // and update state covariance P_{k+1} = (I - KH)P

    }

    public double timeStep() {
        // Update dt
        Long old_t = t;
        t = java.lang.System.currentTimeMillis();
        return (t.doubleValue() - old_t.doubleValue())*1000.0;
    }

    public boolean rollBack(Long old_t) {
        // roll back the state of the filter
        double secondsBack = (old_t.doubleValue() - t.doubleValue()) * 1000.0;

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
