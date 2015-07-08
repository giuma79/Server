package edu.cmu.ri.airboat.server;

import org.apache.commons.math.linear.MatrixUtils;
import org.apache.commons.math.linear.RealMatrix;

/**
 * @author jjb
 */
public class VelocityMotorMap {
    LutraMadaraContainers containers;
    VelocityMotorMap(LutraMadaraContainers containers) {
        this.containers = containers;
    }

    double v_to_Signal(double v) { // if you request forward velocity only (motors are equal)
        double result = 0;
        if (containers.thrustType.get() == THRUST_TYPES.DIFFERENTIAL.getLongValue()) {
            result = RMO.interpolate1D(diff_VtoSignalMapMatrix,v,1);
        }
        else if (containers.thrustType.get() == THRUST_TYPES.VECTORED.getLongValue()) {
            result = RMO.interpolate1D(vect_VtoSignalMapMatrix,v,1);
        }
        return result;
    }

    double motorSignalClip(double M) {
        double result = M;
        if (Math.abs(M) > 1.0) {
            result = Math.signum(M);
        }
        return result;
    }

    double[] Signal_to_VW(double signal0, double signal1) {
        double[] result = new double[2];
        if (containers.thrustType.get() == THRUST_TYPES.DIFFERENTIAL.getLongValue()) {
            result[0] = RMO.interpolate1D(diff_SignaltoVMapMatrix,signal0+signal1,1);
            result[1] = RMO.interpolate1D(diff_SignaltoWMapMatrix,signal0-signal1,1);
        }
        else if (containers.thrustType.get() == THRUST_TYPES.VECTORED.getLongValue()) {
            result[0] = RMO.interpolate1D(vect_SignaltoVMapMatrix,signal0,1);
            result[1] = RMO.interpolate1D(vect_SignaltoWMapMatrix,signal1,1);
        }
        return result;
    }

    //TODO: build these maps via testing
    // DIFFERENTIAL
    // forward velocity --> two motors, separate but equal signals (so M0+M1 is [-2,2], M0 = M1)
    double[][] diff_VtoSignalMapArray = new double[][]{ {0,0}, // [v,M0+M1]
                                                        {0,0} };
    double[][] diff_SignaltoVMapArray = new double[][]{ {0,0}, // [M0+M1,v]
                                                        {0,0} };
    RealMatrix diff_VtoSignalMapMatrix = MatrixUtils.createRealMatrix(diff_VtoSignalMapArray);
    RealMatrix diff_SignaltoVMapMatrix = MatrixUtils.createRealMatrix(diff_SignaltoVMapArray);

    // angular velocity --> two motors with equal magnitude, opposite signs (so M0+M1 = 0, M0 = -M1)
    double[][] diff_WtoSignalMapArray = new double[][] { {0,0}, // [w,M0-M1]
                                                         {0,0} };
    double[][] diff_SignaltoWMapArray = new double[][] { {0,0}, // [M0-M1,w]
                                                         {0,0} };
    RealMatrix diff_WtoSignalMapMatrix = MatrixUtils.createRealMatrix(diff_WtoSignalMapArray);
    RealMatrix diff_SignaltoWMapMatrix = MatrixUtils.createRealMatrix(diff_SignaltoWMapArray);


    // VECTORED
    // forward velocity --> one motor with signal between 0 and 1
    double[][] vect_VtoSignalMapArray = new double[][]{ {0,0}, // [v,M0]
                                                         {0,0} };
    double[][] vect_SignaltoVMapArray = new double[][]{ {0,0}, // [M0, v]
                                                        {0,0} };
    RealMatrix vect_VtoSignalMapMatrix = MatrixUtils.createRealMatrix(vect_VtoSignalMapArray);
    RealMatrix vect_SignaltoVMapMatrix = MatrixUtils.createRealMatrix(vect_SignaltoVMapArray);
    // angular velocity --> one angle (treat it like a motor, the greater the angle, the greater the thrust)
    double[][] vect_WtoSignalMapArray = new double[][]{ {0,0}, // [w,M1]
            {0,0} };
    double[][] vect_SignaltoWMapArray = new double[][]{ {0,0}, // [M1, w]
            {0,0} };
    RealMatrix vect_WtoSignalMapMatrix = MatrixUtils.createRealMatrix(vect_WtoSignalMapArray);
    RealMatrix vect_SignaltoWMapMatrix = MatrixUtils.createRealMatrix(vect_SignaltoWMapArray);

    /*
    double[] v_to_M(double v, double w) { // if you request forward and rotational velocity
        double[] result = new double[2];
        if (containers.thrustType.get() == THRUST_TYPES.DIFFERENTIAL.getLongValue()) {
            // returns required motor actions --> [m0, m1]
            double M0plusM1 = RMO.interpolate1D(differentialForwardMapMatrix,v);
            double M0minusM1 = RMO.interpolate1D(differentialRotMapMatrix,w);
            double M0 = 0.5*M0plusM1+M0minusM1;
            double M1 = M0plusM1 - M0;
            double[] clippedM = motorSignalClip(M0,M1);
            result[0] = clippedM[0];
            result[1] = clippedM[1];
        }
        if (containers.thrustType.get() == THRUST_TYPES.VECTORED.getLongValue()) {
            // returns required motor action and fan angle --> [m0, mth]

        }
        return result;
    }

    double[] motorSignalClip(double M0, double M1) {
        double[] result = new double[]{M0,M1};
        while (Math.abs(M0) > 1.0) {
            M1 = M1 - Math.signum(M0)*(Math.abs(M0)-1);
            M0 = M0 - Math.signum(M0)*(Math.abs(M0)-1);
        }
        while (Math.abs(M1) > 1.0) {
            M0 = M0 - Math.signum(M1)*(Math.abs(M1)-1);
            M1 = M1 - Math.signum(M1)*(Math.abs(M1)-1);
        }
        if (Math.abs(M0) > 1) { // last ditch effort to clip weird combos
            M0 = Math.signum(M0);
        }
        if (Math.abs(M1) > 1) { // last ditch effort to clip weird combos
            M1 = Math.signum(M1);
        }
        result[0] = M0;
        result[1] = M1;
        return result;
    }
    */



}
