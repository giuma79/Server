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
    double[] v_to_M(double v, double w) {
        double[] result = new double[2];
        if (containers.thrustType.get() == THRUST_TYPES.DIFFERENTIAL.getLongValue()) {
            // returns required motor actions --> [m0, m1]
            double M0plusM1 = RMO.interpolate1D(differentialForwardMapMatrix,v);
            double M0minusM1 = RMO.interpolate1D(differentialRotMapMatrix,w);
            double M0 = 0.5*M0plusM1+M0minusM1;
            double M1 = M0plusM1 - M0;
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
        }
        if (containers.thrustType.get() == THRUST_TYPES.VECTORED.getLongValue()) {
            // returns required motor action and fan angle --> [m0, mth]

        }
        return result;
    }

    // DIFFERENTIAL
        // forward velocity --> two motors
        // angular velocity --> two motors
        double[][] differentialForwardMapArray = new double[][]{ {0,0}, // [v,M0+M1]
                                                                 {0,0} };
        RealMatrix differentialForwardMapMatrix = MatrixUtils.createRealMatrix(differentialForwardMapArray);

        double[][] differentialRotMapArray = new double[][] { {0,0}, // [w,M0-M1]
                                                              {0,0} };
        RealMatrix differentialRotMapMatrix = MatrixUtils.createRealMatrix(differentialRotMapArray);

    // VECTORED
        // forward velocity --> one motor, one angle
        // angular velocity --> one motor, one angle



}
