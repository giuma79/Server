package edu.cmu.ri.airboat.server;

import com.madara.KnowledgeRecord;

import org.apache.commons.math.linear.LUDecompositionImpl;
import org.apache.commons.math.linear.MatrixUtils;
import org.apache.commons.math.linear.RealMatrix;

/**
 * @author jjb
 */
public class RMO {

    public static double distance(RealMatrix x1, RealMatrix x2) {
        // assumes COLUMNS ONLY
        RealMatrix difference = MatrixUtils.createRealMatrix(x1.getRowDimension(), 1);
        difference = x1.subtract(x2);
        double result = 0;
        for (int i = 0; i < x1.getRowDimension(); i++) {
            result += difference.getEntry(i,0)*difference.getEntry(i, 0);
        }
        return Math.sqrt(result);
    }

    public static double dot(RealMatrix x1, RealMatrix x2) {
        // assumes COLUMNS ONLY
        double result = 0;
        for (int i = 0; i < x1.getRowDimension(); i++) {
            result += x1.getEntry(i,0)*x2.getEntry(i,0);
        }
        return result;
    }

    public static double norm2(RealMatrix x) {
        // assumes COLUMNS ONLY
        double result = 0;
        result = dot(x,x);
        return Math.sqrt(result);
    }

    public static RealMatrix linspace(double start, double end, int count) {
        RealMatrix result = MatrixUtils.createRealMatrix(count,1);
        for (int i = 0; i < count; i++) {
            result.setEntry(i,0,start + (double)i/(count-1)*(end-start));
        }
        return result;
    }

    public static String realMatrixToString(RealMatrix A) {
        String PString = "\n[";
        for (int i = 0; i < A.getRowDimension(); i++) {
            for (int j = 0; j < A.getColumnDimension()-1; j++) {
                PString = PString + String.format(" %8.6e   ", A.getEntry(i, j));
            }
            PString = PString + String.format("%8.6e\n", A.getEntry(i, A.getColumnDimension() - 1));
        }
        PString = PString + "]";
        return PString;
    }

    public static RealMatrix elemWiseMult(RealMatrix a, RealMatrix b) {
        RealMatrix result = MatrixUtils.createRealMatrix(a.getRowDimension(),a.getColumnDimension());
        for (int i = 0; i < a.getRowDimension(); i++) {
            for (int j = 0; j < a.getColumnDimension(); j++) {
                result.setEntry(i,j,a.getEntry(i,j)*b.getEntry(i,j));
            }
        }
        return result;
    }

    public static RealMatrix elemWisePower(RealMatrix a, double p) {
        RealMatrix result = MatrixUtils.createRealMatrix(a.getRowDimension(),a.getColumnDimension());
        for (int i = 0; i < a.getRowDimension(); i++) {
            for (int j = 0; j < a.getColumnDimension(); j++) {
                result.setEntry(i,j,Math.pow(a.getEntry(i,j),p));
            }
        }
        return result;
    }

    public static RealMatrix sumOverRows(RealMatrix a) {
        RealMatrix result = MatrixUtils.createRealMatrix(1, a.getColumnDimension());
        for (int i = 0; i < a.getRowDimension(); i++) {
            for (int j = 0; j < a.getColumnDimension(); j++) {
                result.setEntry(0,j,result.getEntry(0,j)+a.getEntry(i,j));
            }
        }
        return result;
    }

    public static RealMatrix meanOverRows(RealMatrix a) {
        RealMatrix result = sumOverRows(a);
        for (int j = 0; j < a.getColumnDimension(); j++) {
            result.setEntry(0,j,result.getEntry(0,j)/(double)a.getRowDimension());
        }
        return result;
    }

    public static RealMatrix inverse(RealMatrix a) {
        RealMatrix result;
        LUDecompositionImpl decomp = new LUDecompositionImpl(a);
        result = decomp.getSolver().getInverse();
        return result;
    }

    public static double interpolate1D(RealMatrix XY, double x, int yCol) {
        // assumes first column is X data, second column is Y data, and X is in ascending sorted order
        for (int i = 0; i < XY.getRowDimension(); i++) {
            if (x >= XY.getEntry(i,0)) {
                if (x == XY.getEntry(0,0)) { // edge case, equal to the FIRST row in XY
                    return XY.getEntry(0,yCol);
                }
                if (x == XY.getEntry(XY.getRowDimension(),0)) { // edge case, equal to the LAST row in XY
                    return XY.getEntry(XY.getRowDimension(),yCol);
                }
                return XY.getEntry(i,yCol) + (x-XY.getEntry(i,0))/(XY.getEntry(i+1,0)-XY.getEntry(i,0))*(XY.getEntry(i+1,yCol)-XY.getEntry(i,yCol));
            }
        }
        throw new ArrayIndexOutOfBoundsException();
    }

    public static double[][] concat2D_double(double[][] a1, double[][] a2) {
        double[][] result = new double[a1.length + a2.length][];
        System.arraycopy(a1, 0, result, 0, a1.length);
        System.arraycopy(a2, 0, result, a1.length, a2.length);
        return result;
    }
}
