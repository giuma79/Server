package edu.cmu.ri.airboat.server;

import android.util.Log;

import com.madara.KnowledgeBase;
import com.madara.KnowledgeRecord;
import com.madara.containers.DoubleVector;

import org.apache.commons.math.linear.MatrixUtils;
import org.apache.commons.math.linear.RealMatrix;

/**
 * @author jjb
 */
public class BoatMotionController {

    int stateSize;
    RealMatrix x;
    RealMatrix xd;
    KnowledgeBase knowledge;
    DoubleVector x_KB;

    public BoatMotionController(KnowledgeBase knowledge, int stateSize) {
        this.knowledge = knowledge;
        x_KB = new DoubleVector();
        x_KB.setName(this.knowledge,".x");
        this.stateSize = stateSize;
        x_KB.resize(stateSize);
        x = MatrixUtils.createRealMatrix(stateSize,1);
    }

    public void control() {
        updateFromKnowledgeBase();
    }

    void updateFromKnowledgeBase() {
        // update destination
        KnowledgeRecord xd_KR = knowledge.get("device.{id}.dest");
        try {
            double[] xd_array = xd_KR.toDoubleArray();
            xd = MatrixUtils.createColumnRealMatrix(xd_array);
            // update current state
            for (int i = 0; i < stateSize; i++) {
                x.setEntry(i,0,x_KB.get(i));
            }
            Log.w("jjb","xd = " + realMatrixToString(xd));
            Log.w("jjb","x = " + realMatrixToString(x));
        }
        catch (Exception ex) {
        }
    }

    public String realMatrixToString(RealMatrix A) {
        String PString = "\n[";
        for (int i = 0; i < A.getRowDimension(); i++) {
            for (int j = 0; j < A.getColumnDimension()-1; j++) {
                PString = PString + String.format(" %5.3e   ", A.getEntry(i, j));
            }
            PString = PString + String.format("%5.3e\n",A.getEntry(i,A.getColumnDimension()-1));
        }
        PString = PString + "]";
        return PString;
    }

}
