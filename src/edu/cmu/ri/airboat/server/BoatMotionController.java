package edu.cmu.ri.airboat.server;

import android.util.Log;

import com.madara.KnowledgeBase;
import com.madara.KnowledgeRecord;
import com.madara.containers.Double;
import com.madara.containers.DoubleVector;

import org.apache.commons.math.linear.MatrixUtils;
import org.apache.commons.math.linear.RealMatrix;

/**
 * @author jjb
 */
public class BoatMotionController implements VelocityProfileListener {

    int stateSize;
    RealMatrix x;
    RealMatrix xd;
    RealMatrix profile;
    KnowledgeBase knowledge;
    DoubleVector x_KB;
    Long t;
    boolean executingProfile;
    Double distance;
    Double sufficientProximity;

    RealMatrix xError;


    public BoatMotionController(KnowledgeBase knowledge, int stateSize) {
        this.knowledge = knowledge;
        x_KB = new DoubleVector();
        x_KB.setName(this.knowledge,".x");
        this.stateSize = stateSize;
        x_KB.resize(stateSize);
        xError = MatrixUtils.createRealMatrix(3,1);
        x = MatrixUtils.createRealMatrix(stateSize,1);
        distance = new Double();
        distance.setName(knowledge,".distToDest");
        sufficientProximity = new Double();
        sufficientProximity.setName(knowledge,".sufficientProximity");
    }

    public void control() {
        updateFromKnowledgeBase();

        // current position error
        xError = xd.subtract(x.getSubMatrix(0, 2, 0, 0));
        distance.set(RMO.norm2(x.getSubMatrix(0, 1, 0, 0), xd.getSubMatrix(0, 1, 0, 0)));
        if (distance.get() < sufficientProximity.get()) {
            executingProfile = false;
        }

        //String distanceString = String.format("Distance from x to xd = %5.3e",distance);
        //Log.w("jjb",distanceString);

        // determine which controller to use, simple PID or P-PI pos./vel. cascade
        if (executingProfile) {
            PPICascade(xError);
        }
        else {
            simplePID(xError);
        }

        motorCommands();
    }

    void simplePID(RealMatrix xError) {
        // Operate on x,y, and theta concurrently.
        // Error in theta scales the forward velocity lower
        // This forces the controller to prioritize corrections in heading
    }

    void PPICascade(RealMatrix xError) {
        // Operate in two phases
        // If the theta error is above some threshold, focus purely on that, ignoring the velocity profile
        // If the theta error is below that threshold, execute the P-PI cascade on the velocity profile
        // As long as the theta error remains below that threshold, you assume the controller is solving
        //   a 1-D problem, modulating the velocity independently of a simple PID that is correcting theta error

    }

    void motorCommands() {
    }

    void updateFromKnowledgeBase() {
        // remember to subtract device.{.id}.home from the destination so xd is centered about (0,0) like x
        KnowledgeRecord home = knowledge.get("device."+ knowledge.get(".id") +".home");
        KnowledgeRecord xd_KR = knowledge.get("device."+ knowledge.get(".id") +".dest");
        double[] xd_array = xd_KR.toDoubleArray();
        double[] home_array = home.toDoubleArray();
        xd = MatrixUtils.createColumnRealMatrix(xd_array);
        xd = xd.subtract(MatrixUtils.createColumnRealMatrix(home_array));

        // update current state
        for (int i = 0; i < stateSize; i++) {
            x.setEntry(i,0,x_KB.get(i));
        }

        Log.w("jjb","xd = " + RMO.realMatrixToString(xd));
        Log.w("jjb","x = " + RMO.realMatrixToString(x));
    }

    public void newProfile(RealMatrix profile, double sufficientProximity) {
        this.profile = profile;
        executingProfile = true;
    }

    public void shutdown() {
        x_KB.free();
        distance.free();
        sufficientProximity.free();
    }



}
