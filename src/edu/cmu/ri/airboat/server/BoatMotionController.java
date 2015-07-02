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
    KnowledgeBase knowledge;
    Long t;
    boolean executingProfile;
    LutraMadaraContainers containers;

    public BoatMotionController(KnowledgeBase knowledge, int stateSize,LutraMadaraContainers containers) {
        this.knowledge = knowledge;
        this.containers = containers;
        this.stateSize = stateSize;
        xError = MatrixUtils.createRealMatrix(3,1);
        x = MatrixUtils.createRealMatrix(stateSize,1);
        xd = MatrixUtils.createRealMatrix(3,1);
    }

    public void control() {
        updateFromKnowledgeBase();

        // current position error
        xError = xd.subtract(x.getSubMatrix(0, 2, 0, 0));
        containers.distToDest.set(RMO.norm2(x.getSubMatrix(0, 1, 0, 0), xd.getSubMatrix(0, 1, 0, 0)));
        if (containers.distToDest.get() < containers.sufficientProximity.get()) {
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
        //KnowledgeRecord home = containers.self.device.home.toRecord();
        //KnowledgeRecord xd_KR = containers.self.device.dest.toRecord();
        //double[] xd_array = xd_KR.toDoubleArray();
        //double[] home_array = home.toDoubleArray();
        //xd = MatrixUtils.createColumnRealMatrix(xd_array);
        //xd = xd.subtract(MatrixUtils.createColumnRealMatrix(home_array));
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
        executingProfile = true;
    }

    public void shutdown() {
    }



}
