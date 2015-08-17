package edu.cmu.ri.airboat.server;

import com.gams.algorithms.BaseAlgorithm;
import com.gams.algorithms.DebuggerAlgorithm;
import com.gams.controllers.BaseController;
import com.madara.KnowledgeRecord;

import java.util.StringTokenizer;

import edu.cmu.ri.crw.AbstractVehicleServer;
import edu.cmu.ri.crw.VehicleServer.WaypointState;
import edu.cmu.ri.crw.data.Utm;
import edu.cmu.ri.crw.data.UtmPose;
import robotutils.Pose3D;

/**
 * Simple algorithm: continually try to maintain current global location
 *
 * @author jjb
 *
 */
public class DwellAlgorithm extends BaseAlgorithm {

    public DwellAlgorithm() {
    }

    @Override
    public void init(BaseController controller) {
        super.init(controller);

    }

    @Override
    public int analyze() {
        return 0;
    }

    @Override
    public int plan() {


        return 0;
    }

    @Override
    public int execute() {

        return 0;
    }


    public void shutdown() {
        // Free MADARA containers
    }
}
