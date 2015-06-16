package edu.cmu.ri.airboat.server;

import com.madara.KnowledgeBase;
import com.madara.containers.DoubleVector;

import org.apache.commons.math.linear.RealMatrix;

/**
 * @author jjb
 */
public class BoatMotionController {

    RealMatrix x;
    RealMatrix xd;
    KnowledgeBase knowledge;
    DoubleVector x_KB;

    public BoatMotionController(KnowledgeBase knowledge) {
        this.knowledge = knowledge;
        x_KB.setName(this.knowledge,".x");
    }

    public void control() {
    }

}
