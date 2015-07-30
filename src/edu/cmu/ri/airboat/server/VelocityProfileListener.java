package edu.cmu.ri.airboat.server;

import org.apache.commons.math.linear.RealMatrix;

import java.util.EventListener;

/**
 * @author jjb
 */
public interface VelocityProfileListener extends EventListener{
    void newProfile(RealMatrix profile);
}
