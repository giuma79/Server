package edu.cmu.ri.airboat.server;

import edu.cmu.ri.crw.AbstractVehicleServer;
import edu.cmu.ri.crw.data.Twist;
import edu.cmu.ri.crw.data.UtmPose;

import com.gams.algorithms.BaseAlgorithm;
import com.gams.controllers.BaseController;
import com.madara.KnowledgeBase;
import com.madara.transport.QoSTransportSettings;
import com.madara.transport.TransportType;


/**
 * @author jjb
 */
public class LutraGAMS extends AbstractVehicleServer {

    int id;
    int teamSize;
    String ipAddress;

    BaseController controller;
    LutraPlatform platform;
    QoSTransportSettings settings;
    KnowledgeBase knowledge;
    BaseAlgorithm algorithm;

    public LutraGAMS(int id, int teamSize, String ipAddress) {
        this.id = id;
        this.teamSize = teamSize;
        this.ipAddress = ipAddress;

        platform = new LutraPlatform(knowledge);
        settings = new QoSTransportSettings();
        settings.setHosts(new String[]{"239.255.0.1:4150"});
        settings.setType(TransportType.MULTICAST_TRANSPORT);
        knowledge = new KnowledgeBase(ipAddress,settings);
        controller = new BaseController(knowledge);
        controller.initVars(id, teamSize);
        controller.initPlatform(platform);
        controller.initAlgorithm(algorithm);
    }









    //////////////////////////////////// Unused stuff START
    public UtmPose[] getWaypoints() {return null;}
    public void setAutonomous(boolean isAutonomous){}
    public boolean isAutonomous(){return true;}
    public boolean isConnected() {return true;}
    public WaypointState getWaypointStatus() {return null;}
    public void stopWaypoints() {}
    public void startWaypoints(final UtmPose[] waypoints, final String controller) {}
    public Twist getVelocity() {return null;}
    public void setVelocity(Twist vel) {}
    public void setPose(UtmPose utmPose) {}
    public UtmPose getPose() {return null;}
    public byte[] captureImage(int i, int i1) {return new byte[0];}
    public void startCamera(int i, double v, int i1, int i2) {}
    public void stopCamera() {}
    public CameraState getCameraStatus() {return null;}
    public void setSensorType(int i, SensorType sensorType) {}
    public SensorType getSensorType(int i) {return null;}
    public int getNumSensors() {
        return 0;
    }
//////////////////////////////////// Unused stuff END

}
