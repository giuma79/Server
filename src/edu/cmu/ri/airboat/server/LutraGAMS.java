package edu.cmu.ri.airboat.server;

import edu.cmu.ri.crw.AbstractVehicleServer;
import edu.cmu.ri.crw.data.Twist;
import edu.cmu.ri.crw.data.UtmPose;

import com.gams.algorithms.BaseAlgorithm;
//import com.gams.algorithms.DebuggerAlgorithm;
//import com.gams.platforms.DebuggerPlatform;
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
    public String ipAddress;

    BaseController controller;
    LutraPlatform platform;
    QoSTransportSettings settings;
    KnowledgeBase knowledge;
    BaseAlgorithm algorithm;

    public LutraGAMS(int id, int teamSize, String ipAddress) {
        this.id = id;
        this.teamSize = teamSize;
        this.ipAddress = ipAddress;

        settings = new QoSTransportSettings();
        settings.setHosts(new String[]{"239.255.0.1:4150"});
        settings.setType(TransportType.MULTICAST_TRANSPORT);
        knowledge = new KnowledgeBase(ipAddress,settings);
        controller = new BaseController(knowledge);
    }


    void start(final AbstractVehicleServer lutra) {


        controller.initVars(id, teamSize);
        platform = new LutraPlatform(knowledge);
        algorithm = new DwellAlgorithm(lutra, ipAddress);
        controller.initPlatform(platform);
        controller.initAlgorithm(algorithm);
        platform.start();
        new Thread(new Runnable() {
            @Override
            public void run() {
                controller.run(1.0/5.0,3600.0); // run --> time interval, duration |  runHz --> run Hz, run duration, send Hz
                //knowledge.print();
            }
        }).start();

        /*
        new Thread(new Runnable() {
            @Override
            public void run() {
                // Controllers require a knowledge base. This one will have no networking.
                System.out.println("Creating knowledge base...");
                KnowledgeBase knowledge = new KnowledgeBase();

                System.out.println("Passing knowledge base to base controller...");
                BaseController controller = new BaseController(knowledge);

                // give our agent id 0 of 4 processes
                controller.initVars(0, 4);

                // initialize the debugger platform and algorithm
                controller.initPlatform(new DebuggerPlatform());
                controller.initAlgorithm(new DebuggerAlgorithm());

                System.out.println("Running controller every 1s for 10s...");
                controller.run(1.0, 20.0);

                knowledge.print();

                controller.free();
                knowledge.free();
            }
        }).start();
        */

    }

    void shutdown() {
        knowledge.free();
        controller.free();
        platform.shutdown();
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
