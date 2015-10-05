package edu.cmu.ri.airboat.server;

import android.provider.Settings;
import android.util.Log;

import edu.cmu.ri.crw.AbstractVehicleServer;
import edu.cmu.ri.crw.data.Twist;
import edu.cmu.ri.crw.data.UtmPose;

import com.gams.algorithms.BaseAlgorithm;
import com.gams.controllers.BaseController;
import com.madara.KnowledgeBase;
import com.madara.logger.GlobalLogger;
import com.madara.transport.QoSTransportSettings;
import com.madara.transport.TransportType;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * @author jjb
 */
public class LutraGAMS extends AbstractVehicleServer {

    int id;
    int teamSize;
    public THRUST_TYPES thrustType;

    BaseController controller;
    LutraPlatform platform;
    QoSTransportSettings settings;
    //QoSTransportSettings simSettings;

    KnowledgeBase knowledge;
    BaseAlgorithm algorithm;

    private static String MadaraLogFilename() {
        Date d = new Date();
        SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_hhmmss");
        return "/mnt/sdcard/MADARA_LOG_" + sdf.format(d) + ".txt";
    }

    public LutraGAMS(int id, int teamSize, THRUST_TYPES thrustType) {
        this.id = id;
        this.teamSize = teamSize;
        this.thrustType = thrustType;

        settings = new QoSTransportSettings();
        settings.setHosts(new String[]{"192.168.1.255:15000"});
        settings.setType(TransportType.BROADCAST_TRANSPORT);
        //settings.setRebroadcastTtl(1);
        //settings.enableParticipantTtl(1);
        settings.setDeadline(2);


        knowledge = new KnowledgeBase(String.format("device.%d_KB",id),settings);


        /*
        simSettings = new QoSTransportSettings();
        simSettings.setHosts(new String[]{"239.255.0.1:4150"});
        simSettings.setType(TransportType.MULTICAST_TRANSPORT);
        knowledge.attachTransport(String.format("device.%d_KB", id),simSettings);
        */

        controller = new BaseController(knowledge);


        Log.i("jjb","ABOUT TO CREATE MADARA LOGFILE");
        GlobalLogger.clear();
        //GlobalLogger.setLevel(1);
        //GlobalLogger.setTimestampFormat("%F  %X: ");
        //GlobalLogger.addFile(MadaraLogFilename());




        //com.gams.utility.Logging.setLevel(6);
    }


    void start(final AbstractVehicleServer lutra) {
        controller.initVars(id, teamSize);
        platform = new LutraPlatform(knowledge,thrustType);
        algorithm = new DwellAlgorithm();
        controller.initPlatform(platform);
        controller.initAlgorithm(algorithm);
        platform.start();
        new Thread(new Runnable() {
            @Override
            public void run() {
                controller.runHz(5.0,7200.0,5.0); // run --> time interval, duration, send interval |  runHz --> run Hz, run duration, send Hz
            }
        }).start();
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
