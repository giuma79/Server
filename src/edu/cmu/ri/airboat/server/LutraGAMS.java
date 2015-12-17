package edu.cmu.ri.airboat.server;

import android.provider.Settings;
import android.util.Log;

import edu.cmu.ri.crw.AbstractVehicleServer;
import edu.cmu.ri.crw.data.Twist;
import edu.cmu.ri.crw.data.UtmPose;

import com.gams.algorithms.BaseAlgorithm;
import com.gams.controllers.BaseController;
import com.madara.KnowledgeBase;

//import com.madara.KnowledgeList;
//import com.madara.KnowledgeRecord;
//import com.madara.KnowledgeType;

import com.madara.Variables;
import com.madara.filters.EndpointClear;
import com.madara.transport.QoSTransportSettings;
import com.madara.transport.TransportType;
import com.madara.transport.filters.RecordFilter;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * @author jjb
 */
public class LutraGAMS extends AbstractVehicleServer {

    int id;
    String name;
    int teamSize;
    public THRUST_TYPES thrustType;
    final double GAMS_MAPE_LOOP_RUN_HZ = 5.0;
    final double GAMS_SEND_HZ = 2.0;

    BaseController controller;
    LutraPlatform platform;
    QoSTransportSettings settings;
    //QoSTransportSettings analyticsAgentSettings;
    //QoSTransportSettings simSettings;

    KnowledgeBase knowledge;
    BaseAlgorithm algorithm;

    private String MadaraLogFilename() {
        Date d = new Date();
        SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_hhmmss");
        return String.format("/mnt/sdcard/MADARA_LOG_AGENT#%d_",id) + sdf.format(d) + ".txt";
    }

    private String GAMSLogFilename() {
        Date d = new Date();
        SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_hhmmss");
        return String.format("/mnt/sdcard/GAMS_LOG_AGENT#%d_",id) + sdf.format(d) + ".txt";
    }

            /*
            args[0]: The knowledge record that the filter is acting upon
            args[1]: The name of the knowledge record, if applicable ("" if unnamed, but this should never happen)
            args[2]: The type of operation calling the filter (integer valued). Valid types are: Madara::Transport::Transport_Context::IDLE_OPERATION (should never see), Madara::Transport::Transport_Context::SENDING_OPERATION (transport is trying to send the record), Madara::Transport::Transport_Context::RECEIVING_OPERATION (transport has received the record and is ready to apply the update), Madara::Transport::Transport_Context::REBROADCASTING_OPERATION (transport is trying to rebroadcast the record -- only happens if rebroadcast is enabled in Transport Settings)
            args[3]: Bandwidth used while sending through this transport, measured in bytes per second.
            args[4]: Bandwidth used while receiving from this transport, measured in bytes per second.
            args[5]: Message timestamp (when the message was originally sent, in seconds)
            args[6]: Current timestamp (the result of time (NULL))
            args[7]: Knowledge Domain (partition of the knowledge updates)
            args[8]: Originator (identifier of sender, usually host:port)
            */

    /*
    public class PacketReceiveLogFilter implements RecordFilter {
        public KnowledgeRecord filter(KnowledgeList args, Variables variables) {
            KnowledgeRecord result = new KnowledgeRecord();

            String logString = String.format("originator %s, sent %d, rcvd %d, bandwidth %d, %s = %s",
                    args.get(8).toString(),
                    args.get(5).toLong(),
                    args.get(6).toLong(),
                    args.get(4).toLong(),
                    args.get(1).toString(),
                    args.get(0).toString());

            Log.i("jjb_RCVFILTER",logString);

            result = args.get(0); // we're just logging this, so send it through
            return result;
        }
    }

    public class PacketSendLogFilter implements RecordFilter {
        public KnowledgeRecord filter(KnowledgeList args, Variables variables) {
            KnowledgeRecord result = new KnowledgeRecord();

            String logString = String.format("sent %d, bandwidth %d, %s = %s",
                    args.get(5).toLong(),
                    args.get(4).toLong(),
                    args.get(1).toString(),
                    args.get(0).toString());

            Log.i("jjb_SNDFILTER",logString);

            result = args.get(0);
            return result;
        }
    }
    */


    public LutraGAMS(int id, String name, int teamSize, THRUST_TYPES thrustType) {
        this.id = id;
        this.name = name;
        this.teamSize = teamSize;
        this.thrustType = thrustType;


        //Log.i("jjb", "ABOUT TO CREATE MADARA LOGFILE");
        //com.madara.logger.GlobalLogger.clear();
        //com.madara.logger.GlobalLogger.setLevel(6);
        //com.madara.logger.GlobalLogger.setTimestampFormat("%F  %X: ");
        //com.madara.logger.GlobalLogger.addFile(MadaraLogFilename());



        //Log.i("jjb", "ABOUT TO CREATE GAMS LOGFILE");
        //com.gams.logger.GlobalLogger.clear();
        //com.gams.logger.GlobalLogger.setLevel(6);
        //0com.gams.logger.GlobalLogger.setTimestampFormat("%F  %X: ");
        //com.gams.logger.GlobalLogger.addFile(GAMSLogFilename());


        settings = new QoSTransportSettings();


        settings.setHosts(new String[]{"192.168.1.255:15000"});
        settings.setType(TransportType.BROADCAST_TRANSPORT);
        //settings.setRebroadcastTtl(1);
        //settings.enableParticipantTtl(1);
        settings.setDeadline(3);
        //settings.addReceiveFilter(KnowledgeType.ALL_TYPES, new PacketReceiveLogFilter());
        //settings.addSendFilter(KnowledgeType.ALL_TYPES, new PacketSendLogFilter());



        /*
        settings.setHosts(new String[]{"registry.senseplatypus.com:6078"});
        settings.setType(TransportType.REGISTRY_CLIENT);
        EndpointClear filter = new EndpointClear();
        filter.addReceiveFilterTo(settings);
        */


        knowledge = new KnowledgeBase(String.format("device.%d_KB",id),settings);

        /*
        simSettings = new QoSTransportSettings();
        simSettings.setHosts(new String[]{"239.255.0.1:4150"});
        simSettings.setType(TransportType.MULTICAST_TRANSPORT);
        knowledge.attachTransport(String.format("agent.%d_KB", id),simSettings);
        */

        controller = new BaseController(knowledge);

        /*
        analyticsAgentSettings = new QoSTransportSettings();
        settings.setHosts( ... ); unicast?
        analyticsAgentSettings.setDomains( ... ); comma separated "domain1,domain2"? how to send to a specific domain?
        */

    }


    void start(final AbstractVehicleServer lutra) {
        controller.initVars(id, teamSize);
        platform = new LutraPlatform(knowledge,thrustType, name);
        algorithm = new DwellAlgorithm();
        controller.initPlatform(platform);
        controller.initAlgorithm(algorithm);
        platform.start();
        new Thread(new Runnable() {
            @Override
            public void run() {
                controller.runHz(GAMS_MAPE_LOOP_RUN_HZ,72000.0,GAMS_SEND_HZ); // run --> time interval, duration, send interval |  runHz --> run Hz, run duration, send Hz
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
