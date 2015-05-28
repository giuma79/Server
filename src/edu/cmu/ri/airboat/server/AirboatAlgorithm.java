package edu.cmu.ri.airboat.server;

import android.util.Log;

import com.gams.algorithms.BaseAlgorithm;
import com.gams.controllers.BaseController;
import com.google.code.microlog4android.LoggerFactory;
import com.madara.KnowledgeRecord;

import java.util.StringTokenizer;
import java.util.logging.Logger;

import edu.cmu.ri.crw.AbstractVehicleServer;
import edu.cmu.ri.crw.AsyncVehicleServer;
import edu.cmu.ri.crw.FunctionObserver;
import edu.cmu.ri.crw.VehicleServer;
import edu.cmu.ri.crw.VehicleServer.WaypointState;
import edu.cmu.ri.crw.WaypointListener;
import edu.cmu.ri.crw.data.Utm;
import edu.cmu.ri.crw.data.UtmPose;
import robotutils.Pose3D;

/**
 * GAMS BaseAlgorithm implementation
 *
 * @author nbb
 *
 */
public class AirboatAlgorithm extends BaseAlgorithm implements WaypointListener {

    com.madara.containers.Vector waypoints;
    com.madara.containers.String wpEventId;
    com.madara.containers.String wpState;
    com.madara.containers.String wpController;
    com.madara.containers.Integer waypointsReceivedAck;
    com.madara.containers.Integer waypointsCompletedAck;
    com.madara.containers.Integer autonomyEnabled;
    com.madara.containers.Integer autonomyEnabledReceivedAck;
    private UtmPose[] _wpList;
    //    private int _wpEventId;
//    private int _wpIndex;
    private boolean _startNewWaypoints = false;

    // Local reference to vehicle server.
    protected AbstractVehicleServer _server;
    // IP address including port number
    protected String _ipAddress;

    public AirboatAlgorithm(AbstractVehicleServer _server, String ipAddress) {
        this._server = _server;
        this._ipAddress = ipAddress;
    }

    @Override
    public void init(BaseController controller) {
        super.init(controller);

        waypoints = new com.madara.containers.Vector();
        waypoints.setName(knowledge, _ipAddress + ".waypoints");

        wpEventId = new com.madara.containers.String();
        wpEventId.setName(knowledge, _ipAddress + ".waypoints.eventId");

        wpState = new com.madara.containers.String();
        wpState.setName(knowledge, _ipAddress + ".waypoints.state");

        wpController = new com.madara.containers.String();
        wpController.setName(knowledge, _ipAddress + ".waypoints.controller");

        waypointsReceivedAck = new com.madara.containers.Integer();
        waypointsReceivedAck.setName(knowledge, _ipAddress + ".waypoints.received");

        waypointsCompletedAck = new com.madara.containers.Integer();
        waypointsCompletedAck.setName(knowledge, _ipAddress + ".waypoints.completed");

        autonomyEnabled = new com.madara.containers.Integer();
        autonomyEnabled.setName(knowledge, _ipAddress + ".autonomy");

        autonomyEnabledReceivedAck = new com.madara.containers.Integer();
        autonomyEnabledReceivedAck.setName(knowledge, _ipAddress + ".autonomy.received");

        _server.addWaypointListener(this);
//        _server.addWaypointListener(this, new FunctionObserver<Void>() {
//
//            @Override
//            public void completed(Void v) {
//            }
//
//            @Override
//            public void failed(FunctionObserver.FunctionError fe) {
//            }
//        });
    }

    @Override
    public int analyze() {
//        if(self.id.toLong() == 0) {
//            System.out.println("ANALYZE\t " + waypointsReceivedAck.get());
//        }
        if (autonomyEnabledReceivedAck.get() == 1) {
            if (autonomyEnabled.get() == 0) {
                wpState.set(WaypointState.OFF.toString());
            } else {
                // What to set wpState to?
            }

            _server.setAutonomous(autonomyEnabled.get() == 1);
//            _server.setAutonomous(autonomyEnabled.get() == 1, new FunctionObserver<Void>() {
//
//                @Override
//                public void completed(Void v) {
//                }
//
//                @Override
//                public void failed(FunctionObserver.FunctionError fe) {
//                }
//            });
            autonomyEnabledReceivedAck.set(0);
        }
        if (waypointsReceivedAck.get() == 1) {
            waypoints.resize(-1);
            KnowledgeRecord[] records = waypoints.toArray();
            _wpList = new UtmPose[records.length];
            for (int i = 0; i < records.length; i++) {
                UtmPose utmPose = getUtmPose(records[i].toString());
                _wpList[i] = utmPose;
                records[i].free();
            }
            _startNewWaypoints = true;
            waypointsReceivedAck.set(0);
            if (_wpList.length == 0) {
                wpState.set(WaypointState.DONE.toString());
            } else {
                wpState.set(WaypointState.GOING.toString());
            }
        }

        return 0;
    }

    @Override
    public int plan() {
//        if(self.id.toLong() == 0) {
//            System.out.println("PLAN");
//        }
        if (_startNewWaypoints) {
            if (_wpList.length == 0) {
                _server.stopWaypoints();
//                _server.stopWaypoints(new FunctionObserver<Void>() {
//
//                    @Override
//                    public void completed(Void v) {
//                    }
//
//                    @Override
//                    public void failed(FunctionObserver.FunctionError fe) {
//                    }
//                });
            } else {
                UtmPose[] _wpListClone = _wpList.clone();
//                System.out.println("start waypoints " + _wpListClone.length);
                _server.startWaypoints(_wpListClone, wpController.get());
//                _server.startWaypoints(_wpListClone, wpController.get(), new FunctionObserver<Void>() {
//
//                    @Override
//                    public void completed(Void v) {
//                    }
//
//                    @Override
//                    public void failed(FunctionObserver.FunctionError fe) {
//                    }
//                });
            }
        }
        _startNewWaypoints = false;

        return 0;
    }

    @Override
    public int execute() {
//        if(self.id.toLong() == 0) {
//            System.out.println("EXECUTE");
//        }
        return 0;
    }

    /**
     * Conversion method that takes a variable name in a MADARA Knowledge Record
     * and extracts a UtmPose from that variable.
     *
     * @param utmString UTM pose in String format
     * @return UTMPose formatted object
     */
    public static UtmPose getUtmPose(String utmString) {
        // Convert a knowledge record into a UTM pose.
        StringTokenizer st = new StringTokenizer(utmString, ",");
        int numTokens = st.countTokens();
        // 8 arg: easting, northing, altitude, roll, pitch, yaw, zone, hemisphere
        // 5 arg: easting, northing, yaw, zone, hemisphere
        // 4 arg: easting, northing, zone, hemisphere
        double easting;
        double northing;
        double altitude = 0;
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        int zone;
        String hemisphere;
        boolean isNorth;
        try {
            if (numTokens == 8) {
                // 8 arg: easting, northing, altitude, roll, pitch, yaw, zone,hemisphere
                easting = Double.valueOf(st.nextToken());
                northing = Double.valueOf(st.nextToken());
                altitude = Double.valueOf(st.nextToken());
                roll = Double.valueOf(st.nextToken());
                pitch = Double.valueOf(st.nextToken());
                yaw = Double.valueOf(st.nextToken());
                zone = Integer.valueOf(st.nextToken());
                hemisphere = st.nextToken();
            } else if (numTokens == 5) {
                // 5 arg: easting, northing, yaw, zone,hemisphere
                easting = Double.valueOf(st.nextToken());
                northing = Double.valueOf(st.nextToken());
                yaw = Double.valueOf(st.nextToken());
                zone = Integer.valueOf(st.nextToken());
                hemisphere = st.nextToken();
            } else if (numTokens == 4) {
                // 4 arg: easting, northing, zone,hemisphere
                easting = Double.valueOf(st.nextToken());
                northing = Double.valueOf(st.nextToken());
                zone = Integer.valueOf(st.nextToken());
                hemisphere = st.nextToken();
            } else {
                return null;
            }
            isNorth = hemisphere.charAt(0) == 'N' || hemisphere.charAt(0) == 'n';

            return new UtmPose(new Pose3D(easting, northing, altitude, roll, pitch, yaw), new Utm(zone, isNorth));
        } catch (Exception e) {
            e.printStackTrace();
        }
        return null;
    }

    /**
     * Receive changes in waypoint state from the vehicle server and update the
     * MADARA waypoint variables to match.
     *
     * @param waypointState the latest state of vehicle waypoint execution
     */
    @Override
    public void waypointUpdate(final VehicleServer.WaypointState waypointState) {
        if (!wpState.get().equalsIgnoreCase(waypointState.toString())) {
            wpState.set(waypointState.toString());
            if (wpState.get().equalsIgnoreCase(WaypointState.DONE.toString())) {
                waypointsCompletedAck.set(1);
            }
        }
    }

    public void shutdown() {
        // Free MADARA containers
    }
}
