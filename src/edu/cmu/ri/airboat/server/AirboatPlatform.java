package edu.cmu.ri.airboat.server;


import com.gams.controllers.BaseController;
import com.gams.platforms.BasePlatform;
import com.gams.platforms.Status;
import com.gams.utility.Position;
import com.madara.EvalSettings;
import com.madara.KnowledgeBase;

import org.jscience.geography.coordinates.LatLong;
import org.jscience.geography.coordinates.UTM;
import org.jscience.geography.coordinates.crs.CoordinatesConverter;
import org.jscience.geography.coordinates.crs.ReferenceEllipsoid;

import java.io.FileWriter;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.measure.quantity.Angle;
import javax.measure.unit.NonSI;
import javax.measure.unit.SI;

import edu.cmu.ri.crw.AbstractVehicleServer;
import edu.cmu.ri.crw.AsyncVehicleServer;
import edu.cmu.ri.crw.FunctionObserver;
import edu.cmu.ri.crw.ImageListener;
import edu.cmu.ri.crw.PoseListener;
import edu.cmu.ri.crw.SensorListener;
import edu.cmu.ri.crw.VehicleServer;
import edu.cmu.ri.crw.data.SensorData;
import edu.cmu.ri.crw.data.Utm;
import edu.cmu.ri.crw.data.UtmPose;
import robotutils.Pose3D;
import robotutils.Quaternion;

/**
 * GAMS BasePlatform implementation
 *
 * @author nbb
 *
 */
public class AirboatPlatform extends BasePlatform implements PoseListener, SensorListener, ImageListener {

    public static final boolean PRINT_DEVICE_KB = true;
    public static final long PRINT_DEVICE_KB_ID = 1;
    public static final int PRINT_LEADER_KB_RATE = 0; // msec

    com.madara.containers.Vector waypoints;
    com.madara.containers.String wpEventId;
    com.madara.containers.String wpState;
    com.madara.containers.String wpController;
    com.madara.containers.Integer waypointsReceivedAck;
    com.madara.containers.Integer waypointsCompletedAck;

    // Delay for sending modified fields
    EvalSettings evalSettings;

    // Local reference to vehicle server.
//    protected AsyncVehicleServer _server;
    protected AbstractVehicleServer _server;
    // IP address including port number
    protected String _ipAddress;
    // Device id
    protected final int id;

    long lastTime = 0, curTime;

    public AirboatPlatform(AbstractVehicleServer _server, String ipAddress, int id) {
        this._server = _server;
        this._ipAddress = ipAddress;
        this.id = id;

        evalSettings = new EvalSettings();
        evalSettings.setDelaySendingModifieds(true);
    }

    @Override
    public void init(BaseController controller) {
        super.init(controller);
        self.id.set(id);

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

        _server.addImageListener(this);
//        _server.addImageListener(this, new FunctionObserver<Void>() {
//
//            @Override
//            public void completed(Void v) {
//            }
//
//            @Override
//            public void failed(FunctionObserver.FunctionError fe) {
//            }
//        });
        _server.addPoseListener(this);
//        _server.addPoseListener(this, new FunctionObserver<Void>() {
//
//            @Override
//            public void completed(Void v) {
//            }
//
//            @Override
//            public void failed(FunctionObserver.FunctionError fe) {
//            }
//        });
//        _server.addSensorListener(0, this);
//        _server.addSensorListener(0, this, new FunctionObserver<Void>() {
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

    /**
     * Analyzes the platform.
     *
     * @return status information (@see Status)
     *
     */
    public int analyze() {
        if (PRINT_DEVICE_KB) {
            if (self.id.toLong() == PRINT_DEVICE_KB_ID) {
                curTime = System.currentTimeMillis();
                if (curTime - lastTime >= PRINT_LEADER_KB_RATE) {
                    lastTime = curTime;
//            LOGGER.info("Print KB START for " + self.id);
                    knowledge.print();
//            LOGGER.info("Printing KB DONE for " + self.id);
                }
            }
        }
//    System.out.println("Platform.analyze called");

        return Status.OK.value();
    }

    /**
     * Returns the position accuracy in meters
     *
     * @return position accuracy
     *
     */
    public double getPositionAccuracy() {
//    System.out.println("  Platform.getPositionAccuracy called");
        return 5.0;
    }

    public double getGpsAccuracy() {
//    System.out.println("  Platform.getPositionAccuracy called");
        return 5.0;
    }

    /**
     * Returns the current GPS position
     *
     */
    public Position getPosition() {
//    System.out.println("  Platform.getPosition called");
        Position position = new Position(self.device.location.get(0), self.device.location.get(1), self.device.location.get(2));
        return position;
    }

    /**
     * Returns to the home location. This should be a non-blocking call.
     *
     * @return status information (@see Status)
     *
     */
    public int home() {
//    System.out.println("  Platform.home called");
        return Status.OK.value();
    }

    /**
     * Requests the platform to land. This should be a non-blocking call.
     *
     * @return status information (@see Status)
     *
     */
    public int land() {
//    System.out.println("  Platform.land called");
        return Status.OK.value();
    }

    /**
     * Initializes a move to the target position. This should be a non-blocking
     * call.
     *
     * @param target the new position to move to
     * @param proximity the minimum distance between current position and target
     * position that terminates the move.
     * @return status information (@see Status)
     *
     */
    public int move(Position target, double proximity) {
//        System.out.println(id + " Platform.move called");

        // Update SAMI boat proxy variables
        wpController.set("POINT_AND_SHOOT");
        wpState.set(VehicleServer.WaypointState.GOING.toString());
        waypointsReceivedAck.set(0);
        waypointsCompletedAck.set(0);

        // Send single point as waypoint list to server
        UtmPose[] _wpList = new UtmPose[1];

        // Convert from lat/long to UTM coordinates
        UTM utmLoc = UTM.latLongToUtm(
                LatLong.valueOf(target.getX(), target.getY(), NonSI.DEGREE_ANGLE),
                ReferenceEllipsoid.WGS84);

        // Convert to UTM data structure
        Pose3D pose = new Pose3D(utmLoc.eastingValue(SI.METER),
                utmLoc.northingValue(SI.METER), target.getZ(),
                Quaternion.fromEulerAngles(0, 0, 0));
        Utm origin = new Utm(utmLoc.longitudeZone(),
                utmLoc.latitudeZone() > 'O');
        UtmPose utm = new UtmPose(pose, origin);
        _wpList[0] = utm;

        if (_wpList[0] != null) {
            _server.startWaypoints(_wpList, wpController.get());
//            _server.startWaypoints(_wpList, wpController.get(), new FunctionObserver<Void>() {
//
//                @Override
//                public void completed(Void v) {
//                }
//
//                @Override
//                public void failed(FunctionObserver.FunctionError fe) {
//                }
//            });
        }

        // Write destination and source to device id path
        self.device.dest.set(0, target.getX());
        self.device.dest.set(1, target.getY());
        self.device.dest.set(2, target.getZ());
        self.device.source.set(0, self.device.location.get(0));
        self.device.source.set(1, self.device.location.get(1));
        self.device.source.set(2, self.device.location.get(2));

        return Status.OK.value();
    }

    /**
     * Get sensor radius
     *
     * @return minimum radius of all available sensors for this platform
     */
    public double getMinSensorRange() {
//    System.out.println("  Platform.getMinSensorRange called");
        return 0.0;
    }

    /**
     * Gets the movement speed
     *
     * @return movement speed
     *
     */
    public double getMoveSpeed() {
//    System.out.println("  Platform.getMoveSpeed called");
        return 10.0;
    }

    /**
     * Gets the unique id of the platform. This should be an alphanumeric id
     * that can be part of a MADARA variable name. Specifically, this is used in
     * the variable expansion of .platform.{yourid}.
     *
     *
     * @return the id of the platform (alphanumeric only: no spaces!)
     *
     */
    public java.lang.String getId() {
        // Get IP address
        return _ipAddress;
    }

    /**
     * Gets the name of the platform
     *
     * @return the name of the platform
     *
     */
    public java.lang.String getName() {
        return "Lutra";
    }

    /**
     * Gets results from the platform's sensors. This should be a non-blocking
     * call.
     *
     * @return 1 if moving, 2 if arrived, 0 if an error occurred
     *
     */
    public int sense() {
//    System.out.println("Platform.sense called");
        return Status.OK.value();
    }

    /**
     * Sets move speed
     *
     * @param speed new speed in meters/second
     *
     */
    public void setMoveSpeed(double speed) {
//    System.out.println("  Platform.setMoveSpeed called with " + speed);
    }

    /**
     * Takes off. This should be a non-blocking call.
     *
     * @return status information (@see Status)
     *
     */
    public int takeoff() {
//    System.out.println("  Platform.takeoff called");
        return Status.OK.value();
    }

    /**
     * Stops moving
     *
     */
    public void stopMove() {
//    System.out.println("  Platform.stopMove called");
        _server.stopWaypoints();
//        _server.stopWaypoints(
//                new FunctionObserver<Void>() {
//
//                    @Override
//                    public void completed(Void v) {
//                        // Clear waypoints?
//                    }
//
//                    @Override
//                    public void failed(FunctionObserver.FunctionError fe) {
//                    }
//                });
    }

    @Override
    public void receivedPose(UtmPose utmPose) {
//        LOGGER.info("Pose update START for " + self.id + ", " + utmPose.toString());
        setUtmPose(knowledge, _ipAddress + ".pose", utmPose);
        knowledge.sendModifieds();
//        LOGGER.info("Pose update DONE for " + self.id + ", " + utmPose.toString());
    }

    /**
     * Conversion method that takes a UTMPose and a variable name in MADARA and
     * set the variable in the provided knowledge base to the given UtmPose.
     *
     * @param knowledge a knowledge base that will be updated
     * @param knowledgePath the name of the variable in the knowledge base that
     * should be updated
     * @param utmPose the UTM pose that the knowledge base will be updated with
     */
    public void setUtmPose(KnowledgeBase knowledge, String knowledgePath, UtmPose utmPose) {
        // @todo Redirect SAMI knowledge path to use device id instead of ip address

//        System.out.println("setUtmPose " + utmPose.toString());
        // Write pose to ip address path
        knowledge.set(knowledgePath + ".x", utmPose.pose.getX(), evalSettings);
        knowledge.set(knowledgePath + ".y", utmPose.pose.getY(), evalSettings);
        knowledge.set(knowledgePath + ".z", utmPose.pose.getZ(), evalSettings);
        knowledge.set(knowledgePath + ".roll", utmPose.pose.getRotation().toRoll(), evalSettings);
        knowledge.set(knowledgePath + ".pitch", utmPose.pose.getRotation().toPitch(), evalSettings);
        knowledge.set(knowledgePath + ".yaw", utmPose.pose.getRotation().toYaw(), evalSettings);
        knowledge.set(knowledgePath + ".zone", utmPose.origin.zone, evalSettings);
        knowledge.set(knowledgePath + ".hemisphere", utmPose.origin.isNorth ? "North" : "South", evalSettings);

        // Write pose to device location path
        UTM utm = UTM.valueOf(utmPose.origin.zone, utmPose.origin.isNorth ? 'N' : 'S', utmPose.pose.getX(), utmPose.pose.getY(), SI.METRE);
        CoordinatesConverter<UTM, LatLong> utmToLatLong = UTM.CRS.getConverterTo(LatLong.CRS);
        LatLong latLong = utmToLatLong.convert(utm);

        self.device.location.set(0, latLong.latitudeValue(NonSI.DEGREE_ANGLE));
        self.device.location.set(1, latLong.longitudeValue(NonSI.DEGREE_ANGLE));
        self.device.location.set(2, utmPose.pose.getZ());
    }

    @Override
    public void receivedSensor(SensorData sensorData) {
        knowledge.set(".sensor." + sensorData.channel, sensorData.data, evalSettings);
    }

    @Override
    public void receivedImage(byte[] bytes) {
        // TODO: how do you set an IMAGE_JPEG to a variable in the knowledge base?
    }

    public void shutdown() {
        // Free MADARA containers
    }
}
