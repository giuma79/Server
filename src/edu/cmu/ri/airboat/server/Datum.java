package edu.cmu.ri.airboat.server;

import com.gams.utility.Position;

import org.apache.commons.math.linear.RealMatrix;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;


enum SENSOR_TYPES {
    GPS, COMPASS, GYRO, IMU, DGPS, MOTOR, ES2, EC, TEMP, DO
}

/**
 * @author jjb
 */
public class Datum {

    SENSOR_TYPES type;
    Position position;
    Long timestamp;
    RealMatrix z; // sensor value
    RealMatrix R; // sensor covariance
    long id;
    int boatID;
    static long idIncrement = 0;
    static DateFormat df = new SimpleDateFormat("yy/MM/dd HH:mm:ss");
    Date dateobj;

    public Datum(SENSOR_TYPES type, Long timestamp, RealMatrix z, int boatID) {
        this.type = type;
        this.timestamp = timestamp;
        this.boatID = boatID;
        this.z = z.copy();
        this.id = idIncrement;
        idIncrement++;
        dateobj = new Date();
    }
    public Datum(SENSOR_TYPES type, Long timestamp, RealMatrix z, RealMatrix R, int boatID) { // for localization data
        this(type,timestamp,z, boatID);
        this.R = R.copy();
    }
    public Datum(SENSOR_TYPES type, Position position, Long timestamp, RealMatrix z, int boatID) { // for environmental data
        this(type,timestamp,z, boatID);
        this.position = position;
    }

    public void setZ(RealMatrix z) {this.z = z.copy(); }
    public void setR(RealMatrix R) {this.R = R.copy(); }
    public void setPosition(Position position) {this.position = position;}
    public void setTimestamp(Long timestamp) {this.timestamp = timestamp;}
    public void setType(SENSOR_TYPES type) {this.type = type;}
    public RealMatrix getZ() {return this.z.copy();}
    public SENSOR_TYPES getType() {return this.type;}
    public RealMatrix getR() {return this.R.copy();}
    public Long getTimestamp() {return this.timestamp;}
    public Position getPosition() {return this.position;}
    public String toString() {
        return String.format("TYPE = %s,  DATE = %s,  LAT = %d,  LONG = %d, VALUE = %f",
                typeString(),df.format(dateobj),position.getX(),position.getY(),zString());
    }

    String zString () {
        String a = "[";
        for (int i = 0; i < z.getRowDimension()-1; i++) {
            a = a + String.format("%f,",z.getEntry(i,0));
        }
        a = a + String.format("%f]");
        return a;
    }

    public boolean isType(SENSOR_TYPES type) {return this.type == type;}

    public String typeString() {
        if (this.type == SENSOR_TYPES.GPS) {
            return "GPS";
        }
        if (this.type == SENSOR_TYPES.COMPASS) {
            return "COMPASS";
        }
        if (this.type == SENSOR_TYPES.GYRO) {
            return "GYRO";
        }
        if (this.type == SENSOR_TYPES.IMU) {
            return "IMU";
        }
        if (this.type ==  SENSOR_TYPES.DGPS) {
            return "DGPS";
        }
        if (this.type == SENSOR_TYPES.MOTOR) {
            return "MOTOR";
        }
        return "UNKNOWN";
    }

    public void toKnowledgeBase() {
        //
    }


}
