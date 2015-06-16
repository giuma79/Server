package edu.cmu.ri.airboat.server;

import com.gams.utility.Position;

import org.apache.commons.math.linear.RealMatrix;



enum SENSOR_TYPES {
    GPS, COMPASS, GYRO, IMU
}

/**
 * @author jjb
 */
public class Datum {

    SENSOR_TYPES type;
    Position location;
    Long timestamp;
    RealMatrix z; // sensor value
    RealMatrix R; // sensor covariance


    public Datum(SENSOR_TYPES type, Long timestamp, RealMatrix z) {
        this.type = type;
        this.timestamp = timestamp;
        this.z = z.copy();
    }
    public Datum(SENSOR_TYPES type, Long timestamp, RealMatrix z, RealMatrix R) { // for localization data
        this(type,timestamp,z);
        this.R = R.copy();
    }
    public Datum(SENSOR_TYPES type, Position location, Long timestamp, RealMatrix z) { // for environmental data
        this(type,timestamp,z);
        this.location = location;
    }


    public void setZ(RealMatrix z) {this.z = z.copy(); }
    public void setR(RealMatrix R) {this.R = R.copy(); }
    public void setLocation(Position location) {this.location = location;}
    public void setTimestamp(Long timestamp) {this.timestamp = timestamp;}
    public void setType(SENSOR_TYPES type) {this.type = type;}
    public RealMatrix getZ() {return this.z.copy();}
    public SENSOR_TYPES getType() {return this.type;}
    public RealMatrix getR() {return this.R.copy();}
    public Long getTimestamp() {return this.timestamp;}

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
        return "UNKNOWN";
    }


}
