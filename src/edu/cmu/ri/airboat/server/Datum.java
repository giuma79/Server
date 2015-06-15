package edu.cmu.ri.airboat.server;

import com.gams.utility.Position;

import org.apache.commons.math.linear.RealMatrix;

import java.sql.Timestamp;

/**
 * @author jjb
 */
public class Datum {

    String type;
    Position location;
    Timestamp timestamp;
    RealMatrix z; // sensor value
    RealMatrix R; // sensor covariance

    public EnvironmentalDatum(String type, Position location, Timestamp timestamp, double value) {
        this.type = type;
        this.location = location;
        this.timestamp = timestamp;
        this.z = value;
    }

    public void setValue(double value) {this.value = value; }
    public void setLocation(Position location) {
        this.location = location;
    }
    public void setTimestamp(Timestamp timestamp) {
        this.timestamp = timestamp;
    }
    public void setType(String type) {this.type = type;}
    public double getValue() {
        return this.value;
    }


}
