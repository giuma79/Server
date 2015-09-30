package edu.cmu.ri.airboat.server;

import com.gams.utility.Position;

import org.apache.commons.math.linear.RealMatrix;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.EnumSet;
import java.util.Set;

enum SENSOR_CATEGORY {
    LOCALIZATION, ENVIRONMENTAL
}
enum SENSOR_TYPE {

    GPS(SENSOR_CATEGORY.LOCALIZATION,false),
    COMPASS(SENSOR_CATEGORY.LOCALIZATION,false),
    GYRO(SENSOR_CATEGORY.LOCALIZATION,false),
    IMU(SENSOR_CATEGORY.LOCALIZATION,false),
    DGPS(SENSOR_CATEGORY.LOCALIZATION,false),
    MOTOR(SENSOR_CATEGORY.LOCALIZATION,false),
    EC(SENSOR_CATEGORY.ENVIRONMENTAL,false),
    TEMP(SENSOR_CATEGORY.ENVIRONMENTAL,true),
    DO(SENSOR_CATEGORY.ENVIRONMENTAL,true),
    WIFI(SENSOR_CATEGORY.ENVIRONMENTAL,false),
    DEPTH(SENSOR_CATEGORY.ENVIRONMENTAL,false);

    SENSOR_CATEGORY category;
    boolean hysteresis;

    SENSOR_TYPE(SENSOR_CATEGORY category, boolean hysteresis) {
        this.category = category;
        this.hysteresis = hysteresis;
    }
    public static Set<SENSOR_TYPE> localization = EnumSet.of(GPS, COMPASS, GYRO, IMU, DGPS, MOTOR);
    public static Set<SENSOR_TYPE> environmental = EnumSet.of(EC, TEMP, DO, WIFI, DEPTH);

    /*
    public static int getEnvironmentalOrdinal(SENSOR_TYPE type) { // return index of environmental sensor --> not necessary if you use a HashMap
        SENSOR_TYPE[] types = SENSOR_TYPE.values();
        int environmentalSensorTypeCount = 0;
        for (int i = 0; i < types.length; i++) {
            if (types[i].category == SENSOR_CATEGORY.ENVIRONMENTAL) {
                if (type == types[i]) {
                    return environmentalSensorTypeCount;
                }
                environmentalSensorTypeCount++;
            }
        }
        return -1;
    }
    */
}

/**
 * @author jjb
 */
public class Datum {

    SENSOR_TYPE type;
    Position position;
    Long timestamp;
    RealMatrix z; // sensor value
    RealMatrix R; // sensor covariance
    long id;
    int boatID;
    static long idIncrement = 0;
    static DateFormat df = new SimpleDateFormat("yy/MM/dd HH:mm:ss");
    Date dateobj;

    static LutraMadaraContainers containers;

    public Datum(SENSOR_TYPE type, Long timestamp, RealMatrix z, int boatID) {
        this.type = type;
        this.timestamp = timestamp;
        this.boatID = boatID;
        this.z = z.copy();
        this.id = idIncrement;
        idIncrement++;
        dateobj = new Date();
    }
    public Datum(SENSOR_TYPE type, Long timestamp, RealMatrix z, RealMatrix R, int boatID) { // for localization data
        this(type,timestamp,z, boatID);
        this.R = R.copy();
    }
    public Datum(SENSOR_TYPE type, Position position, Long timestamp, RealMatrix z, int boatID) { // for environmental data
        this(type,timestamp,z, boatID);
        this.position = position;
    }

    static void setContainersObject(LutraMadaraContainers inputContainers) {
        containers = inputContainers;
    }

    public void setZ(RealMatrix z) {this.z = z.copy(); }
    public void setR(RealMatrix R) {this.R = R.copy(); }
    public void setPosition(Position position) {this.position = position;}
    public void setTimestamp(Long timestamp) {this.timestamp = timestamp;}
    public void setType(SENSOR_TYPE type) {this.type = type;}
    public RealMatrix getZ() {return this.z.copy();}
    public SENSOR_TYPE getType() {return this.type;}
    public RealMatrix getR() {return this.R.copy();}
    public Long getTimestamp() {return this.timestamp;}
    public Position getPosition() {return this.position;}

    @Override
    public String toString() {
        return String.format("TYPE = %s,  DATE = %s,  TIME = %d,  LAT = %.6e,  LONG = %.6e, VALUE = %s",
                typeString(this.type),df.format(dateobj),timestamp,position.getX(),position.getY(),zString());
    }

    String zString () {
        String a = "[";
        for (int i = 0; i < z.getRowDimension()-1; i++) {
            a = a + String.format("%f,",z.getEntry(i,0));
        }
        a = a + String.format("%f]",z.getEntry(z.getRowDimension()-1,0));
        return a;
    }

    public boolean isType(SENSOR_TYPE type) {return this.type == type;}

    static public String typeString(SENSOR_TYPE type) {
        if (type == SENSOR_TYPE.GPS) {
            return "GPS";
        }
        if (type == SENSOR_TYPE.COMPASS) {
            return "COMPASS";
        }
        if (type == SENSOR_TYPE.GYRO) {
            return "GYRO";
        }
        if (type == SENSOR_TYPE.IMU) {
            return "IMU";
        }
        if (type ==  SENSOR_TYPE.DGPS) {
            return "DGPS";
        }
        if (type == SENSOR_TYPE.MOTOR) {
            return "MOTOR";
        }
        if (type == SENSOR_TYPE.EC) {
            return "EC";
        }
        if (type == SENSOR_TYPE.TEMP) {
            return "TEMP";
        }
        if (type == SENSOR_TYPE.DO) {
            return "DO";
        }
        if (type == SENSOR_TYPE.WIFI) {
            return "WIFI";
        }
        return "UNKNOWN";
    }

    public void toKnowledgeBase() {
        // TODO: put everything into the environmentalData FlexMap in LutraMadaraContainers

        //long currentCount = ;


    }


}
