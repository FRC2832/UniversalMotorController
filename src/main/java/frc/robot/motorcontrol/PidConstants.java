package frc.robot.motorcontrol;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.Hashtable;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This class lets us configure PIDs using our units that make sense.  The input units should be 
 * something like meters or degrees, and the output would be in volts, with max as 12V.
 */
public class PidConstants {

    /** 
     * How much voltage do we add based on amount of error from set point.
     * For example, if P=2, that means for each input unit we are off, add 2 volts to the output.
     * 
     * Units: Volts/error
     */
    public double kP;

    /** 
     * How much voltage do we add based on the sum of error.  This number should be small!!!
     * For example, if I=2, for each input unit we are off for 1 sec, add 2 volts to the output.
     * 
     * Units: Volts/sum of error
     */
    public double kI;

    /**
     * How much voltage do we add to dampen rapid changes.
     * 
     * Units: Volts/change in error
     */
    public double kD;

    /**
     * How much voltage is needed to start the mechanism moving.  Setting to zero is usually fine.
     * 
     * Units: Volts
     */
    public double kS;

    /**
     * How much voltage is needed to overcome gravity.  Set to zero if not a vertical mechanism.
     *  
     * Units: Volts
     */
    public double kG;
    
    /**
     * Also called kF, how many sensor units change per volt.  Should be set to 12V/max rpm or unit.
     *  
     * Units: Volts / Rev/S
     */
    public double kV;

    /** 
     * How many sensor units^s squared change per volt.  Used to match the feed forward accelerations.
     * Set to zero in most applications.
     * 
     * Units: Volts / Rev/S^2
     */
    public double kA;

    /**
     * How much range you want before the I-Term enables.  For example, if kiZone=10, and the measurement
     * is 12 units off, ignore the I term.  OK to start at zero, this is used to stop integral windup when
     * the mechanism starts moving and usually adds more overshoot after reaching the target.
     * 
     * Units: Distance
     */
    public double kiZone;

    /**
     * How much range is allowable to turn off the PID.  For example, if kiError = 2, and the mechanism is 1
     * unit away, the PID would turn off.  OK to start at zero.  This has a side effect of basically always
     * stopping control the iError off, so probably good to keep off.
     * 
     * Units: Distance
     */
    public double kiError;

    /**
     * Used in motion control to say the max speed the mechanism should travel.
     * 
     * Units: Distance/Sec
     */
    public double kVelMax;

    /**
     * Used in motion control to say the max acceleration the mechanism should travel.
     * 
     * Units: Distance/Sec^2
     */
    public double kAccelMax;

    Hashtable<String,DoubleEntry> subs = new Hashtable<String,DoubleEntry>();
    Runnable onChange = null;

    /**
     * Create a generic Constants table with no backing
     */
    public PidConstants() {
        kP = 0;
        kI = 0;
        kD = 0;
        kS = 0;
        kG = 0;
        kV = 0;
        kA = 0;
        kiZone = 0;
        kiError = 0;
        kVelMax = 0;
        kAccelMax = 0;
    }

    public PidConstants(String key) {
        this(key, true);
    }
    /**
     * Create a generic Constants table with persistent backing from NetworkTables
     * @param key NetworkTable location to save at
     */
    public PidConstants(String key, boolean persist) {
        var inst = NetworkTableInstance.getDefault();
        ArrayList<String> topics = new ArrayList<>();
        //String baseName = "/Preferences/PID " + key.replace('/', '_') + "/";
        String baseName = "/PIDs/" + key.replace('/', '_') + "/";
        String[] variables = new String[] {
            "kP","kI","kD","kS","kG","kV","kA","kiZone","kiError","kVelMax","kAccelMax"
        };

        for(String variable : variables) {
            //get the topic to describe this entry
            String topicName = baseName + variable;
            topics.add(topicName);
            DoubleTopic topic = inst.getDoubleTopic(topicName);
            DoubleEntry entry = topic.getEntry(0);
            //publish the default value in case nothing is there yet so we can edit it easily
            if(!entry.exists()) {
                entry.set(0);
            }
            topic.setPersistent(persist);
            subs.put(variable, entry);
        }

        //create listener for remote changes 
        //NOTE: Remote does not work in the Simulation dashboard, need to use external dashboard like Elastic or Shuffleboard
        MultiSubscriber multi = new MultiSubscriber(inst, topics.toArray(new String[0]));
        inst.addListener(
            multi,
            EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
            event -> {
                if (onChange != null) {
                    loadFromNT();
                    onChange.run();
                }
            }
        );

        if (persist) {
            loadFromNT();
        }
    }

    public void loadFromNT() {
        if (subs.isEmpty()) {
            return;
        }
        kP = subs.get("kP").get();
        kI = subs.get("kI").get();
        kD = subs.get("kD").get();
        kS = subs.get("kS").get();
        kG = subs.get("kG").get();
        kV = subs.get("kV").get();
        kA = subs.get("kA").get();
        kiZone = subs.get("kiZone").get();
        kiError = subs.get("kiError").get();
        kVelMax = subs.get("kVelMax").get();
        kAccelMax = subs.get("kAccelMax").get();
    }

    public void pushToNT() {
        if (subs.isEmpty()) {
            return;
        }
        subs.get("kP").set(kP);
        subs.get("kI").set(kI);
        subs.get("kD").set(kD);
        subs.get("kS").set(kS);
        subs.get("kG").set(kG);
        subs.get("kV").set(kV);
        subs.get("kA").set(kA);
        subs.get("kiZone").set(kiZone);
        subs.get("kiError").set(kiError);
        subs.get("kVelMax").set(kVelMax);
        subs.get("kAccelMax").set(kAccelMax);
    }

    public void onChange(Runnable funcToRun) {
        onChange = funcToRun;
    }
}
