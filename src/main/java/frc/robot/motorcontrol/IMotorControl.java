package frc.robot.motorcontrol;

public interface IMotorControl {
    /**
     * Turn on brake mode on the motor
     * @param brakeOn If true, set the motor to brake mode, false is coast mode
     */
    void setBrakeMode(boolean brakeOn);
    /**
     * Set the motor to a speed equal to a percent of battery voltage
     * @param percent Bounded from -1 to 1 (aka -100% to 100%)
     */
    void setPower(double percent);
    /**
     * Set the motor to a specific voltage.  This is prefered for smoother control through battery issues
     * @param voltage The voltage to command the motor with
     */
    void setVoltage(double voltage);
    void stopMotor(boolean coast);
    void setScaleFactor(double factor);
    void setInverted(boolean isInverted);
    /**
     * Get a space delimeted string of all the active faults on the motor.
     * @return a space delimeted string of all the active faults on the motor
     */
    String getFaults();
    String[] getFaultArray();
    String getStickyFaults();
    String[] getStickyFaultArray();
    /**
     * Get the firmware version of the motor
     * @return The firmware version of the motor
     */
    String getVersion();
    double getSupplyVoltage();
    double getSupplyCurrent();
    double getMotorVoltage();
    /**
     * Returns the scaled motor position using setScaleFactor()
     * @return The scaled motor position
     */
    double getPosition();
    /**
     * Returns the scaled motor velocity using setScaleFactor()
     * @return The scaled motor velocity
     */
    double getVelocity();
    /**
     * Returns the raw motor position using the native units of the motor
     * @return The raw motor position
     */
    double getMotorPosition();
    /**
     * Returns the raw motor velocity using the native units of the motor
     * @return The raw motor velocity
     */
    double getMotorVelocity();
    double getRpm();
    void setCurrentLimit(double limit);
    /**
     * Set the motor to the specified RPM.  This ignores the Velocity and Accel Max constants from PID constants.
     * @param rpm
     */
    void setRpm(double rpm);
    void setVelocity(double velocity);
    /**
     * Puts the motor in the specified position
     * @param position Position to move the motor to
     */
    void setPosition(double position);
    /** 
     * Sets the internal motor encoder position
     * @param position The position you want the encoder in
     */
    void setEncoderPosition(double position);
    /**
     * Gets the motor's temperature
     * @return The temperature of the motor in C
     */
    double getMotorTemp();
    void clearStickyFaults();
    boolean isConnected();
    boolean isFaulted();
    /**
     * Get the base motor if you want to use the vendor specific APIs that aren't implemented
     * @return An object to cast to the correct motor type (depends on what motor this interface implemented)
     */
    Object getBaseMotor();
    boolean atSetpoint();
    /**
     * This function only needs to be called if the pidConstants variable is changed.  If changed by
     * NetworkTables, it will autoupdate the PID values.
     */
    void configurePid();

    String getName();
    /* soft limits, hard limits, elevator control, arm control, ramp rate, angle position control (with rollover), output min/max, pid min/max, follower */
}
