package frc.robot.motorcontrol;

import java.text.MessageFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXMotor extends MotorControl {
    TalonFX motor;
    /** Current TalonFX configuration. */
    private TalonFXConfiguration configuration;
    /** Current TalonFX Configurator. */
    private TalonFXConfigurator cfg;

    StatusSignal<Temperature> motorTemp;
    StatusSignal<AngularVelocity> motorRpm;
    StatusSignal<AngularVelocity> motorVelocity;
    StatusSignal<Voltage> motorVoltage;
    StatusSignal<Current> supplyCurrent;
    StatusSignal<Voltage> supplyVoltage;
    StatusSignal<Angle> motorPosition;
    StatusSignal<Angle> motorRawPosition;
    StatusSignal<Current> statorCurrent;

    List<StatusSignal<Boolean>> motorFaults;
    List<StatusSignal<Boolean>> stickyFaults;

    private MotionMagicVoltage positionSetter;
    private MotionMagicVelocityVoltage velocitySetter;
    private VelocityVoltage rpmSetter;
    
    double scaleFactor = 1;
    boolean allowConfig;

    DoublePublisher statorPub;

    public TalonFXMotor(String motorName, int id) {
        this(motorName, new TalonFX(id), false);
    }

    public TalonFXMotor(String motorName, int id, String canbus) {
        this(motorName, new TalonFX(id, canbus), false);
    }

    public TalonFXMotor(String motorName, TalonFX motor, boolean logOnly) {
        super(motorName, logOnly, false);
        this.motor = motor;
        
        configuration = new TalonFXConfiguration();
        cfg = motor.getConfigurator();
        positionSetter = new MotionMagicVoltage(0).withUpdateFreqHz(0);
        velocitySetter = new MotionMagicVelocityVoltage(0).withUpdateFreqHz(0);
        rpmSetter = new VelocityVoltage(0).withUpdateFreqHz(0).withSlot(1);

        //input data signals
        motorTemp = motor.getDeviceTemp();
        motorRpm = motor.getRotorVelocity();
        motorVoltage = motor.getMotorVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        supplyVoltage = motor.getSupplyVoltage();
        motorPosition = motor.getPosition();
        motorVelocity = motor.getVelocity();
        motorRawPosition = motor.getRotorPosition();
        statorCurrent = motor.getStatorCurrent();

        //configure all signals for our rate (we pick 18 milliseconds to always have fresh data for 20ms loops)
        BaseStatusSignal.setUpdateFrequencyForAll(1./0.018, motorTemp, motorRpm, motorVoltage, supplyCurrent, 
            supplyVoltage, motorPosition, motorVelocity, motorRawPosition, statorCurrent);

        motorFaults = Arrays.asList(
            motor.getFault_Hardware(),
            motor.getFault_ProcTemp(),
            motor.getFault_DeviceTemp(),
            motor.getFault_Undervoltage(),
            motor.getFault_BootDuringEnable(),
            motor.getFault_UnlicensedFeatureInUse(),
            motor.getFault_BridgeBrownout(),
            motor.getFault_RemoteSensorReset(),
            motor.getFault_MissingDifferentialFX(),
            motor.getFault_RemoteSensorPosOverflow(),
            motor.getFault_OverSupplyV(),
            motor.getFault_UnstableSupplyV(),
            motor.getFault_ReverseHardLimit(),
            motor.getFault_ForwardHardLimit(),
            motor.getFault_ReverseSoftLimit(),
            motor.getFault_ForwardSoftLimit(),
            motor.getFault_MissingSoftLimitRemote(),
            motor.getFault_MissingHardLimitRemote(),
            motor.getFault_RemoteSensorDataInvalid(),
            motor.getFault_FusedSensorOutOfSync(),
            motor.getFault_StatorCurrLimit(),
            motor.getFault_SupplyCurrLimit(),
            motor.getFault_UsingFusedCANcoderWhileUnlicensed(),
            motor.getFault_StaticBrakeDisabled()
        );

        stickyFaults = Arrays.asList(
            motor.getStickyFault_Hardware(),
            motor.getStickyFault_ProcTemp(),
            motor.getStickyFault_DeviceTemp(),
            motor.getStickyFault_Undervoltage(),
            motor.getStickyFault_BootDuringEnable(),
            motor.getStickyFault_UnlicensedFeatureInUse(),
            motor.getStickyFault_BridgeBrownout(),
            motor.getStickyFault_RemoteSensorReset(),
            motor.getStickyFault_MissingDifferentialFX(),
            motor.getStickyFault_RemoteSensorPosOverflow(),
            motor.getStickyFault_OverSupplyV(),
            motor.getStickyFault_UnstableSupplyV(),
            motor.getStickyFault_ReverseHardLimit(),
            motor.getStickyFault_ForwardHardLimit(),
            motor.getStickyFault_ReverseSoftLimit(),
            motor.getStickyFault_ForwardSoftLimit(),
            motor.getStickyFault_MissingSoftLimitRemote(),
            motor.getStickyFault_MissingHardLimitRemote(),
            motor.getStickyFault_RemoteSensorDataInvalid(),
            motor.getStickyFault_FusedSensorOutOfSync(),
            motor.getStickyFault_StatorCurrLimit(),
            motor.getStickyFault_SupplyCurrLimit(),
            motor.getStickyFault_UsingFusedCANcoderWhileUnlicensed(),
            motor.getStickyFault_StaticBrakeDisabled()
        );

        for (StatusSignal<Boolean> statusSignal : motorFaults) {
            statusSignal.setUpdateFrequency(4);
        }
        for (StatusSignal<Boolean> statusSignal : stickyFaults) {
            statusSignal.setUpdateFrequency(4);
        }
        
        motor.optimizeBusUtilization();
        statorPub = motorTable.getDoubleTopic("Stator Current").publish();
        
        //load data from the motor
        var statusCode = cfg.refresh(configuration);
        scaleFactor = configuration.Feedback.SensorToMechanismRatio;
        pidConstants.kP = configuration.Slot0.kP;
        pidConstants.kI = configuration.Slot0.kI;
        pidConstants.kD = configuration.Slot0.kD;
        pidConstants.kV = configuration.Slot0.kV;
        pidConstants.kA = configuration.Slot0.kA;
        pidConstants.kS = configuration.Slot0.kS;
        pidConstants.kG = configuration.Slot0.kG;
        pidConstants.kVelMax = configuration.MotionMagic.MotionMagicCruiseVelocity;
        pidConstants.kAccelMax = configuration.MotionMagic.MotionMagicAcceleration;
        pidConstants.pushToNT();

        //set RPM constants
        configuration.Slot1.kP = 0.3;
        configuration.Slot1.kI = 0.2;
        configuration.Slot1.kD = 0;
        configuration.Slot1.kV = 0.0035;
        configuration.Slot1.kA = 0;
        configuration.Slot1.kS = 0;
        configuration.Slot1.kG = 0;
        if(allowConfig) {
            statusCode = cfg.apply(configuration.Slot1);
        }
    }

    @Override
    public void configurePid() {
        var statusCode = cfg.refresh(configuration);
        
        configuration.Slot0.kP = pidConstants.kP;
        configuration.Slot0.kI = pidConstants.kI;
        configuration.Slot0.kD = pidConstants.kD;
        configuration.Slot0.kV = pidConstants.kV;
        configuration.Slot0.kA = pidConstants.kA;
        configuration.Slot0.kS = pidConstants.kS;
        configuration.Slot0.kG = pidConstants.kG;
        configuration.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        //removed as part of phoenix6: iZone, iError

        configuration.MotionMagic
            .withMotionMagicCruiseVelocity(pidConstants.kVelMax)
            .withMotionMagicAcceleration(pidConstants.kAccelMax);
        //implement in future? withMotionMagicExpo_kV, kA
        if(allowConfig) {
            statusCode = cfg.apply(configuration);
        }
    }

    @Override
    public void setBrakeMode(boolean brakeOn) {
        var statusCode = motor.setNeutralMode(brakeOn ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setPower(double percent) {
        motor.set(percent);
        atSetpoint = false;
    }

    @Override
    public void stopMotor(boolean coast) {
        if (coast) {
            motor.setControl(new CoastOut());
        } else {
            motor.stopMotor();
        }
    }

    /**
     * CTRE returns positions as motor revolutions, this lets us turn motor revolutions into something useful.
     */
    @Override
    public void setScaleFactor(double factor) {
        var statusCode = cfg.refresh(configuration);
        configuration.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
            .withSensorToMechanismRatio(factor);
        configuration.MotionMagic
            .withMotionMagicCruiseVelocity(pidConstants.kVelMax)
            .withMotionMagicAcceleration(pidConstants.kAccelMax);
        if(allowConfig) {
            statusCode = cfg.apply(configuration);
        }
        scaleFactor = factor;
    }

    @Override
    public String[] getFaultArray() {
        return handleFaults(motorFaults);
    }

    @Override
    public String[] getStickyFaultArray() {
        return handleFaults(stickyFaults);
    }

    private String[] handleFaults(List<StatusSignal<Boolean>> faultList) {
        ArrayList<String> faults = new ArrayList<String>();

        for (var fault : faultList) {
            Boolean isFaulted = fault.refresh().getValue();
            var statusCode = fault.getStatus();
            if (isFaulted) {
                var name = fault.getName();
                //remove prefixes for shorter fault names (like "Fault_")
                var index = name.indexOf('_');
                if (index > 0) {
                    name = name.substring(index+1);
                }
                faults.add(name);
            }
        }

        return faults.toArray(new String[0]);
    }

    @Override
    public String getVersion() {
        //not using status code checks here, maybe implement later?
        Integer major = motor.getVersionMajor().getValue();
        Integer minor = motor.getVersionMinor().getValue();
        Integer build = motor.getVersionBuild().getValue();
        Integer bugfix = motor.getVersionBugfix().getValue();
        return MessageFormat.format("{0}.{1}.{2}.{3}", major, minor, bugfix, build);
    }

    @Override
    public double getSupplyVoltage() {
        var statusCode = supplyVoltage.refresh().getStatus();
        return supplyVoltage.getValueAsDouble();
    }

    @Override
    public double getSupplyCurrent() {
        var statusCode = supplyCurrent.refresh().getStatus();
        return supplyCurrent.getValueAsDouble();
    }

    @Override
    public double getMotorVoltage() {
        var statusCode = motorVoltage.refresh().getStatus();
        return motorVoltage.getValueAsDouble();
    }

    @Override
    public double getMotorPosition() {
        var statusCode = motorRawPosition.refresh().getStatus();
        return motorRawPosition.getValueAsDouble();
    }

    @Override
    public double getMotorVelocity() {
        var statusCode = motorRpm.refresh().getStatus();
        return motorRpm.getValueAsDouble();
    }

    @Override
    public double getRpm() {
        var statusCode = motorRpm.refresh().getStatus();
        //internally the motor runs in rev/sec, but return an WpiLib unit type.  We are using WpiLib to scale it for us.
        return motorRpm.getValue().in(Units.RPM);
    }

    @Override
    public void setCurrentLimit(double limit) {
        var statusCode = cfg.refresh(configuration.CurrentLimits);
        if(allowConfig) {
            statusCode = cfg.apply(
                configuration.CurrentLimits.withSupplyCurrentLimit(limit)
                    .withSupplyCurrentLimitEnable(true));
        }
    }

    /**
     * Note, this uses a different PID controller versus the position/velocity controller
     * @param rpm Rpm to command the motor with
     */
    @Override
    public void setRpm(double rpm) {
        var statusCode = motor.setControl(rpmSetter.withVelocity(rpm / 60));
        //check to see if RPM is within 5% of the requested RPM
        atSetpoint = Math.abs(getRpm() - rpm) < (rpm * 0.05);
    }

    @Override
    public void setVelocity(double velocity) {
        var statusCode = motor.setControl(velocitySetter.withVelocity(velocity).withAcceleration(pidConstants.kAccelMax));
        //atSetpoint
    }

    @Override
    public void setPosition(double position) {
        var statusCode = motor.setControl(positionSetter.withPosition(position));
        //atSetpoint
    }

    @Override
    public void setEncoderPosition(double position) {
        var statusCode = cfg.setPosition(Angle.ofBaseUnits(position * 2 * Math.PI, Units.Rotations).in(Units.Rotations));
    }

    /** Returns motor temperature in C */
    @Override
    public double getMotorTemp() {
        var statusCode = motorTemp.refresh().getStatus();
        return motorTemp.getValueAsDouble();
    }

    @Override
    public void clearStickyFaults() {
        var statusCode = motor.clearStickyFaults();
    }

    @Override
    public boolean isConnected() {
        //the signal to update doesn't matter here, just checking the status returned
        var statusCode = motorTemp.refresh().getStatus();
        return statusCode != StatusCode.EcuIsNotPresent && statusCode != StatusCode.RxTimeout && statusCode != StatusCode.CanMessageStale;
    }

    @Override
    public void setInverted(boolean isInverted) {
        var statusCode = cfg.refresh(configuration.MotorOutput);
        configuration.MotorOutput.withInverted(
            isInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive);
        if(allowConfig) {
            statusCode = cfg.apply(configuration.MotorOutput);
        }
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
        atSetpoint = false;
    }

    /** Returns motor of type TalonFX */
    @Override
    public Object getBaseMotor() {
        return motor;
    }

    @Override
    public double getPosition() {
        var statusCode = motorPosition.refresh().getStatus();
        return motorPosition.getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        var statusCode = motorVelocity.refresh().getStatus();
        return motorVelocity.getValueAsDouble();
    }

    /****************************************************************************************/
    /* Unique Motor Commands */
    public double getStatorCurrent() {
        var statusCode = statorCurrent.refresh().getStatus();
        return statorCurrent.getValueAsDouble();
    }

    public void setStatorCurrentLimit(double limit) {
        var statusCode = cfg.refresh(configuration.CurrentLimits);
        if(allowConfig) {
            statusCode = cfg.apply(
                configuration.CurrentLimits.withStatorCurrentLimit(limit)
                    .withStatorCurrentLimitEnable(true));
        }
    }

    @Override
    protected void customLogging() {
        statorPub.set(getStatorCurrent());
    }
}
