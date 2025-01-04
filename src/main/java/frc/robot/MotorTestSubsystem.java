package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.motorcontrol.IMotorControl;
import frc.robot.motorcontrol.SparkFlexMotor;
import frc.robot.motorcontrol.SparkMaxMotor;
import frc.robot.motorcontrol.TalonFXMotor;

public class MotorTestSubsystem extends SubsystemBase {
    IMotorControl motor;

    public MotorTestSubsystem() {
        //motor = new TalonFXMotor("TalonFX", 34);
        motor = new SparkMaxMotor("SparkMax", 3);
        motor.setBrakeMode(false);
        motor.setScaleFactor(10);
        motor.setEncoderPosition(5);
        //low current limit needed with bench supply
        motor.setCurrentLimit(5);
        
        if (motor instanceof TalonFXMotor) {
            var talon = (TalonFXMotor)motor;
            talon.pidConstants.kP = 0.3;
            talon.pidConstants.kI = 0.2;
            talon.pidConstants.kD = 0.1;

            talon.setStatorCurrentLimit(20);
        }
        motor.configurePid();

        //To implement
        //setInverted
        //void setVelocity(double velocity);
        //Object getBaseMotor();
    }

    @Override
    public void periodic() {

    }

    public void stop() {
        motor.stopMotor(true);
    }

    public Command stopMotor() {
        return runOnce(this::stop);
    }

    public Command clearStickyFaults() {
        return runOnce(motor::clearStickyFaults).ignoringDisable(true);
    }

    public Command setCoast() {
        return runOnce(() -> motor.setBrakeMode(false));
    }

    public Command setBrake() {
        return runOnce(() -> motor.setBrakeMode(true));
    }

    public Command setPower(DoubleSupplier power) {
        return run(() -> motor.setPower(power.getAsDouble()));
    }

    public Command setVoltage(DoubleSupplier voltage) {
        return run(() -> motor.setVoltage(voltage.getAsDouble()));
    }

    public Command setEncoderPosition(double position) {
        return runOnce(() -> motor.setEncoderPosition(position)).ignoringDisable(true);
    }

    public Command setPosition(double position) {
        return run(() -> motor.setPosition(position));
    }

    public Trigger atSetpoint() {
        return new Trigger(motor::atSetpoint);
    }

    public Command setVelocity(double velocity) {
        return run(() -> motor.setVelocity(velocity));
    }

    public Command setRpm() {
        var topic = NetworkTableInstance.getDefault().getDoubleTopic("/SmartDashboard/Target RPM");
        var entry = topic.getEntry(0);
        entry.set(0);
        return setRpm(entry);
    }

    public Command setRpm(DoubleSupplier rpm) {
        return run(() -> motor.setRpm(rpm.getAsDouble()));
    }
}
