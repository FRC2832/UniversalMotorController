// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.motorcontrol.MotorControls;

public class RobotContainer {
  XboxController controller;
  MotorTestSubsystem sub;

  public RobotContainer() {
    controller = new XboxController(0);
    sub = new MotorTestSubsystem();

    configureBindings();
    var test = new SendableChooser<String>();
    test.setDefaultOption("none", "none");
    test.addOption("auto1", "auto1");
    SmartDashboard.putData(test);

    SmartDashboard.putData("Stop Motor", sub.stopMotor());
    SmartDashboard.putData("Set Coast", sub.setCoast());
    SmartDashboard.putData("Set Brake", sub.setBrake());
    SmartDashboard.putData("Set RPM", sub.setRpm());
    SmartDashboard.putData("Set Encoder 5", sub.setEncoderPosition(5));
    SmartDashboard.putData("Set Power", sub.setPower(controller::getLeftY));
    SmartDashboard.putData("Set Voltage", sub.setVoltage(controller::getRightY));
    SmartDashboard.putData("Set Position -10", sub.setPosition(-10));
    SmartDashboard.putData("Set Position 10", sub.setPosition(10).until(sub.atSetpoint()).finallyDo(sub::stop));
    SmartDashboard.putData("Set Velocity 1", sub.setVelocity(1));

    SmartDashboard.putData("Clear Sticky Faults", new InstantCommand(MotorControls::ClearStickyFaults).ignoringDisable(true));

    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(sub);

    
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
