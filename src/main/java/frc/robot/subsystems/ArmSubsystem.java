// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class ArmSubsystem extends ProfiledPIDSubsystem {
  /** Creates a new ArmSubsystem2. */
  public ArmSubsystem() {
    super(
      // The ProfiledPIDController used by the subsystem
      new ProfiledPIDController(
        Constants.ArmConstants.ArmkP,
        0.0,
        0.0,
        // The motion profile constraints
        new TrapezoidProfile.Constraints(Constants.ArmConstants.MaxArmVelocity, Constants.ArmConstants.MaxArmAcceleration)));
    m_Arm.setSelectedSensorPosition(0.0);
  }

  // Declare
  private final WPI_TalonSRX m_Arm = new WPI_TalonSRX(Constants.ArmConstants.kArmMotorID);

  // Wrist motor to test
  //private final WPI_TalonSRX m_Arm = new WPI_TalonSRX(13);
  //private final Encoder m_ArmEncoder = new Encoder(0, 1,true); 

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    m_Arm.set(output*Constants.ArmConstants.ArmSpeed);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    SmartDashboard.putNumber(("Arm Position"), -m_Arm.getSelectedSensorPosition());
    //return m_ArmEncoder.getDistance();
    return -m_Arm.getSelectedSensorPosition();
  }

}