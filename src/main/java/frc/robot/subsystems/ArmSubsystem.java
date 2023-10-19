// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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
    m_Arm1.setSelectedSensorPosition(0.0);
  }

  // Declare
  private final WPI_TalonSRX m_Arm1 = new WPI_TalonSRX(Constants.ArmConstants.kArmMotor1ID);
  private final WPI_TalonSRX m_Arm2 = new WPI_TalonSRX(Constants.ArmConstants.kArmMotor2ID);
  private final MotorControllerGroup m_ArmGroup = new MotorControllerGroup(m_Arm1, m_Arm2);

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    SmartDashboard.putNumber(("Arm PID output"), output*Constants.ArmConstants.ArmSpeed);
    m_ArmGroup.set(output*Constants.ArmConstants.ArmSpeed);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    SmartDashboard.putNumber(("Arm getMeasurement"), m_Arm1.getSelectedSensorPosition()/(Constants.ArmConstants.PulsesPerDegree));
    return (m_Arm1.getSelectedSensorPosition()/(Constants.ArmConstants.PulsesPerDegree));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(("Arm Position"), m_Arm1.getSelectedSensorPosition()/(Constants.ArmConstants.PulsesPerDegree));
  }

  public void ArmTurnMethod(double speed) {
    m_ArmGroup.set(speed*0.25);
    SmartDashboard.putNumber(("Arm manual speed"), speed*0.25);
  }

}