// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class ArmSubsystem extends ProfiledPIDSubsystem {
  /** Creates a new ArmSubsystem2. */
  public ArmSubsystem() {
    super(
      // The ProfiledPIDController used by the subsystem
      new ProfiledPIDController(
        0.1,
        0,
        0,
        // The motion profile constraints
        new TrapezoidProfile.Constraints(0, 0)));
  }

  // Declare
  //private final CANSparkMax m_Arm = new CANSparkMax(Constants.ArmConstants.kArmMotorID, MotorType.kBrushed);
  //private final TalonSRX m_ArmEncoder = new TalonSRX(Constants.ArmConstants.kArmEncoderID);

  // Wrist motor to test
  private final WPI_TalonSRX m_Arm = new WPI_TalonSRX(13);
  private final Encoder m_ArmEncoder = new Encoder(0, 1,true); 

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    m_Arm.set(output*Constants.ArmConstants.ArmSpeed);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_ArmEncoder.getDistance();
  }

  public void TurnMotor(double speed) {
    m_Arm.set(speed*Constants.ArmConstants.ArmSpeed);
  }
  
}