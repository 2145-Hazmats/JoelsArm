// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem2. */
  public ArmSubsystem() {
    // reset encoder to 0 when code is deployed. Not when the robot is enabled/disabled btw
    m_RelativeEncoder.setPosition(0.0);
    // Approximately correct conversion factors
    m_RelativeEncoder.setPositionConversionFactor(1.45);
    m_RelativeEncoder.setVelocityConversionFactor(1.45);
    // Put informational variables we don't change on SmartDashboard
    SmartDashboard.putNumber(("CountsPerRevolution"), m_RelativeEncoder.getCountsPerRevolution());
    SmartDashboard.putNumber(("MeasurementPeriod"), m_RelativeEncoder.getMeasurementPeriod());
    SmartDashboard.putBoolean(("Inverted"), m_RelativeEncoder.getInverted());
  }

  private final CANSparkMax m_Arm = new CANSparkMax(4, MotorType.kBrushless);
  private final RelativeEncoder m_RelativeEncoder = m_Arm.getEncoder();

  // Runs the arm manually. Positive speed is clockwise
  public void ArmTurnMethod(double speed) {
    m_Arm.set(speed*Constants.ArmConstants.ManualSpeed);
    SmartDashboard.putNumber(("Position"), m_RelativeEncoder.getPosition());
  }

  // Command for ArmTurnMethod
  public Command ArmTurnCommand(double speed) {
    return new StartEndCommand(()->this.ArmTurnMethod(speed), ()->this.ArmTurnMethod(0.0), this);
  }

  // Turn the arm to a specified angle. Slows down based on the current arm angle
  public void ArmTurnToAngle(double angle) {
    // Stop if angle is close enough
    if (Math.abs(m_RelativeEncoder.getPosition() - angle) <= Constants.ArmConstants.DegreeOfError) {
      m_Arm.stopMotor();
    }
    // Spin counter-clockwise
    else if ((m_RelativeEncoder.getPosition() - angle) > 0) {
      m_Arm.set(-Constants.ArmConstants.TurnToSpeed*
      Math.min(1.0, Math.max((m_RelativeEncoder.getPosition() - angle)/Constants.ArmConstants.SlowMultiplier, 0.0)));
    }
    // Spin clockwise
    else if ((m_RelativeEncoder.getPosition() - angle) < 0) {
      m_Arm.set(Constants.ArmConstants.TurnToSpeed*
      Math.min(1.0, Math.max((angle - m_RelativeEncoder.getPosition())/Constants.ArmConstants.SlowMultiplier, 0.0)));
    }
    // Display SmartDashboard
    SmartDashboard.putNumber(("Position:"), m_RelativeEncoder.getPosition());
    SmartDashboard.putNumber(("Goal:"), angle);
    SmartDashboard.putNumber(("Speed:"), Constants.ArmConstants.TurnToSpeed*Math.min(1.0, Math.max((angle - m_RelativeEncoder.getPosition())/Constants.ArmConstants.SlowMultiplier, 0.0))*100);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
