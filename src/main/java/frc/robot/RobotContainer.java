// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public ArmSubsystem m_ArmSubsystem = new ArmSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // PIDs set at specific angles
    /*
    m_driverController.a().onTrue(Commands.runOnce(() -> {
      m_ArmSubsystem.setGoal(0.0);
      m_ArmSubsystem.enable();
    }, m_ArmSubsystem));
    m_driverController.b().onTrue(Commands.runOnce(() -> {
      m_ArmSubsystem.setGoal(30.0);
      m_ArmSubsystem.enable();
    }, m_ArmSubsystem));
    m_driverController.x().onTrue(Commands.runOnce(() -> {
      m_ArmSubsystem.setGoal(-60.0);
      m_ArmSubsystem.enable();
    }, m_ArmSubsystem));
    m_driverController.y().onTrue(Commands.runOnce(() -> {
      m_ArmSubsystem.setGoal(60.0);
      m_ArmSubsystem.enable();
    }, m_ArmSubsystem));
    // Disable PID button
    m_driverController.back().onTrue(Commands.runOnce(() -> {
      m_ArmSubsystem.disable();
    }, m_ArmSubsystem));
    */

    m_driverController.a().onTrue(
      Commands.run(() -> m_ArmSubsystem.ArmTurnToAngle(0.0), m_ArmSubsystem)
    );
    m_driverController.b().onTrue(
      Commands.run(() -> m_ArmSubsystem.ArmTurnToAngle(90.0), m_ArmSubsystem)
    );
    m_driverController.x().onTrue(
      Commands.run(() -> m_ArmSubsystem.ArmTurnToAngle(-90.0), m_ArmSubsystem)
    );
    m_driverController.y().onTrue(
      Commands.run(() -> m_ArmSubsystem.ArmTurnToAngle(120.0), m_ArmSubsystem)
    );

    m_driverController.leftBumper().whileTrue(m_ArmSubsystem.ArmTurnCommand(-0.5));
    m_driverController.rightBumper().whileTrue(m_ArmSubsystem.ArmTurnCommand(0.5));

  }
  
}
