// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  private static RobotContainer m_robotContainer = new RobotContainer();

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drive = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final Coral m_coral = new Coral();
  private final Algae m_algae = new Algae();
  private final Climber m_climber = new Climber();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_PORT);

  private final CommandXboxController m_manipController =
      new CommandXboxController(OperatorConstants.MANIP_PORT);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putNumber("Drive P",Constants.DriveConstants.DRIVE_P);
    SmartDashboard.putNumber("Drive I",Constants.DriveConstants.DRIVE_I);
    SmartDashboard.putNumber("Drive D",Constants.DriveConstants.DRIVE_D);

    SmartDashboard.putNumber("Arm P",Constants.ArmConstants.ARM_P);
    SmartDashboard.putNumber("Arm I",Constants.ArmConstants.ARM_I);
    SmartDashboard.putNumber("Arm D",Constants.ArmConstants.ARM_D);

    // m_drive.setDefaultCommand(new Drive(m_drive));
    m_arm.setDefaultCommand(m_arm.idleCommand());
    m_drive.setDefaultCommand(m_drive.driveCommand(getDriver().getLeftY(),getDriver().getRightX()));
    m_coral.setDefaultCommand(m_coral.intakeCoral());
    m_algae.setDefaultCommand(m_algae.intakeAlgae());
    m_climber.setDefaultCommand(m_climber.idleCommand());
    
    NamedCommands.registerCommand("deliverCoral", m_coral.deliverCoral());

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Leave", m_drive.followPathCommand("Leave"));
    autoChooser.addOption("Auto RP", m_drive.followPathCommand("Auto RP"));
    autoChooser.addOption("2 Coral", m_drive.followPathCommand("2 Coral"));
    autoChooser.addOption("Tuning Auto", m_drive.followPathCommand("Tuning_Auto"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    m_driverController.leftBumper().onTrue(m_drive.DriveToStation());

    m_manipController.povUp().onTrue(m_arm.setHeight(Constants.ArmConstants.L3_HEIGHT));
    m_manipController.povRight().onTrue(m_arm.setHeight(Constants.ArmConstants.L2_HEIGHT));
    m_manipController.povDown().onTrue(m_arm.setHeight(Constants.ArmConstants.L1_HEIGHT));
    m_manipController.povLeft().onTrue(m_arm.setHeight(Constants.ArmConstants.STOW_HEIGHT));

    m_manipController.a().whileTrue(m_coral.deliverCoral());
    m_manipController.b().whileTrue(m_algae.deliverAlgae());
    
    m_manipController.x().onTrue(m_climber.toggleClimber());
  }
  public CommandXboxController getDriver() {
    return m_driverController;
  }

  public CommandXboxController getManipulator() {
    return m_manipController;
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
