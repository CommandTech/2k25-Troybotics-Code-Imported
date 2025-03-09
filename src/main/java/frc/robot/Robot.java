// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    FollowPathCommand.warmupCommand().schedule();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //Shows the subsystems on the SmartDashboard for testing
    SmartDashboard.putData(RobotContainer.getInstance().getDrivetrain());
    SmartDashboard.putData(RobotContainer.getInstance().getArm());
    SmartDashboard.putData(RobotContainer.getInstance().getAlgae());
    SmartDashboard.putData(RobotContainer.getInstance().getCoral());
    SmartDashboard.putData(RobotContainer.getInstance().getClimber());
    
    //PID tuning
    SmartDashboard.putNumber("Drive P",Constants.DriveConstants.DRIVE_P);
    SmartDashboard.putNumber("Drive I",Constants.DriveConstants.DRIVE_I);
    SmartDashboard.putNumber("Drive D",Constants.DriveConstants.DRIVE_D);

    SmartDashboard.putNumber("Drive S",Constants.DriveConstants.DRIVE_S);
    SmartDashboard.putNumber("Drive V",Constants.DriveConstants.DRIVE_V);
    
    SmartDashboard.putData("DriveToStation Command", RobotContainer.getInstance().getDrivetrain().DriveToStation());
    SmartDashboard.putData("Tuning Path Command", RobotContainer.getInstance().getDrivetrain().followPathCommand("TuningAuto"));


    SmartDashboard.putNumber("Arm P",Constants.ArmConstants.ARM_P);
    SmartDashboard.putNumber("Arm I",Constants.ArmConstants.ARM_I);
    SmartDashboard.putNumber("Arm D",Constants.ArmConstants.ARM_D);

    SmartDashboard.putNumber("Arm S",Constants.ArmConstants.ARM_S);
    SmartDashboard.putNumber("Arm V",Constants.ArmConstants.ARM_V);
    SmartDashboard.putNumber("Arm A",Constants.ArmConstants.ARM_A);
    SmartDashboard.putNumber("Arm G",Constants.ArmConstants.ARM_G);

    SmartDashboard.putData("Arm to L3 Command", RobotContainer.getInstance().getArm().setHeight(Constants.ArmConstants.L3_HEIGHT));
    SmartDashboard.putData("Arm to L2 Command", RobotContainer.getInstance().getArm().setHeight(Constants.ArmConstants.L2_HEIGHT));
    SmartDashboard.putData("Arm to L1 Command", RobotContainer.getInstance().getArm().setHeight(Constants.ArmConstants.L1_HEIGHT));
    SmartDashboard.putData("Arm to Stow Command", RobotContainer.getInstance().getArm().setHeight(Constants.ArmConstants.STOW_HEIGHT));
    
    SmartDashboard.putNumber("Climber P",Constants.ClimberConstants.CLIMBER_P);
    SmartDashboard.putNumber("Climber I",Constants.ClimberConstants.CLIMBER_I);
    SmartDashboard.putNumber("Climber D",Constants.ClimberConstants.CLIMBER_D);

    SmartDashboard.putNumber("Climber S",Constants.ClimberConstants.CLIMBER_S);
    SmartDashboard.putNumber("Climber V",Constants.ClimberConstants.CLIMBER_V);
    SmartDashboard.putNumber("Climber A",Constants.ClimberConstants.CLIMBER_A);
    SmartDashboard.putNumber("Climber G",Constants.ClimberConstants.CLIMBER_G);

    SmartDashboard.putData("Climber Deploy", RobotContainer.getInstance().getClimber().setHeight(Constants.ClimberConstants.CLIMBER_READY_ANGLE));
    SmartDashboard.putData("Climber Retract", RobotContainer.getInstance().getArm().setHeight(Constants.ClimberConstants.CLIMBER_CLIMBED_ANGLE));
    SmartDashboard.putData("Climber Toggle", RobotContainer.getInstance().getClimber().toggleClimber());
    
    SmartDashboard.putData("Deliver Coral", RobotContainer.getInstance().getCoral().deliverCoral());
    SmartDashboard.putData("Intake Coral", RobotContainer.getInstance().getCoral().intakeCoral());
    SmartDashboard.putData("Stop Coral Intake", RobotContainer.getInstance().getCoral().stopIntake());
    
    SmartDashboard.putData("Deliver Algae", RobotContainer.getInstance().getAlgae().deliverAlgae());
    SmartDashboard.putData("Intake Algae", RobotContainer.getInstance().getAlgae().intakeAlgae());
    SmartDashboard.putData("Stop Algae Intake", RobotContainer.getInstance().getAlgae().stopIntake());
    
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
