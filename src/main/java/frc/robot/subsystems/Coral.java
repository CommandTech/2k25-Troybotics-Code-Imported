// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Coral extends SubsystemBase {
  private SparkMax coralMotor;
  private SparkMaxConfig config;
  private boolean hasCoral = false;
  /** Creates a new Coral. */
  public Coral() {
      //creates coral motor
      coralMotor = new SparkMax(Constants.MotorConstants.CORAL_MOTOR_ID, SparkMax.MotorType.kBrushless);
      //creates the config for the motor
      config = new SparkMaxConfig();
      config.inverted(Constants.MotorConstants.CORAL_MOTOR_INVERTED);
      config.smartCurrentLimit(Constants.MotorConstants.CORAL_MOTOR_AMP_LIMIT);
      config.idleMode(IdleMode.kBrake);

      //sets the configuration to the motor
      coralMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

      SmartDashboard.putBoolean("hasCoral", hasCoral);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //1 to deliver 0 to stop -1 to intake
  public void setMotors(double value) {
    //sets the motor speed to the inputed value
    coralMotor.set(value);
  }

  public Command deliverCoral() {
    // Deliver the coral
    setMotors(1);
    return runOnce(() -> hasCoral = false);
  }
  public Command intakeCoral() {
    double current = coralMotor.getOutputCurrent();
    // Intake the coral
    //stops the motor if the current spikes up because the robot has a coral
    if (current < Constants.CoralConstants.INTAKED_CORAL_AMPS) {
      return runOnce(() -> setMotors(-1));
    }
    hasCoral = true;
    return stopIntake();
  }
  public Command stopIntake(){
    return runOnce(() -> setMotors(0));
  }
}
