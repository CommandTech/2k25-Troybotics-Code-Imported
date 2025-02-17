// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Algae extends SubsystemBase {
  private SparkMax algaeMotor;
  private SparkMaxConfig config;
  /** Creates a new Algae. */
  public Algae() {
      algaeMotor = new SparkMax(Constants.MotorConstants.ALGAE_MOTOR_ID, SparkMax.MotorType.kBrushless);
      config = new SparkMaxConfig();
      config.inverted(Constants.MotorConstants.ALGAE_MOTOR_INVERTED);
      config.smartCurrentLimit(Constants.MotorConstants.ALGAE_MOTOR_AMP_LIMIT);
      config.idleMode(IdleMode.kBrake);

      algaeMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  //1 to deliver 0 to stop
  public void setMotors(double value) {
    algaeMotor.set(value);
  }

  public Command deliverAlgae() {
    // Deliver the coral
    return runOnce(() -> setMotors(1));
  }
}
