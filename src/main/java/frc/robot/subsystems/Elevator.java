// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkMax elevator;
  private SparkMaxConfig elevatorConfig;
  private RelativeEncoder elevatorEncoder;
  private SparkClosedLoopController elevatorController;
  private double setpoint = 0;
  /** Creates a new Elevator. */
  public Elevator() {
      elevator = new SparkMax(Constants.MotorConstants.LEADER_LEFT_MOTOR_ID,MotorType.kBrushless);
      elevatorEncoder = elevator.getEncoder();
      elevatorController = elevator.getClosedLoopController();

      elevatorConfig = new SparkMaxConfig();
      elevatorConfig.inverted(Constants.MotorConstants.LEADER_LEFT_MOTOR_INVERTED);
      elevatorConfig.smartCurrentLimit(Constants.MotorConstants.LEADER_LEFT_MOTOR_AMP_LIMIT);
      elevatorConfig.closedLoop
        .p(Constants.ElevatorConstants.ELEVATOR_P)
        .i(Constants.ElevatorConstants.ELEVATOR_I)
        .d(Constants.ElevatorConstants.ELEVATOR_D)
        .velocityFF(Constants.ElevatorConstants.ELEVATOR_FF)
        .outputRange(-1.0, 1.0);
      elevatorConfig.closedLoop.maxMotion
      .maxVelocity(Constants.ElevatorConstants.ELEVATOR_MAX_VELOCITY)
      .maxAcceleration(Constants.ElevatorConstants.ELEVATOR_MAX_ACCELERATION)
      .allowedClosedLoopError(Constants.ElevatorConstants.ELEVATOR_TOLERANCE);

      EncoderConfig elevatorEncoderConfig = elevatorConfig.encoder;
        elevatorEncoderConfig.positionConversionFactor(Constants.ElevatorConstants.ELEVATOR_GEAR_RATIO);

      SoftLimitConfig elevatorSoftLimits = elevatorConfig.softLimit;
        elevatorSoftLimits.forwardSoftLimitEnabled(true);
        elevatorSoftLimits.forwardSoftLimit(Constants.ElevatorConstants.ELEVATOR_TOP_LIMIT);
        elevatorSoftLimits.reverseSoftLimitEnabled(true);
        elevatorSoftLimits.reverseSoftLimit(Constants.ElevatorConstants.ELEVATOR_TOP_LIMIT);

      elevator.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

  @Override
  public void periodic() {
    //Should make the voltage less when it is closer to the setpoint exponentially
    elevator.setVoltage(MathUtil.clamp(
      (setpoint - getPosition())*Math.abs(setpoint - getPosition())
    , -7, 7));
  }

  public void setPosition(double height){
    setpoint = height;
    elevatorController.setReference(height, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  public double getPosition(){
    return elevatorEncoder.getPosition();
  }

  public boolean atSetpoint(){
    return Math.abs(setpoint - getPosition()) < Constants.ElevatorConstants.ELEVATOR_TOLERANCE;
  }

  
  public Command setHeight(double height){
    return run(() -> setPosition(height));
  }
}
