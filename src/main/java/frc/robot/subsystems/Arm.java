// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private final SparkMax elevator;
  private final SparkMaxConfig elevatorConfig;
  private final RelativeEncoder elevatorEncoder;
  private final SparkClosedLoopController elevatorController;
  // private final ProfiledPIDController m_controller;
  private double setpoint;
  // private final ElevatorFeedforward m_feedforward;

  /** Creates a new Elevator. */
  public Arm() {
      elevator = new SparkMax(Constants.MotorConstants.LEADER_LEFT_MOTOR_ID,MotorType.kBrushless);
      elevatorEncoder = elevator.getEncoder();
      elevatorController = elevator.getClosedLoopController();

      elevatorConfig = new SparkMaxConfig();
      elevatorConfig.inverted(Constants.MotorConstants.LEADER_LEFT_MOTOR_INVERTED);
      elevatorConfig.smartCurrentLimit(Constants.MotorConstants.LEADER_LEFT_MOTOR_AMP_LIMIT);
      elevatorConfig.idleMode(IdleMode.kBrake);
      elevatorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(Constants.ArmConstants.ARM_P)
        .i(Constants.ArmConstants.ARM_I)
        .d(Constants.ArmConstants.ARM_D)
        .velocityFF(Constants.ArmConstants.ARM_FF)
        .outputRange(-1.0, 1.0);
      elevatorConfig.closedLoop.maxMotion
      .maxVelocity(Constants.ArmConstants.ARM_MAX_VELOCITY)
      .maxAcceleration(Constants.ArmConstants.ARM_MAX_ACCELERATION)
      .allowedClosedLoopError(Constants.ArmConstants.ARM_TOLERANCE);
      
      EncoderConfig elevatorEncoderConfig = elevatorConfig.encoder;
        elevatorEncoderConfig.positionConversionFactor(Constants.ArmConstants.ARM_GEAR_RATIO);

      SoftLimitConfig elevatorSoftLimits = elevatorConfig.softLimit;
        elevatorSoftLimits.forwardSoftLimitEnabled(true);
        elevatorSoftLimits.forwardSoftLimit(Constants.ArmConstants.ARM_TOP_LIMIT);
        elevatorSoftLimits.reverseSoftLimitEnabled(true);
        elevatorSoftLimits.reverseSoftLimit(Constants.ArmConstants.ARM_TOP_LIMIT);

      elevator.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      
      // m_controller = new ProfiledPIDController(ElevatorConstants.ARM_P,
      //                                         ElevatorConstants.ARM_I,
      //                                         ElevatorConstants.ARM_D,
      //                                         new Constraints(ElevatorConstants.ARM_MAX_VELOCITY,
      //                                                         ElevatorConstants.ARM_MAX_ACCELERATION));
      setpoint = Constants.ArmConstants.STOW_HEIGHT;
      
      // m_feedforward =
      //     new ArmFeedforward(
      //         ElevatorConstants.ARM_S,
      //         ElevatorConstants.ARM_G,
      //         ElevatorConstants.ARM_V,
      //         ElevatorConstants.ARM_A);

      elevatorEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    //Every 20 ms updates the volts the motor should run at
    // double voltsOut = MathUtil.clamp(
    //     m_controller.calculate(getPosition(), setpoint) +
    //     m_feedforward.calculateWithVelocities(getVelocityMetersPerSecond(),
    //                                           m_controller.getSetpoint().velocity), -7, 7);

    // elevator.setVoltage(voltsOut);
    elevatorController.setReference(setpoint, ControlType.kMAXMotionPositionControl);
  }
  
  public double getVelocityMetersPerSecond()
  {
    return ((elevatorEncoder.getVelocity() / 60)/ Constants.ArmConstants.ARM_GEAR_RATIO) *
           (2 * Math.PI * 0.05);
  }

  public void setPosition(double height){
    setpoint = height;
    elevatorController.setReference(height, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  public double getPosition(){
    return elevatorEncoder.getPosition();
  }

  public Trigger atHeight(double height, double tolerance)
  {
    return new Trigger(() -> MathUtil.isNear(height,
                                             getPosition(),
                                             tolerance));
  }

  public void stop()
  {
    elevator.set(0.0);
  }


  public Command setHeight(double height){
    return run(() -> setPosition(height));
  }

  public Command idleCommand(){
    return run(() -> setPosition(getPosition()));
  }
}
