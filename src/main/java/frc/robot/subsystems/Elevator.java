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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final SparkMax elevator;
  private final SparkMaxConfig elevatorConfig;
  private final RelativeEncoder elevatorEncoder;
  private final SparkClosedLoopController elevatorController;
  // private final ProfiledPIDController m_controller;
  private double setpoint;
  // private final ElevatorFeedforward m_feedforward;

  /** Creates a new Elevator. */
  public Elevator() {
      elevator = new SparkMax(Constants.MotorConstants.LEADER_LEFT_MOTOR_ID,MotorType.kBrushless);
      elevatorEncoder = elevator.getEncoder();
      elevatorController = elevator.getClosedLoopController();

      elevatorConfig = new SparkMaxConfig();
      elevatorConfig.inverted(Constants.MotorConstants.LEADER_LEFT_MOTOR_INVERTED);
      elevatorConfig.smartCurrentLimit(Constants.MotorConstants.LEADER_LEFT_MOTOR_AMP_LIMIT);
      elevatorConfig.idleMode(IdleMode.kBrake);
      elevatorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
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
      
      // m_controller = new ProfiledPIDController(ElevatorConstants.ELEVATOR_P,
      //                                         ElevatorConstants.ELEVATOR_I,
      //                                         ElevatorConstants.ELEVATOR_D,
      //                                         new Constraints(ElevatorConstants.ELEVATOR_MAX_VELOCITY,
      //                                                         ElevatorConstants.ELEVATOR_MAX_ACCELERATION));
      setpoint = Constants.ElevatorConstants.STOW_HEIGHT;
      
      // m_feedforward =
      //     new ElevatorFeedforward(
      //         ElevatorConstants.ELEVATOR_S,
      //         ElevatorConstants.ELEVATOR_G,
      //         ElevatorConstants.ELEVATOR_V,
      //         ElevatorConstants.ELEVATOR_A);

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
    return ((elevatorEncoder.getVelocity() / 60)/ ElevatorConstants.ELEVATOR_GEAR_RATIO) *
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
}
