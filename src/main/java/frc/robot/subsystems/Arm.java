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
  private final SparkMax arm;
  private final SparkMaxConfig armConfig;
  private final RelativeEncoder armEncoder;
  private final SparkClosedLoopController armController;
  // private final ProfiledPIDController m_controller;
  private double setpoint;
  // private final armFeedforward m_feedforward;

  /** Creates a new arm. */
  public Arm() {
      //creates an arm spark max
      arm = new SparkMax(Constants.MotorConstants.LEADER_LEFT_MOTOR_ID,MotorType.kBrushless);
      //gets the encoder and the controller
      armEncoder = arm.getEncoder();
      armController = arm.getClosedLoopController();

      //creates the config
      armConfig = new SparkMaxConfig();
      armConfig.inverted(Constants.MotorConstants.LEADER_LEFT_MOTOR_INVERTED);
      armConfig.smartCurrentLimit(Constants.MotorConstants.LEADER_LEFT_MOTOR_AMP_LIMIT);
      armConfig.idleMode(IdleMode.kBrake);
      armConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(Constants.ArmConstants.ARM_P)
        .i(Constants.ArmConstants.ARM_I)
        .d(Constants.ArmConstants.ARM_D)
        .velocityFF(Constants.ArmConstants.ARM_FF)
        .outputRange(-1.0, 1.0);
      armConfig.closedLoop.maxMotion
      .maxVelocity(Constants.ArmConstants.ARM_MAX_VELOCITY)
      .maxAcceleration(Constants.ArmConstants.ARM_MAX_ACCELERATION)
      .allowedClosedLoopError(Constants.ArmConstants.ARM_TOLERANCE);
      
      //Sets the gear ratio of the encoder
      EncoderConfig armEncoderConfig = armConfig.encoder;
        armEncoderConfig.positionConversionFactor(Constants.ArmConstants.ARM_GEAR_RATIO);

      //enables soft limits and sets them
      SoftLimitConfig armSoftLimits = armConfig.softLimit;
        armSoftLimits.forwardSoftLimitEnabled(true);
        armSoftLimits.forwardSoftLimit(Constants.ArmConstants.ARM_TOP_LIMIT);
        armSoftLimits.reverseSoftLimitEnabled(true);
        armSoftLimits.reverseSoftLimit(Constants.ArmConstants.ARM_TOP_LIMIT);

      arm.configure(armConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      
      // m_controller = new ProfiledPIDController(armConstants.ARM_P,
      //                                         armConstants.ARM_I,
      //                                         armConstants.ARM_D,
      //                                         new Constraints(armConstants.ARM_MAX_VELOCITY,
      //                                                         armConstants.ARM_MAX_ACCELERATION));
      //Defaults the setpoint to the stow height
      setpoint = Constants.ArmConstants.STOW_HEIGHT;
      
      // m_feedforward =
      //     new ArmFeedforward(
      //         armConstants.ARM_S,
      //         armConstants.ARM_G,
      //         armConstants.ARM_V,
      //         armConstants.ARM_A);

      //sets the encoder position to 0 at the start
      armEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    //Every 20 ms updates the volts the motor should run at
    // double voltsOut = MathUtil.clamp(
    //     m_controller.calculate(getPosition(), setpoint) +
    //     m_feedforward.calculateWithVelocities(getVelocityMetersPerSecond(),
    //                                           m_controller.getSetpoint().velocity), -7, 7);

    // arm.setVoltage(voltsOut);
    //Every 20 ms, the arm will set the reference to the desired setpoint
    // armController.setReference(setpoint, ControlType.kMAXMotionPositionControl);
  }
  
  public double getVelocityMetersPerSecond() {
    //using the gear ratio and encoder velocity, determines the velocity of the arm
    return ((armEncoder.getVelocity() / 60)/ Constants.ArmConstants.ARM_GEAR_RATIO) *
           (2 * Math.PI * 0.05);
  }

  public void setPosition(double height){
    //sets setpoint to the desired height and sets the controller reference to the height
    setpoint = height;
    armController.setReference(height, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  public double getPosition(){
    //gets the current position of the encoder
    return armEncoder.getPosition();
  }

  public Trigger atHeight(double height, double tolerance){
    //returns if the arm is within tolerance of the desired height
    return new Trigger(() -> MathUtil.isNear(height,
                                             getPosition(),
                                             tolerance));
  }

  public void stop() {
    //stops the arm's movement
    arm.set(0.0);
  }


  public Command setHeight(double height) {
    //sets the desired height to the inputed height
    return run(() -> setPosition(height));
  }

  public Command idleCommand() {
    //Keeps the arm at the current height
    return run(() -> setPosition(getPosition()));
  }
}
