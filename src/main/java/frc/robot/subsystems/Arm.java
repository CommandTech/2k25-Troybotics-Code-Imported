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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private final SparkMax arm;
  private final SparkMaxConfig armConfig;
  private final RelativeEncoder armEncoder;
  // private final SparkClosedLoopController armController;
  private final PIDController pidController;
  private final TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goalState;
  private TrapezoidProfile.State currentState;
  private final TrapezoidProfile profile;
  private double setpoint = 0.0;

  /** Creates a new arm. */
  public Arm() {
      //creates an arm spark max
      arm = new SparkMax(Constants.MotorConstants.LEADER_LEFT_MOTOR_ID,MotorType.kBrushless);
      //gets the encoder and the controller
      armEncoder = arm.getEncoder();
      // armController = arm.getClosedLoopController();

      //creates the config
      armConfig = new SparkMaxConfig();
      armConfig.inverted(Constants.MotorConstants.LEADER_LEFT_MOTOR_INVERTED);
      armConfig.smartCurrentLimit(Constants.MotorConstants.LEADER_LEFT_MOTOR_AMP_LIMIT);
      armConfig.idleMode(IdleMode.kBrake);
      // armConfig.closedLoop
      //   .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //   .p(Constants.ArmConstants.ARM_P)
      //   .i(Constants.ArmConstants.ARM_I)
      //   .d(Constants.ArmConstants.ARM_D)
      //   .velocityFF(Constants.ArmConstants.ARM_FF)
      //   .outputRange(-1.0, 1.0);
      // armConfig.closedLoop.maxMotion
      // .maxVelocity(Constants.ArmConstants.ARM_MAX_VELOCITY)
      // .maxAcceleration(Constants.ArmConstants.ARM_MAX_ACCELERATION)
      // .allowedClosedLoopError(Constants.ArmConstants.ARM_TOLERANCE);
      
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
      
      constraints = new TrapezoidProfile.Constraints(
          Constants.ArmConstants.ARM_MAX_VELOCITY,
          Constants.ArmConstants.ARM_MAX_ACCELERATION
      );
      
      pidController = new PIDController(
          Constants.ArmConstants.ARM_P,
          Constants.ArmConstants.ARM_I,
          Constants.ArmConstants.ARM_D
      );
      
      pidController.setTolerance(Constants.ArmConstants.ARM_TOLERANCE);

      // Initialize states and profile
      currentState = new TrapezoidProfile.State(0, 0);
      goalState = new TrapezoidProfile.State(0, 0);
      profile = new TrapezoidProfile(constraints);

      //sets the encoder position to 0 at the start
      armEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    //Every 20 ms updates the volts the motor should run at
    // arm.setVoltage(voltsOut);
    //Every 20 ms, the arm will set the reference to the desired setpoint
    currentState = profile.calculate(0.020, currentState, goalState); // 20ms control loop

    double pidOutput = pidController.calculate(getPosition(), currentState.position);
            double ff = calculateFeedForward(currentState);
            
            double outputPower = MathUtil.clamp(
                pidOutput + ff,
                -7,
                7
            );

    arm.set(outputPower);
  }
  
  public double getVelocityMetersPerSecond() {
    //using the gear ratio and encoder velocity, determines the velocity of the arm
    return ((armEncoder.getVelocity() / 60)/ Constants.ArmConstants.ARM_GEAR_RATIO) *
           (2 * Math.PI * 0.05);
  }

  public void setPosition(double height){
    //sets setpoint to the desired height and sets the controller reference to the height
    setpoint = height;
    // armController.setReference(setpoint, SparkBase.ControlType.kMAXMotionPositionControl);
    goalState = new TrapezoidProfile.State(setpoint, 0);
  }

  public double getPosition(){
    //gets the current position of the encoder
    return armEncoder.getPosition();
  }

  public Trigger atHeight(double tolerance){
    //returns if the arm is within tolerance of the desired height
    return new Trigger(() -> MathUtil.isNear(setpoint,
                                             getPosition(),
                                             tolerance));
  }

  public void stop() {
    //stops the arm's movement
    arm.set(0.0);
    pidController.reset();
  }


  public Command setHeight(double height) {
    //sets the desired height to the inputed height
    return run(() -> setPosition(height));
  }

  public Command idleCommand() {
    //Keeps the arm at the current height
    return run(() -> setPosition(getPosition()));
  }

  private double calculateFeedForward(TrapezoidProfile.State state) {
    // kS (static friction), kG (gravity), kV (velocity),
    return Constants.ArmConstants.ARM_S * Math.signum(state.velocity) +
           Constants.ArmConstants.ARM_G +
           Constants.ArmConstants.ARM_V * state.velocity;
}
}
