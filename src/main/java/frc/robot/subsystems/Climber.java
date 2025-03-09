// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private SparkMax climberMotor;
  private SparkMaxConfig config; 
  private final RelativeEncoder climberEncoder;
  private final SparkClosedLoopController climberController;
  private double setpoint;
  private boolean isDeployed;
 
  /** Creates a new Climber. */
  public Climber() {
      //creates a new SparkMax with the CanID of ALGAE_MOTOR_ID constant
      climberMotor = new SparkMax(Constants.MotorConstants.CLIMBER_MOTOR_ID, SparkMax.MotorType.kBrushless);
      climberEncoder = climberMotor.getEncoder();
      climberController = climberMotor.getClosedLoopController();

      //creates a configuration for the motor
      config = new SparkMaxConfig();
      config.inverted(Constants.MotorConstants.CLIMBER_MOTOR_INVERTED);
      config.smartCurrentLimit(Constants.MotorConstants.CLIMBER_MOTOR_AMP_LIMIT);
      config.idleMode(IdleMode.kBrake);
      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(Constants.ClimberConstants.CLIMBER_P)
        .i(Constants.ClimberConstants.CLIMBER_I)
        .d(Constants.ClimberConstants.CLIMBER_D)
        .velocityFF(Constants.ClimberConstants.CLIMBER_FF)
        .outputRange(-1.0, 1.0);
      config.closedLoop.maxMotion
        .maxVelocity(Constants.ClimberConstants.CLIMBER_MAX_VELOCITY)
        .maxAcceleration(Constants.ClimberConstants.CLIMBER_MAX_ACCELERATION)
        .allowedClosedLoopError(Constants.ClimberConstants.CLIMBER_TOLERANCE);
      
      //Sets the gear ratio of the encoder
      EncoderConfig climberEncoderConfig = config.encoder;
        climberEncoderConfig.positionConversionFactor(Constants.ClimberConstants.CLIMBER_GEAR_RATIO);

      //enables soft limits and sets them
      SoftLimitConfig climberSoftLimits = config.softLimit;
        climberSoftLimits.forwardSoftLimitEnabled(true);
        climberSoftLimits.forwardSoftLimit(Constants.ClimberConstants.CLIMBER_READY_ANGLE);
        climberSoftLimits.reverseSoftLimitEnabled(true);
        climberSoftLimits.reverseSoftLimit(Constants.ClimberConstants.CLIMBER_STOWED_ANGLE);

      //sets the configuration to the motor
      climberMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      
      setpoint = Constants.ClimberConstants.CLIMBER_STOWED_ANGLE;
      climberEncoder.setPosition(0);

      isDeployed = false;
      SmartDashboard.putBoolean("Climber Deployed", isDeployed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPosition(double height){
    //sets setpoint to the desired height and sets the controller reference to the height
    setpoint = height;
    climberController.setReference(setpoint, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  public double getPosition(){
    //gets the current position of the encoder
    return climberEncoder.getPosition();
  }

  public Trigger atHeight(double tolerance){
    //returns if the arm is within tolerance of the desired height
    return new Trigger(() -> MathUtil.isNear(setpoint,
                                             getPosition(),
                                             tolerance));
  }

  public void stop() {
    //stops the arm's movement
    climberMotor.set(0.0);
  }

  public Command toggleClimber(){
    if (!isDeployed){
      isDeployed = true;
      return setHeight(Constants.ClimberConstants.CLIMBER_READY_ANGLE);
    }
    isDeployed = false;
    return setHeight(Constants.ClimberConstants.CLIMBER_CLIMBED_ANGLE);
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
