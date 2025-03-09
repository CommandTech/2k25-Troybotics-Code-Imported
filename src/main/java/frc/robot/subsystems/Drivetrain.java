// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

public class Drivetrain extends SubsystemBase {
  private SparkMax leftDriveL;
  private SparkMax leftDriveF;
  private SparkMax rightDriveL;
  private SparkMax rightDriveF;
  private DifferentialDrive differentialDrive;
  private RobotConfig config;
  private DifferentialDriveKinematics kinematics;
  private Pose2d pose;
  private SparkMaxConfig leftConfigL;
  private SparkMaxConfig leftConfigF;
  private SparkMaxConfig rightConfigL;
  private SparkMaxConfig rightConfigF;
  private RelativeEncoder leftEncoderL;
  private RelativeEncoder rightEncoderL;
  private DifferentialDrivePoseEstimator poseEstimator;
  private DifferentialDriveWheelPositions wheelPositions;

  private PhotonCamera camera;
  //x y z (meters)
  //roll, pitch, yaw (radians)
  private static final Pose3d ROBOT_TO_CAMERA = new Pose3d(Units.inchesToMeters(12.), Units.inchesToMeters(2.), 0.,
            new Rotation3d(0., Units.degreesToRadians(56.), Units.degreesToRadians(0.)));

  /** Creates a new Drive. */
  public Drivetrain() {
      camera = new PhotonCamera("photonvision");

      leftDriveL = new SparkMax(Constants.MotorConstants.LEADER_LEFT_MOTOR_ID,MotorType.kBrushless);
      leftEncoderL = leftDriveL.getEncoder();

      leftConfigL = new SparkMaxConfig();
      leftConfigL.inverted(Constants.MotorConstants.LEADER_LEFT_MOTOR_INVERTED);
      leftConfigL.smartCurrentLimit(Constants.MotorConstants.LEADER_LEFT_MOTOR_AMP_LIMIT);
      leftConfigL.closedLoop
        .p(Constants.DriveConstants.DRIVE_P)
        .i(Constants.DriveConstants.DRIVE_I)
        .d(Constants.DriveConstants.DRIVE_D)
        .velocityFF(Constants.DriveConstants.DRIVE_FF)
        .outputRange(-1.0, 1.0);
      leftConfigL.idleMode(IdleMode.kBrake);
      leftDriveL.configure(leftConfigL, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

      leftDriveF = new SparkMax(Constants.MotorConstants.FOLLOWER_LEFT_MOTOR_ID,MotorType.kBrushless);

      leftConfigF = new SparkMaxConfig();
      leftConfigF.inverted(Constants.MotorConstants.FOLLOWER_LEFT_MOTOR_INVERTED);
      leftConfigF.smartCurrentLimit(Constants.MotorConstants.FOLLOWER_LEFT_MOTOR_AMP_LIMIT);
      leftConfigF.closedLoop
        .p(Constants.DriveConstants.DRIVE_P)
        .i(Constants.DriveConstants.DRIVE_I)
        .d(Constants.DriveConstants.DRIVE_D)
        .velocityFF(Constants.DriveConstants.DRIVE_FF)
        .outputRange(-1.0, 1.0);
      leftConfigF.idleMode(IdleMode.kBrake);
      leftDriveF.configure(leftConfigF, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      

      rightDriveL = new SparkMax(Constants.MotorConstants.LEADER_RIGHT_MOTOR_ID,MotorType.kBrushless);
      rightEncoderL = rightDriveL.getEncoder();

      rightConfigL = new SparkMaxConfig();
      rightConfigL.inverted(Constants.MotorConstants.LEADER_RIGHT_MOTOR_INVERTED);
      rightConfigL.smartCurrentLimit(Constants.MotorConstants.LEADER_RIGHT_MOTOR_AMP_LIMIT);
      rightConfigL.closedLoop
        .p(Constants.DriveConstants.DRIVE_P)
        .i(Constants.DriveConstants.DRIVE_I)
        .d(Constants.DriveConstants.DRIVE_D)
        .velocityFF(Constants.DriveConstants.DRIVE_FF)
        .outputRange(-1.0, 1.0);
      rightConfigL.idleMode(IdleMode.kBrake);
      rightDriveL.configure(rightConfigL, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

      rightDriveF = new SparkMax(Constants.MotorConstants.FOLLOWER_RIGHT_MOTOR_ID,MotorType.kBrushless);

      rightConfigF = new SparkMaxConfig();
      rightConfigF.inverted(Constants.MotorConstants.FOLLOWER_RIGHT_MOTOR_INVERTED);
      rightConfigF.smartCurrentLimit(Constants.MotorConstants.FOLLOWER_RIGHT_MOTOR_AMP_LIMIT);
      rightConfigF.closedLoop
        .p(Constants.DriveConstants.DRIVE_P)
        .i(Constants.DriveConstants.DRIVE_I)
        .d(Constants.DriveConstants.DRIVE_D)
        .velocityFF(Constants.DriveConstants.DRIVE_FF)
        .outputRange(-1.0, 1.0);
      rightConfigF.idleMode(IdleMode.kBrake);
      rightDriveF.configure(rightConfigF, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      

      differentialDrive = new DifferentialDrive(leftDriveL, rightDriveL);
      addChild("Differential Drive 1", differentialDrive);
      differentialDrive.setExpiration(0.1);
      differentialDrive.setMaxOutput(1.0);

      kinematics = new DifferentialDriveKinematics(0.8204);
      pose = new Pose2d();
      wheelPositions = new DifferentialDriveWheelPositions(leftEncoderL.getPosition(), rightEncoderL.getPosition());

      poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics,
        new Rotation2d(),
        leftEncoderL.getPosition(),
        rightEncoderL.getPosition(),
        pose
      );

      try {
        config = RobotConfig.fromGUISettings();
      } catch (IOException e) {
        e.printStackTrace();
      } catch (ParseException e) {
        e.printStackTrace();
      }
  

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    Pose2d visionPose = GetBestPose();

    wheelPositions = new DifferentialDriveWheelPositions(leftEncoderL.getPosition(), rightEncoderL.getPosition());

    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), new Rotation2d(), wheelPositions);
    poseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
    pose = poseEstimator.getEstimatedPosition();
  }

  public double getLeftPositionMeters() {
    return leftEncoderL.getPosition() * Constants.DriveConstants.WHEEL_DIAMETER_METERS;
  }
  public double getRightPositionMeters() {
    return rightEncoderL.getPosition() * Constants.DriveConstants.WHEEL_DIAMETER_METERS;
  }

  public void setMotors(double left, double right) {
    setLeftMotors(left);
    setRightMotors(right);
  }
  public void setLeftMotors (double volt) {
    leftDriveL.set(volt);
    
  }
  public void setRightMotors (double volt) {
    rightDriveL.set(volt);
  }

  public Pose2d getPose()
  {
    return pose;
  }
  public Pose2d resetPose(Pose2d newPose)
  {
    pose = newPose;
    return pose;
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeed(){
    return new DifferentialDriveWheelSpeeds(leftDriveL.getEncoder().getVelocity(), rightDriveL.getEncoder().getVelocity());
  }
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return kinematics.toChassisSpeeds(getWheelSpeed());
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    DifferentialDriveWheelSpeeds tankWheelsSpeeds = kinematics.toWheelSpeeds(speeds);
    tankWheelsSpeeds.desaturate(0.75);

    differentialDrive.arcadeDrive(-tankWheelsSpeeds.leftMetersPerSecond, -tankWheelsSpeeds.rightMetersPerSecond);
  }

  public void drive(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    DifferentialDriveWheelSpeeds tankWheelsSpeeds = kinematics.toWheelSpeeds(speeds);
    tankWheelsSpeeds.desaturate(0.75);

    differentialDrive.arcadeDrive(-tankWheelsSpeeds.leftMetersPerSecond, -tankWheelsSpeeds.rightMetersPerSecond);
  }

  public Command followPathCommand(String pathName) {
    try{
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        return new FollowPathCommand(
                path,
                this::getPose, // Robot pose supplier
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
                config, // The robot configuration
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
                this // Reference to this subsystem to set requirements
        );
    } catch (Exception e){
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  public Command driveCommand(double leftSpeed, double rightSpeed){
    return run(() -> drive(getRobotRelativeSpeeds(), null));
  }

  public Pose2d GetBestPose(){
    var result = camera.getLatestResult();
    if (result != null){
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      Transform3d bestPose = bestTarget.getBestCameraToTarget();


      //no idea what this does, need to test
      Pose3d bestPose3d = new Pose3d(bestPose.getTranslation(), bestPose.getRotation());
      
      Rotation2d newRotation = Rotation2d.fromDegrees(bestPose3d.getRotation().getAngle() - 180.);

      Pose2d finalPose = new Pose2d(bestPose3d.getTranslation().toTranslation2d(), newRotation).plus(
        new Transform2d(
                ROBOT_TO_CAMERA.getTranslation().toTranslation2d(),
                ROBOT_TO_CAMERA.getRotation().toRotation2d()));

      return finalPose;
    }
    return new Pose2d();
  }

  public Command DriveToStation(){
    if (pose.getY() > 4){
      //Blue station
      return run(() -> followPathCommand("DriveToBlueStation"));
    }
    //Red Station
    return run(() -> followPathCommand("DriveToRedStation"));
  }
}