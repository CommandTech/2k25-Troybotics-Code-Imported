// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class CoralConstants {
    //Number of amps the motors spikes to when it has a piece in it
    public static final double INTAKED_CORAL_AMPS = 30;
  }
  public static class AlgaeConstants {
    //Number of amps the motors spikes to when it has a piece in it
    public static final double INTAKED_ALGAE_AMPS = 30;
  }
  public static class MotorConstants {
    //Left Leader motor constants
    public static final int LEADER_LEFT_MOTOR_ID = 1;
    public static final boolean LEADER_LEFT_MOTOR_INVERTED = true;
    public static final int LEADER_LEFT_MOTOR_AMP_LIMIT = 40;

    //Left follower motor constants
    public static final int FOLLOWER_LEFT_MOTOR_ID = 2;
    public static final boolean FOLLOWER_LEFT_MOTOR_INVERTED = true;
    public static final int FOLLOWER_LEFT_MOTOR_AMP_LIMIT = 40;

    //Right Leader motor constants
    public static final int LEADER_RIGHT_MOTOR_ID = 3;
    public static final boolean LEADER_RIGHT_MOTOR_INVERTED = false;
    public static final int LEADER_RIGHT_MOTOR_AMP_LIMIT = 40;
    
    //Right Follower motor constants
    public static final int FOLLOWER_RIGHT_MOTOR_ID = 4;
    public static final boolean FOLLOWER_RIGHT_MOTOR_INVERTED = false;
    public static final int FOLLOWER_RIGHT_MOTOR_AMP_LIMIT = 40;
    
    //Coral motor constants
    public static final int CORAL_MOTOR_ID = 5;
    public static final boolean CORAL_MOTOR_INVERTED = false;
    public static final int CORAL_MOTOR_AMP_LIMIT = 40;

    //Algae motor constants
    public static final int ALGAE_MOTOR_ID = 6;
    public static final boolean ALGAE_MOTOR_INVERTED = false;
    public static final int ALGAE_MOTOR_AMP_LIMIT = 40;

    //Arm motor constants
    public static final int ARM_MOTOR_ID = 7;
    public static final boolean ARM_MOTOR_INVERTED = false;
    public static final int ARM_MOTOR_AMP_LIMIT = 40;
  }
  public static class DriveConstants {
    //Gear ratio on the drivetrain
    public static final double GEAR_REDUCTION = 1.0;
    //Wheel diameter of the drivetrain
    public static final double WHEEL_DIAMETER_METERS = 0.1524;
    //Width of the robot
    public static final double TRACK_WIDTH_METERS = 0.6731;
    
    //PID of the drivetrain
    public static final double DRIVE_P = 0.1;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.1;
    public static final double DRIVE_FF = 1/473;

    public static final double DRIVE_S = 0.1;
    public static final double DRIVE_V = 0.1;
  }
  public static class ArmConstants {
    //Delivery heights for the arm encoder values
    public static final double L3_HEIGHT = 500;
    public static final double L2_HEIGHT = 250;
    public static final double L1_HEIGHT = 100;

    public static final double STOW_HEIGHT = 0;

    //Bounds of the arm
    public static final double ARM_TOP_LIMIT = 500;
    public static final double ARM_BOTTOM_LIMIT = 0;
    //Gear ratio of the arm
    public static final double ARM_GEAR_RATIO = 100;

    //Arm tolerance
    public static final double ARM_TOLERANCE = 0.5;

    //Arm PID
    public static final double ARM_P = 0.1;
    public static final double ARM_I = 0.0;
    public static final double ARM_D = 0.1;
    public static final double ARM_FF = 1/473;
    
    public static final double ARM_S = 0.01964; // volts (V)
    public static final double ARM_V = 3.894; // volt per velocity (V/(m/s))
    public static final double ARM_A = 0.173; // volt per acceleration (V/(m/s²))
    public static final double ARM_G = 0.91274; // volts (V)

    public static final double ARM_MAX_VELOCITY = 5;
    public static final double ARM_MAX_ACCELERATION = 5;
  }
  public static class OperatorConstants {
    public static final int DRIVER_PORT = 1;
    public static final int MANIP_PORT = 2;
  }
}
