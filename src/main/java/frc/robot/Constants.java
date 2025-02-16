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
  public static class MotorConstants {
    public static final int LEADER_LEFT_MOTOR_ID = 1;
    public static final boolean LEADER_LEFT_MOTOR_INVERTED = true;
    public static final int LEADER_LEFT_MOTOR_AMP_LIMIT = 40;

    public static final int FOLLOWER_LEFT_MOTOR_ID = 2;
    public static final boolean FOLLOWER_LEFT_MOTOR_INVERTED = true;
    public static final int FOLLOWER_LEFT_MOTOR_AMP_LIMIT = 40;

    public static final int LEADER_RIGHT_MOTOR_ID = 3;
    public static final boolean LEADER_RIGHT_MOTOR_INVERTED = false;
    public static final int LEADER_RIGHT_MOTOR_AMP_LIMIT = 40;
    
    public static final int FOLLOWER_RIGHT_MOTOR_ID = 4;
    public static final boolean FOLLOWER_RIGHT_MOTOR_INVERTED = false;
    public static final int FOLLOWER_RIGHT_MOTOR_AMP_LIMIT = 40;
    
    public static final int CORAL_MOTOR_ID = 5;
    public static final boolean CORAL_MOTOR_INVERTED = false;
    public static final int CORAL_MOTOR_AMP_LIMIT = 40;

    public static final int ALGAE_MOTOR_ID = 6;
    public static final boolean ALGAE_MOTOR_INVERTED = false;
    public static final int ALGAE_MOTOR_AMP_LIMIT = 40;

    public static final int ELEVATOR_MOTOR_ID = 7;
    public static final boolean ELEVATOR_MOTOR_INVERTED = false;
    public static final int ELEVATOR_MOTOR_AMP_LIMIT = 40;
  }
  public static class DriveConstants {
    public static final double GEAR_REDUCTION = 1.0;
    public static final double WHEEL_DIAMETER_METERS = 0.1524;
    public static final double TRACK_WIDTH_METERS = 0.6731;
    
    public static final double DRIVE_P = 0.1;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.1;
    public static final double DRIVE_FF = 1/473;
  }
  public static class ElevatorConstants {
    public static final double L4_HEIGHT = 0;
    public static final double L3_HEIGHT = 0;
    public static final double L2_HEIGHT = 0;
    public static final double L1_HEIGHT = 0;

    public static final double STOW_HEIGHT = 0;

    public static final double ELEVATOR_TOP_LIMIT = 0;
    public static final double ELEVATOR_BOTTOM_LIMIT = 0;
    public static final double ELEVATOR_GEAR_RATIO = 0;

    public static final double ELEVATOR_TOLERANCE = 0.5;

    public static final double ELEVATOR_P = 0.1;
    public static final double ELEVATOR_I = 0.0;
    public static final double ELEVATOR_D = 0.1;
    public static final double ELEVATOR_FF = 1/473;

    public static final double ELEVATOR_MAX_VELOCITY = 0;
    public static final double ELEVATOR_MAX_ACCELERATION = 0;
  }
  public static class OperatorConstants {
    public static final int DRIVER_PORT = 0;
    public static final int MANIP_PORT = 0;
  }
}
