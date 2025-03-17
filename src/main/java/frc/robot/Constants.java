// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{ //HELLOOOOOOO

  public static final double ROBOT_MASS = (114) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
  public static final double WHEEL_LOCK_TIME = 10; // seconds
        public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(0.2);

    public static final PPHolonomicDriveController PP_CONTROLLER = new PPHolonomicDriveController(
      // PPHolonomicController is the built in path following controller for holonomic
      // drive trains
      new PIDConstants(5.7, 0.0, 0.02),
      // Translation PID constants
      new PIDConstants(1.9, 0.0, 0.02)
    );
    // Rotation PID constants);

  }

  public static class ElevPID {
    public static final double ELEVATOR_P = 0.11;
    public static final double ELEVATOR_I = 0.0;
    public static final double ELEVATOR_D = 0.000;

    public static final double C_WRIST_P = 0.07;
    public static final double C_WRIST_I = 0.000003;
    public static final double C_WRIST_D = 0.4;

    public static final double A_WRIST_P = 0.14;
    public static final double A_WRIST_I = 0.0;
    public static final double A_WRIST_D = 0.0;
  }

  public static class ClimbConstants {
    public static final double CLIMB_LIMIT = 10.0;
    public static final double CLIMB_SPEED = 1;
  } // 4 for intaking from coral station

  public static class HomeConstants {
    public static final double CORAL_HOME_POSITION = 3.5;
    public static final double CORAL_STOW_POSITION = 0.5;
    public static final double ALGAE_HOME_POSITION = 1.0;
    public static final double ELEVATOR_HOME_POSITION = 0.0;
  }

  public static class IOSpeeds {
    public static final double CORAL_INTAKE_SPEED = -0.3;
    public static final double ALGAE_INTAKE_SPEED = -1.0;
    public static final double CORAL_SHOOT_SPEED = 0.3;
    public static final double ALGAE_SHOOT_SPEED = 0.75;
  }

  public static class ReefPose {
    public static final Translation2d BLUE_REEF_CENTER = new Translation2d(Meters.of(4.5), Meters.of(4));
    public static final Translation2d RED_REEF_CENTER = new Translation2d(Meters.of(13), Meters.of(4));
    //positive = further way
    public static final Translation2d FRONT_BACK_OFFSET = new Translation2d(Inches.of(54), Inches.of(0));
    //positive = further from the center

    public static final Translation2d INITIAL_ALIGNMENT_OFFSET = new Translation2d(Meters.of(2).plus(Inches.of(3)), Meters.of(0));

    public static final Translation2d centerOffset = new Translation2d(Inches.of(0), Inches.of(5.9));
    public static final Translation2d leftRightOffset = new Translation2d(Inches.of(0), Inches.of(3.7));

  }

  public static class ReefLevels {
    public static final double E_L1_POSITION = 15;
    public static final double E_L2_POSITION = 39.5;
    public static final double E_L3_POSITION = 65.5;

    public static final double C_L1_POSITION = 1.0;
    public static final double C_L2_POSITION = 14.57;
    public static final double C_L3_POSITION = 11.5;

    public static final double A_TILT_HIGH_POSITION = 9;

    public static final double E_PROCESSOR_POSITION = 2;
    public static final double E_AL2_POSITION = 40;
    public static final double E_AL3_POSITION = 75;
    public static final double A_IO_POSITION = 8;

    public static final double E_STATION_POSITION = 18.34; // tweak this value a bit
    public static final double C_STATION_POSITION = 5.95;
  }

  public static class Tolerances {
    public static final double ELEVATOR_TOLERANCE = 1;
    public static final double CLIMB_TOLERANCE = 10;
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double DEADBAND         = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}
