// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // test comment
  public static final double LOOP_PERIOD_SECS = 0.02;

  public static final boolean TUNING_MODE = true;

  public static final String CAN_BUS_NAME = "SwerveBus";

  // FIXME: specify the name of the camera used for detecting AprilTags
  public static final String CAMERA_NAME = "Microsoft_LifeCam_HD-3000";

  private static final RobotType ROBOT = RobotType.ROBOT_2023_SEASON;

  private static final Alert invalidRobotAlert =
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (ROBOT == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_2023_SEASON;
      } else {
        return ROBOT;
      }
    } else {
      return ROBOT;
    }
  }

  // FIXME: update for various robots
  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_2022_PRESEASON:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      case ROBOT_2023_SEASON:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      default:
        return Mode.REAL;
    }
  }

  // FIXME: update for various robots
  public enum RobotType {
    ROBOT_2022_PRESEASON,
    ROBOT_SIMBOT,
    ROBOT_2023_SEASON
  }

  public enum Mode {
    REAL,
    REPLAY,
    SIM
  }
}
