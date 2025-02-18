// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOPigeon2;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIO;
import frc.lib.team3061.swerve.SwerveModuleIOSim;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFX;
import frc.lib.team3061.vision.Vision;
import frc.lib.team3061.vision.VisionConstants;
import frc.lib.team3061.vision.VisionIO;
import frc.lib.team3061.vision.VisionIOPhotonVision;
import frc.lib.team3061.vision.VisionIOSim;
import frc.robot.Constants.Mode;
import frc.robot.commands.ArmMoveHome;
import frc.robot.commands.AutoBalanceMove;
import frc.robot.commands.AutoMoveSpeed;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.CoordinatedArmMove;
import frc.robot.commands.CoordinatedArmMovePos;
import frc.robot.commands.FollowPath;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArmToPos;
import frc.robot.commands.MoveArmToPosN;
import frc.robot.commands.SimpleArmMove;
import frc.robot.commands.TeleopSwerve;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.gripper.Gripper;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static OperatorInterface oi = new OperatorInterface() {};

  private Drivetrain drivetrain;
  private Gripper gripper;
  private static Arm arm;
  private static double adjustJ3 = 0.0;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();

  /** Create the container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //CameraServer.startAutomaticCapture();
    // create real, simulated, or replay subsystems based on the mode and robot specified
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2023_SEASON:
          {
            GyroIO gyro = new GyroIOPigeon2(PIGEON_ID);

            SwerveModule flModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        0,
                        FRONT_LEFT_MODULE_DRIVE_MOTOR,
                        FRONT_LEFT_MODULE_STEER_MOTOR,
                        FRONT_LEFT_MODULE_STEER_ENCODER,
                        FRONT_LEFT_MODULE_STEER_OFFSET),
                    0,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule frModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        1,
                        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_ENCODER,
                        FRONT_RIGHT_MODULE_STEER_OFFSET),
                    1,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule blModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        2,
                        BACK_LEFT_MODULE_DRIVE_MOTOR,
                        BACK_LEFT_MODULE_STEER_MOTOR,
                        BACK_LEFT_MODULE_STEER_ENCODER,
                        BACK_LEFT_MODULE_STEER_OFFSET),
                    2,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule brModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        3,
                        BACK_RIGHT_MODULE_DRIVE_MOTOR,
                        BACK_RIGHT_MODULE_STEER_MOTOR,
                        BACK_RIGHT_MODULE_STEER_ENCODER,
                        BACK_RIGHT_MODULE_STEER_OFFSET),
                    3,
                    MAX_VELOCITY_METERS_PER_SECOND);

            drivetrain = new Drivetrain(gyro, flModule, frModule, blModule, brModule);
            // new Pneumatics(new PneumaticsIORev());
            new Vision(new VisionIOPhotonVision(CAMERA_NAME));
            gripper = new Gripper();
            arm = new Arm();

            break;
          }
        case ROBOT_SIMBOT:
          {
            SwerveModule flModule =
                new SwerveModule(new SwerveModuleIOSim(), 0, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule frModule =
                new SwerveModule(new SwerveModuleIOSim(), 1, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule blModule =
                new SwerveModule(new SwerveModuleIOSim(), 2, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule brModule =
                new SwerveModule(new SwerveModuleIOSim(), 3, MAX_VELOCITY_METERS_PER_SECOND);
            drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
            // new Pneumatics(new PneumaticsIO() {});
            AprilTagFieldLayout layout;
            try {
              layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
            } catch (IOException e) {
              layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
            }
            new Vision(
                new VisionIOSim(layout, drivetrain::getPose, VisionConstants.ROBOT_TO_CAMERA));
            break;
          }
        default:
          break;
      }

    } else {
      SwerveModule flModule =
          new SwerveModule(new SwerveModuleIO() {}, 0, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule frModule =
          new SwerveModule(new SwerveModuleIO() {}, 1, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule blModule =
          new SwerveModule(new SwerveModuleIO() {}, 2, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule brModule =
          new SwerveModule(new SwerveModuleIO() {}, 3, MAX_VELOCITY_METERS_PER_SECOND);
      drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
      // new Pneumatics(new PneumaticsIO() {});
      new Vision(new VisionIO() {});
    }

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    updateOI();

    configureAutoCommands();
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    oi = OISelector.findOperatorInterface();

    /*
     * Set up the default command for the drivetrain. The joysticks' values map to percentage of the
     * maximum velocities. The velocities may be specified from either the robot's frame of
     * reference or the field's frame of reference. In the robot's frame of reference, the positive
     * x direction is forward; the positive y direction, left; position rotation, CCW. In the field
     * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
     * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
     * direction. This is why the left joystick's y axis specifies the velocity in the x direction
     * and the left joystick's x axis specifies the velocity in the y direction.
     */
    drivetrain.setDefaultCommand(
        new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate));

    // arm.setDefaultCommand(
    // new MoveArm(ArmPositions.HOME, arm));

    configureButtonBindings();
  }

  /**
   * Factory method to create the singleton robot container object.
   *
   * @return the singleton robot container object
   */
  public static RobotContainer getInstance() {
    return robotContainer;
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // field-relative toggle
    /*oi.getFieldRelativeButton()
    .toggleOnTrue(
        Commands.either(
            Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
            Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
            drivetrain::getFieldRelative));
             */

    oi.getFieldRelativeButton()
        .onTrue(Commands.runOnce(drivetrain::disableFieldRelative, drivetrain));

    oi.getFieldRelativeButton()
        .onFalse(Commands.runOnce(drivetrain::enableFieldRelative, drivetrain));

    // reset gyro to 0 degrees
    oi.getResetGyroButton().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    // x-stance
    oi.getXStanceButton().onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    oi.getXStanceButton().onFalse(Commands.runOnce(drivetrain::disableXstance, drivetrain));

    oi.getSpeedOverrideL().onTrue(Commands.runOnce(drivetrain::setSpeedOverride));
    oi.getSpeedOverrideL().onFalse(Commands.runOnce(drivetrain::endSpeedOverride));

    oi.getSpeedOverrideR().onTrue(Commands.runOnce(drivetrain::setPrecisionMode));
    oi.getSpeedOverrideR().onFalse(Commands.runOnce(drivetrain::endPrecisionMode));

    oi.getAutoTurnNorth().whileTrue(new AutoTurn(0.0, drivetrain));
    oi.getAutoTurnSouth().whileTrue(new AutoTurn(180.0, drivetrain));
    // intake & gripper controls
    oi.getCloseButton()
        .onTrue(
            new ConditionalCommand(
                Commands.runOnce(gripper::intakeCone, gripper),
                Commands.runOnce(gripper::intakeCube, gripper),
                () -> oi.getGamePieceType()));
    oi.getCloseButton().onFalse(Commands.runOnce(gripper::newIdleIntake, gripper));
    /*
    oi.getOpenButton()
        .onTrue(
            new ConditionalCommand(
                Commands.runOnce(gripper::releaseCube, gripper),
                Commands.runOnce(gripper::releaseCube, gripper),
                () -> oi.getGamePieceType()));
    oi.getOpenButton().onFalse(Commands.runOnce(gripper::stopIntake, gripper));


        oi.getOpenButton()
        .onFalse(Commands.runOnce(gripper::stopIntake, gripper));
    */

    oi.getOpenButton()
        .onTrue(
            new ConditionalCommand(
                new ConditionalCommand(
                    new CoordinatedArmMovePos(ArmPositions.N_CONE_HIGH_DUNK, arm)
                        .andThen(
                            new CoordinatedArmMove(ArmPositions.N_CONE_HIGH_DUNK, arm)
                                .alongWith(Commands.runOnce(gripper::releaseCube, gripper))),
                    new ConditionalCommand(
                        new CoordinatedArmMovePos(ArmPositions.N_CONE_MID_DUNK, arm)
                            .andThen(
                                new CoordinatedArmMove(ArmPositions.N_CONE_MID_DUNK, arm)
                                    .alongWith(Commands.runOnce(gripper::releaseCube, gripper))),
                        Commands.runOnce(gripper::releaseCube, gripper),
                        oi.getMoveToMid()),
                    oi.getMoveToHigh()),
                Commands.runOnce(gripper::releaseCube, gripper),
                () -> oi.getGamePieceType()));

    oi.getOpenButton()
        .onFalse(
            new ConditionalCommand(
                new ConditionalCommand(
                    new CoordinatedArmMove(ArmPositions.N_CONE_HIGH, arm)
                        .alongWith(Commands.runOnce(gripper::stopIntake, gripper)),
                    new ConditionalCommand(
                        new CoordinatedArmMove(ArmPositions.N_CONE_MID, arm)
                            .alongWith(Commands.runOnce(gripper::stopIntake, gripper)),
                        Commands.runOnce(gripper::stopIntake, gripper),
                        oi.getMoveToMid()),
                    oi.getMoveToHigh()),
                Commands.runOnce(gripper::stopIntake, gripper),
                () -> oi.getGamePieceType()));

    oi.getMoveToPickup()
        .onTrue(
            new ConditionalCommand(
                new ConditionalCommand(
                    new SimpleArmMove(ArmPositions.N_CONE_SHELF, arm),
                    new SimpleArmMove(ArmPositions.N_CUBE_SHELF, arm),
                    () -> oi.getGamePieceType()),
                new ConditionalCommand(
                    new CoordinatedArmMovePos(ArmPositions.N_CONE_GROUND_INTERMEDIATE, arm)
                        .andThen(new CoordinatedArmMove(ArmPositions.N_CONE_GROUND, arm)),
                        new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND_INTERMEDIATE, arm)
                        .andThen(new CoordinatedArmMove(ArmPositions.N_CUBE_GROUND, arm)),
                     /*   new CoordinatedArmMove(ArmPositions.N_CUBE_GROUND_NEW, arm)
                        .alongWith( 
                            Commands.runOnce(drivetrain::setPrecisionMode)), */ 
                        
                    () -> oi.getGamePieceType()),
                () -> oi.getPickupLocation()));

    oi.getMoveToPickup()
        .onFalse(
            new ConditionalCommand(
                new ConditionalCommand(
                    new SimpleArmMove(ArmPositions.N_CONE_HOME, arm),
                    new SimpleArmMove(ArmPositions.N_CUBE_HOME, arm),
                    () -> oi.getGamePieceType()),
                new ConditionalCommand(
                    new CoordinatedArmMovePos(ArmPositions.N_CONE_GROUND_INTERMEDIATE, arm)
                        .andThen(new CoordinatedArmMove(ArmPositions.N_CONE_HOME, arm)),
                        new CoordinatedArmMove(ArmPositions.N_CUBE_HOME, arm)
                        .alongWith(
                            Commands.runOnce(drivetrain::endPrecisionMode)),
                    //new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND_INTERMEDIATE, arm)
                        //.andThen(new CoordinatedArmMove(ArmPositions.N_CUBE_HOME, arm)),
                    () -> oi.getGamePieceType()),
                () -> oi.getPickupLocation()));
    /*
        oi.getMoveToPickup()
            .onTrue(
                new ConditionalCommand(
                    new ConditionalCommand(
                        Commands.runOnce(gripper::intakeCube, gripper)
                        .alongWith (new MoveArm(ArmPositions.CONE_SHELF, arm)),
                        new MoveArm(ArmPositions.CUBE_SHELF, arm),
                        () -> oi.getGamePieceType()),
                    new ConditionalCommand(
                        new MoveArmToPos(ArmPositions.CONE_GROUND_INTERMEDIATE, arm)
                            .andThen(new MoveArm(ArmPositions.CONE_GROUND, arm)),
                        new MoveArmToPos(ArmPositions.CONE_GROUND_INTERMEDIATE, arm)
                            .andThen(new MoveArm(ArmPositions.CUBE_GROUND, arm)),
                        () -> oi.getGamePieceType()),
                    () -> oi.getPickupLocation()));

        oi.getMoveToPickup()
            .onFalse(
                new ConditionalCommand(
                    new ConditionalCommand(
                        Commands.runOnce(gripper::newIdleIntake, gripper).alongWith(
                            new MoveArm(ArmPositions.CONE_HOME, arm)
                        )
                        ,
                        Commands.runOnce(gripper::newIdleIntake, gripper).alongWith(
                        new MoveArm(ArmPositions.CUBE_HOME, arm)),
                        () -> oi.getGamePieceType()),
                    new ConditionalCommand(
                        new MoveArmToPos(ArmPositions.CONE_GROUND_INTERMEDIATE, arm)
                            .andThen(new MoveArm(ArmPositions.CONE_HOME, arm)),
                        new MoveArmToPos(ArmPositions.CUBE_GROUND_INTERMEDIATE, arm)
                            .andThen(new MoveArm(ArmPositions.CUBE_HOME, arm)),
                        () -> oi.getGamePieceType()),
                    () -> oi.getPickupLocation()));
    */

    /*
    oi.getMoveToHigh()
    .onTrue(
        new ConditionalCommand(
            new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE_WAY_UP, arm)
                .andThen(new MoveArm(ArmPositions.CONE_HIGH, arm)),
                //new MoveArm(ArmPositions.CUBE_HIGH_SHOT, arm),

                new MoveArmToPos(ArmPositions.CUBE_HIGH_INTERMEDIATE, arm)
               .andThen(new MoveArmN(ArmPositions.CUBE_HIGH, arm)),
            () -> oi.getGamePieceType()));

    oi.getMoveToHigh()
    .onFalse(
        new ConditionalCommand(
            new MoveArmToPosN(ArmPositions.CONE_MID_INTERMEDIATE, arm)
                .andThen(new MoveArm(ArmPositions.HOME, arm)),
                //new MoveArm(ArmPositions.HOME, arm),
            new MoveArmToPos(ArmPositions.CUBE_HIGH_INTERMEDIATE_WAY_DOWN, arm)
                .andThen(new MoveArm(ArmPositions.HOME, arm)),
            () -> oi.getGamePieceType()));

            */
    oi.getMoveToHigh()
        .onTrue(
            new ConditionalCommand(
                new CoordinatedArmMovePos(ArmPositions.N_CONE_HIGH_INTERMEDIATE, arm)
                    .andThen(new CoordinatedArmMove(ArmPositions.N_CONE_HIGH, arm)),
                new CoordinatedArmMovePos(ArmPositions.N_CUBE_HIGH_INTERMEDIATE, arm)
                    .andThen(new CoordinatedArmMove(ArmPositions.N_CUBE_HIGH, arm)),
                () -> oi.getGamePieceType()));

    oi.getMoveToHigh()
        .onFalse(
            new ConditionalCommand(
                new CoordinatedArmMovePos(ArmPositions.N_CONE_HIGH_INTERMEDIATE, arm)
                    .andThen(new CoordinatedArmMove(ArmPositions.N_HOME, arm)),
                new CoordinatedArmMovePos(ArmPositions.N_CUBE_HIGH_INTERMEDIATE, arm)
                    .andThen(new CoordinatedArmMove(ArmPositions.N_HOME, arm)),
                () -> oi.getGamePieceType()));
    /*
    oi.getHighScoreButton()
            .onTrue(
                new CoordinatedArmMovePos(ArmPositions.N_CONE_SHELF, arm)
                .andThen(new CoordinatedArmMove(ArmPositions.N_CONE_HIGH, arm)));

    oi.getHighScoreButton()
            .onFalse(
                new CoordinatedArmMovePos(ArmPositions.N_CONE_SHELF, arm)
                .andThen(new CoordinatedArmMove(ArmPositions.N_HOME, arm)));

                */
    /* oi.getHighScoreButton()
    .onTrue(
        new MoveArmToPos(ArmPositions.CONE_HIGH_SCORE, arm)
            .andThen(Commands.runOnce(gripper::releaseCube, gripper)
            .alongWith(new MoveArm(ArmPositions.CONE_HIGH_SCORE, arm)))
    );

    oi.getHighScoreButton()
    .onFalse(
        Commands.runOnce(gripper::opengrip, gripper)
        .andThen(Commands.runOnce(gripper::stopIntake, gripper))
        .andThen(new MoveArm(ArmPositions.CONE_MID_INTERMEDIATE, arm))
    );

    */
    /*
                oi.getMoveToMid()
                .onTrue(
                    new ConditionalCommand(
                        new MoveArm(ArmPositions.CONE_MID, arm),
                        new MoveArm(ArmPositions.CUBE_MID, arm),
                        () -> oi.getGamePieceType()));

            oi.getMoveToMid()
                .onFalse(
                    new ConditionalCommand(
                        new MoveArm(ArmPositions.HOME, arm),
                        new MoveArm(ArmPositions.HOME, arm),
                        () -> oi.getGamePieceType()));

            oi.getMidScoreButton()
                .onTrue(
                    new MoveArmToPos(ArmPositions.CONE_MID_SCORE, arm)
                    .andThen(Commands.runOnce(gripper::releaseCube, gripper)
                    .alongWith(new MoveArm(ArmPositions.CONE_MID_SCORE, arm)))
                );

            oi.getMidScoreButton()
            .onFalse(
                Commands.runOnce(gripper::opengrip, gripper)
                .andThen(Commands.runOnce(gripper::stopIntake, gripper))
                .andThen(new MoveArm(ArmPositions.HOME, arm))
                );

            oi.getMoveToLow()
                .onTrue(
                    new ConditionalCommand(
                        new MoveArmToPos(ArmPositions.CONE_LOW_INTERMEDIATE, arm)
                            .andThen(new MoveArm(ArmPositions.CONE_LOW, arm)),
                        new MoveArm(ArmPositions.CUBE_LOW, arm),
                        () -> oi.getGamePieceType()));

            oi.getMoveToLow()
                .onFalse(
                    new ConditionalCommand(
                        new MoveArm(ArmPositions.HOME, arm),
                        new MoveArm(ArmPositions.HOME, arm),
                        () -> oi.getGamePieceType()));

        oi.getMoveToMid()
        .onTrue(
            new ConditionalCommand(
                new SimpleArmMove(ArmPositions.N_CONE_MID, arm),
                new SimpleArmMove(ArmPositions.N_CUBE_MID, arm),
                () -> oi.getGamePieceType()));

    oi.getMoveToMid()
        .onFalse(
            new ConditionalCommand(
                new SimpleArmMove(ArmPositions.N_HOME, arm),
                new SimpleArmMove(ArmPositions.N_HOME, arm),
                () -> oi.getGamePieceType()));
                */
    oi.getMoveToMid()
        .onTrue(
            new ConditionalCommand(
                new CoordinatedArmMove(ArmPositions.N_CONE_MID, arm),
                new CoordinatedArmMove(ArmPositions.N_CUBE_MID, arm),
                () -> oi.getGamePieceType()));

    oi.getMoveToMid()
        .onFalse(
            new ConditionalCommand(
                new CoordinatedArmMove(ArmPositions.N_HOME, arm),
                new CoordinatedArmMove(ArmPositions.N_HOME, arm),
                () -> oi.getGamePieceType()));

    oi.getMoveToLow()
        .onTrue(
            new ConditionalCommand(
                new CoordinatedArmMove(ArmPositions.N_CONE_LOW, arm),
                new CoordinatedArmMove(ArmPositions.N_CUBE_LOW, arm),
                () -> oi.getGamePieceType()));

    oi.getMoveToLow()
        .onFalse(
            new ConditionalCommand(
                new CoordinatedArmMove(ArmPositions.N_HOME, arm),
                new CoordinatedArmMove(ArmPositions.N_HOME, arm),
                () -> oi.getGamePieceType()));
    /*
        oi.getOpBut8()
            .onTrue(
                Commands.runOnce(gripper::opengrip, gripper)
            );
    */

    oi.getSafetyStop()
        .onTrue(
            Commands.run(arm::allStop, arm)
                .alongWith(Commands.runOnce(gripper::stopIntake, gripper)));

    oi.getSafetyStop().onFalse(new ArmMoveHome(arm));

    // oi.getTestButton().onTrue(new DriveReset(drivetrain));
  }

  /*
    Command ConeHigh = Commands.sequence(
      new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE_WAY_UP, arm),
      new MoveArmToPos(ArmPositions.CONE_HIGH, arm),
      new MoveArmToPos(ArmPositions.CONE_HIGH_SCORE, arm),
      Commands.runOnce(gripper::releaseCube, gripper).withTimeout(1)
      .raceWith(new MoveArm(ArmPositions.CONE_HIGH_SCORE, arm)),
      new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE, arm),
      new MoveArm(ArmPositions.HOME, arm)
  );

  Command CubeHigh = Commands.sequence(
      new MoveArmToPos(ArmPositions.CUBE_HIGH_INTERMEDIATE, arm),
      new MoveArmToPos(ArmPositions.CUBE_HIGH, arm),
      Commands.runOnce(gripper::releaseCube, gripper).withTimeout(1)
      .raceWith(new MoveArm(ArmPositions.CUBE_HIGH, arm)),
      new MoveArmToPos(ArmPositions.CUBE_HIGH_INTERMEDIATE_WAY_DOWN, arm),
      new MoveArm(ArmPositions.HOME, arm)
  );

  Command MoveToCubeGround = Commands.sequence(
      new MoveArmToPos(ArmPositions.CUBE_GROUND_INTERMEDIATE,arm),
      new MoveArm(ArmPositions.CUBE_GROUND,arm)
  );

  Command PickupCubeGround = Commands.sequence(
      new MoveArm(ArmPositions.CUBE_GROUND,arm)
      .raceWith(Commands.runOnce(gripper::intakeCube, gripper).withTimeout(2.0)),
      new MoveArmToPos(ArmPositions.CUBE_GROUND_INTERMEDIATE, arm),
      new MoveArm(ArmPositions.HOME, arm)
  );
  */
  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {

    AUTO_EVENT_MAP.put("event1", Commands.print("passed marker 1"));
    AUTO_EVENT_MAP.put("event2", Commands.print("passed marker 2"));
    AUTO_EVENT_MAP.put("ExtendArm", Commands.print("Extending arm for ground pickup"));
    AUTO_EVENT_MAP.put("RunIntake", Commands.print("Running the intake wheels"));
    AUTO_EVENT_MAP.put("StopIntake", Commands.print("Stopping intake wheels"));
    AUTO_EVENT_MAP.put("CubeMid", Commands.print("Moving Arm to Cube Mid position"));
    AUTO_EVENT_MAP.put("CubeHigh", Commands.print("Moving Arm to Cube High position"));
    AUTO_EVENT_MAP.put("RunOutput", Commands.print("Shoot out the game piece"));
    // AUTO_EVENT_MAP.put("ExtendArmGroundInter",
    // Commands.MoveArmToPos(ArmPositions.CUBE_GROUND_INTERMEDIATE));
    List<PathPlannerTrajectory> simpleBarrierPathsB =
        PathPlanner.loadPathGroup(
            "Simple Barrier B",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    Command simpleConeBarrierB =
        Commands.sequence(
            Commands.run(gripper::closegrip, gripper).withTimeout(0.5),
            Commands.runOnce(gripper::stopIntake, gripper),
            new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE_WAY_UP, arm),
            new MoveArmToPos(ArmPositions.CONE_HIGH, arm),
            new MoveArmToPos(ArmPositions.CONE_HIGH_SCORE, arm),
            Commands.run(gripper::releaseCube, gripper)
                .withTimeout(1)
                .raceWith(new MoveArm(ArmPositions.CONE_HIGH_SCORE, arm)),
            Commands.runOnce(gripper::stopIntake, gripper),
            Commands.runOnce(gripper::opengrip, gripper),
            new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE, arm),
            new MoveArmToPos(ArmPositions.HOME, arm).withTimeout(2.5),
            new FollowPath(simpleBarrierPathsB.get(0), drivetrain, true)
            // new AutoMoveSpeed(1.2, drivetrain).withTimeout(2.5)
            );
    List<PathPlannerTrajectory> simpleBumpPathsB =
        PathPlanner.loadPathGroup(
            "Simple Bump B",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command simpleConeBumpB =
        Commands.sequence(
            Commands.run(gripper::closegrip, gripper).withTimeout(0.5),
            Commands.runOnce(gripper::stopIntake, gripper),
            new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE_WAY_UP, arm),
            new MoveArmToPos(ArmPositions.CONE_HIGH, arm),
            new MoveArmToPos(ArmPositions.CONE_HIGH_SCORE, arm),
            Commands.run(gripper::releaseCube, gripper)
                .withTimeout(1)
                .raceWith(new MoveArm(ArmPositions.CONE_HIGH_SCORE, arm)),
            Commands.runOnce(gripper::stopIntake, gripper),
            Commands.runOnce(gripper::opengrip, gripper),
            new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE, arm),
            new MoveArmToPos(ArmPositions.HOME, arm).withTimeout(2.5),
            new FollowPath(simpleBumpPathsB.get(0), drivetrain, true)
            // new AutoMoveSpeed(1.2, drivetrain).withTimeout(4.5)

            );

    /*
    List<PathPlannerTrajectory> Barrier2M =
    PathPlanner.loadPathGroup(
        "barrier2M",
        AUTO_MAX_SPEED_METERS_PER_SECOND,
        AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command barrier2M = Commands.sequence(
        Commands.run(gripper::closegrip, gripper).withTimeout(0.5),
        Commands.runOnce(gripper::stopIntake, gripper),
        new CoordinatedArmMovePos(ArmPositions.N_CUBE_MID, arm),
        Commands.run(gripper::releaseCube, gripper).withTimeout(1),
        Commands.runOnce(gripper::stopIntake, gripper),
        Commands.runOnce(gripper::opengrip, gripper),
        new CoordinatedArmMovePos(ArmPositions.N_HOME, arm).withTimeout(2.5),

            new FollowPath(Barrier2M.get(0), drivetrain, true),
            new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND_INTERMEDIATE,arm),
            new CoordinatedArmMove(ArmPositions.N_CUBE_GROUND,arm).withTimeout(2.0),

            new FollowPath(Barrier2M.get(1), drivetrain, false)
            .alongWith(
                new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND_INTERMEDIATE, arm),
            new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND,arm)
                .raceWith(Commands.run(gripper::intakeCube, gripper).withTimeout(2.0))),
            Commands.runOnce(gripper::stopIntake, gripper),
        new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND_INTERMEDIATE, arm),
        new CoordinatedArmMovePos(ArmPositions.N_HOME, arm).withTimeout(1.5),

        new FollowPath(Barrier2M.get(2), drivetrain, false),

        new FollowPath(Barrier2M.get(3), drivetrain, false),
        Commands.run(gripper::closegrip, gripper).withTimeout(0.5),
        Commands.runOnce(gripper::stopIntake, gripper),
        new CoordinatedArmMovePos(ArmPositions.N_CUBE_MID, arm),
        Commands.run(gripper::releaseCube, gripper).withTimeout(1),
        Commands.runOnce(gripper::stopIntake, gripper),
        Commands.runOnce(gripper::opengrip, gripper),
        new CoordinatedArmMovePos(ArmPositions.N_HOME, arm).withTimeout(2.5)

    );

    List<PathPlannerTrajectory> BarrierML =
    PathPlanner.loadPathGroup(
        "barrierML",
        AUTO_MAX_SPEED_METERS_PER_SECOND,
        AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command barrierML = Commands.sequence(
        Commands.run(gripper::closegrip, gripper).withTimeout(0.5),
        Commands.runOnce(gripper::stopIntake, gripper),
        new CoordinatedArmMovePos(ArmPositions.N_CUBE_MID, arm),
        Commands.run(gripper::releaseCube, gripper).withTimeout(1),
        Commands.runOnce(gripper::stopIntake, gripper),
        Commands.runOnce(gripper::opengrip, gripper),
        new CoordinatedArmMovePos(ArmPositions.N_HOME, arm).withTimeout(2.5),

        new FollowPath(BarrierML.get(0), drivetrain, true),

        new FollowPath(BarrierML.get(1), drivetrain, false)
        .alongWith(
            new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND_INTERMEDIATE, arm),
        new CoordinatedArmMove(ArmPositions.N_CUBE_GROUND,arm)
            .raceWith(Commands.run(gripper::intakeCube, gripper).withTimeout(2.0))),
        Commands.runOnce(gripper::stopIntake, gripper),
    new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND_INTERMEDIATE, arm),
    new CoordinatedArmMovePos(ArmPositions.N_HOME, arm).withTimeout(1.5),

    new FollowPath(BarrierML.get(2), drivetrain, false),
    new CoordinatedArmMovePos(ArmPositions.N_CUBE_LOW, arm),
    Commands.run(gripper::releaseCube, gripper).withTimeout(1),
    Commands.run(gripper::closegrip, gripper).withTimeout(0.5),
    new CoordinatedArmMovePos(ArmPositions.N_HOME, arm).withTimeout(2.5)

    );

    List<PathPlannerTrajectory> bumpML =
    PathPlanner.loadPathGroup(
        "bumpML",
        AUTO_MAX_SPEED_METERS_PER_SECOND,
        AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command BumpML = Commands.sequence(
        Commands.run(gripper::closegrip, gripper).withTimeout(0.5),
        Commands.runOnce(gripper::stopIntake, gripper),
        new CoordinatedArmMovePos(ArmPositions.N_HOME, arm),
        new CoordinatedArmMovePos(ArmPositions.N_CUBE_MID, arm),
        Commands.run(gripper::releaseCube, gripper).withTimeout(1),
        Commands.runOnce(gripper::stopIntake, gripper),
        Commands.runOnce(gripper::opengrip, gripper),
        new CoordinatedArmMovePos(ArmPositions.N_HOME, arm).withTimeout(2.5),

        new FollowPath(bumpML.get(0), drivetrain, true),

        new FollowPath(bumpML.get(1), drivetrain, false)
        .alongWith(
            new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND_INTERMEDIATE, arm),
        new CoordinatedArmMove(ArmPositions.N_CUBE_GROUND,arm)
            .raceWith(Commands.run(gripper::intakeCube, gripper).withTimeout(2.0))),
        Commands.runOnce(gripper::stopIntake, gripper),
    new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND_INTERMEDIATE, arm),
    new CoordinatedArmMovePos(ArmPositions.N_HOME, arm).withTimeout(1.5),

    new FollowPath(bumpML.get(2), drivetrain, false),
    new CoordinatedArmMovePos(ArmPositions.N_CUBE_LOW, arm),
    Commands.run(gripper::releaseCube, gripper).withTimeout(1),
    Commands.run(gripper::closegrip, gripper).withTimeout(0.5),
    new CoordinatedArmMovePos(ArmPositions.N_HOME, arm).withTimeout(2.5)
    );

    List<PathPlannerTrajectory> Bump2M =
    PathPlanner.loadPathGroup(
        "Bump2M",
        AUTO_MAX_SPEED_METERS_PER_SECOND,
        AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command bump2M = Commands.sequence(
        Commands.run(gripper::closegrip, gripper).withTimeout(0.5),
        Commands.runOnce(gripper::stopIntake, gripper),
        new CoordinatedArmMovePos(ArmPositions.N_CUBE_MID, arm),
        Commands.run(gripper::releaseCube, gripper).withTimeout(1),
        Commands.runOnce(gripper::stopIntake, gripper),
        Commands.runOnce(gripper::opengrip, gripper),
        new CoordinatedArmMovePos(ArmPositions.N_CUBE_MID, arm),
        new CoordinatedArmMovePos(ArmPositions.N_HOME, arm).withTimeout(2.5),

        new FollowPath(Bump2M.get(0), drivetrain, true),

        new FollowPath(Bump2M.get(1), drivetrain, false)
        .alongWith(
            new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND_INTERMEDIATE, arm),
        new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND,arm)
            .raceWith(Commands.run(gripper::intakeCube, gripper).withTimeout(2.0))),
        Commands.runOnce(gripper::stopIntake, gripper),
    new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND_INTERMEDIATE, arm),
    new CoordinatedArmMovePos(ArmPositions.N_HOME, arm).withTimeout(1.5),

    new FollowPath(Bump2M.get(2), drivetrain, false),
        new CoordinatedArmMovePos(ArmPositions.N_CUBE_MID, arm),
        Commands.run(gripper::releaseCube, gripper).withTimeout(1),
     Commands.runOnce(gripper::stopIntake, gripper),
     Commands.runOnce(gripper::opengrip, gripper),
     new CoordinatedArmMovePos(ArmPositions.N_CUBE_MID, arm),
     new CoordinatedArmMovePos(ArmPositions.N_HOME, arm).withTimeout(2.5)
    );*/

    List<PathPlannerTrajectory> simpleBarrierPathsR =
        PathPlanner.loadPathGroup(
            "Simple Barrier R",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    Command simpleConeBarrierR =
        Commands.sequence(
            /*   Commands.run(gripper::closegrip, gripper).withTimeout(0.5),
             Commands.runOnce(gripper::stopIntake, gripper),
             new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE_WAY_UP, arm),
             new MoveArmToPos(ArmPositions.CONE_HIGH, arm),
             new MoveArmToPos(ArmPositions.CONE_HIGH_SCORE, arm),
             Commands.run(gripper::releaseCube, gripper).withTimeout(1)
             .raceWith(new MoveArm(ArmPositions.CONE_HIGH_SCORE, arm)),
            Commands.runOnce(gripper::stopIntake, gripper),
             Commands.runOnce(gripper::opengrip, gripper),
             new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE, arm),
             new MoveArmToPos(ArmPositions.HOME, arm).withTimeout(2.5), */

            new FollowPath(simpleBarrierPathsR.get(0), drivetrain, true)
            // new AutoMoveSpeed(1.2, drivetrain).withTimeout(2.5)
            );
    List<PathPlannerTrajectory> simpleBumpPathsR =
        PathPlanner.loadPathGroup(
            "Simple Bump R",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command simpleConeBumpR =
        Commands.sequence(
            Commands.run(gripper::closegrip, gripper).withTimeout(0.5),
            Commands.runOnce(gripper::stopIntake, gripper),
            new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE_WAY_UP, arm),
            new MoveArmToPos(ArmPositions.CONE_HIGH, arm),
            new MoveArmToPos(ArmPositions.CONE_HIGH_SCORE, arm),
            Commands.run(gripper::releaseCube, gripper)
                .withTimeout(1)
                .raceWith(new MoveArm(ArmPositions.CONE_HIGH_SCORE, arm)),
            Commands.runOnce(gripper::stopIntake, gripper),
            Commands.runOnce(gripper::opengrip, gripper),
            new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE, arm),
            new MoveArmToPos(ArmPositions.HOME, arm).withTimeout(2.5),
            new FollowPath(simpleBumpPathsR.get(0), drivetrain, true)
            // new AutoMoveSpeed(1.2, drivetrain).withTimeout(4.5)

            );

    // build auto path commands
    List<PathPlannerTrajectory> auto1Paths =
        PathPlanner.loadPathGroup(
            "BarrierPath1cone1cube",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command autoTest =
        Commands.sequence(
            // ConeHigh,
            // Commands.run(gripper::closegrip, gripper).withTimeout(0.5),
            // Commands.runOnce(gripper::stopIntake, gripper),
            // new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE_WAY_UP, arm),
            // new MoveArmToPos(ArmPositions.CONE_HIGH, arm),
            // new MoveArmToPos(ArmPositions.CONE_HIGH_SCORE, arm),
            // Commands.run(gripper::releaseCube, gripper).withTimeout(1)
            // .raceWith(new MoveArm(ArmPositions.CONE_HIGH_SCORE, arm)),
            // Commands.runOnce(gripper::stopIntake, gripper),
            // Commands.runOnce(gripper::opengrip, gripper),
            // new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE, arm),
            // new MoveArmToPos(ArmPositions.HOME, arm).withTimeout(2.5),

            new FollowPath(auto1Paths.get(0), drivetrain, true),

            // MoveToCubeGround,
            // new MoveArmToPos(ArmPositions.CUBE_GROUND_INTERMEDIATE,arm),
            // new MoveArm(ArmPositions.CUBE_GROUND,arm).withTimeout(2.0),

            new FollowPath(auto1Paths.get(1), drivetrain, false)
            //  .alongWith(
            //   new MoveArm(ArmPositions.CUBE_GROUND,arm)
            //      .raceWith(Commands.run(gripper::intakeCube, gripper).withTimeout(2.0))),
            //  Commands.runOnce(gripper::stopIntake, gripper),
            // new MoveArmToPos(ArmPositions.CUBE_GROUND_INTERMEDIATE, arm),
            //  new MoveArmToPos(ArmPositions.HOME, arm).withTimeout(1.5)

            //   .alongWith(PickupCubeGround).withTimeout(5.0)
            ,
            new FollowPath(auto1Paths.get(2), drivetrain, false),
            //    new MoveArmToPos(ArmPositions.CUBE_HIGH_INTERMEDIATE, arm),
            // new MoveArmToPosN(ArmPositions.CUBE_HIGH, arm).withTimeout(1.0),
            //  Commands.run(gripper::releaseCube, gripper).withTimeout(1)
            //  .raceWith(new MoveArm(ArmPositions.CUBE_HIGH, arm)),
            //  Commands.runOnce(gripper::stopIntake, gripper),
            //  new MoveArmToPos(ArmPositions.CUBE_HIGH_INTERMEDIATE_WAY_DOWN, arm),
            //  new MoveArmToPos(ArmPositions.HOME, arm),
            //     CubeHigh,
            new FollowPath(auto1Paths.get(3), drivetrain, false));

    //    new FollowPathWithEvents(
    //          new FollowPath(auto1Paths.get(0), drivetrain, true),
    // auto1Paths.get(0).getMarkers(),
    ////          AUTO_EVENT_MAP),
    //      Commands.runOnce(drivetrain::enableXstance, drivetrain),
    //       Commands.waitSeconds(5.0),
    //       Commands.runOnce(drivetrain::disableXstance, drivetrain)); // ,
    // new FollowPathWithEvents(
    /// new FollowPath(auto1Paths.get(1), drivetrain, false),
    // auto1Paths.get(1).getMarkers(),
    // AUTO_EVENT_MAP));

    List<PathPlannerTrajectory> P_balanceAutoPaths =
        PathPlanner.loadPathGroup(
            "BalancePath",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command P_balanceAuto =
        Commands.sequence(
            //  CubeMid,
            new MoveArmToPos(ArmPositions.CUBE_MID, arm),
            Commands.run(gripper::releaseCube, gripper).withTimeout(1),
            Commands.runOnce(gripper::stopIntake, gripper),
            Commands.runOnce(gripper::opengrip, gripper),
            new MoveArmToPos(ArmPositions.HOME, arm).withTimeout(2),
            new FollowPath(P_balanceAutoPaths.get(0), drivetrain, true),
            new FollowPath(P_balanceAutoPaths.get(1), drivetrain, false),
            new AutoBalanceMove(drivetrain).withTimeout(4),
            Commands.runOnce(drivetrain::enableFieldRelative, drivetrain));

    List<PathPlannerTrajectory> P_balancePathwithPickupAutoPaths =
        PathPlanner.loadPathGroup(
            "BalancePathWithPickup",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command P_balanceWithAuto =
        Commands.sequence(
            // ConeHigh,
            new FollowPath(P_balancePathwithPickupAutoPaths.get(0), drivetrain, true),
            // MoveToCubeGround,
            new FollowPath(P_balancePathwithPickupAutoPaths.get(1), drivetrain, false),
            // .alongWith(PickupCubeGround).withTimeout(5.0)

            new FollowPath(P_balancePathwithPickupAutoPaths.get(2), drivetrain, false),
            new AutoBalanceMove(drivetrain).withTimeout(4));

    List<PathPlannerTrajectory> P_balancePathBarrier =
        PathPlanner.loadPathGroup(
            "BalanceBarrier",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command P_balanceWithAutoBarrier =
        Commands.sequence(

            // ConeHigh,

            Commands.run(gripper::closegrip, gripper).withTimeout(0.5),
            Commands.runOnce(gripper::stopIntake, gripper),
            new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE_WAY_UP, arm),
            new MoveArmToPos(ArmPositions.CONE_HIGH, arm),
            new MoveArmToPos(ArmPositions.CONE_HIGH_SCORE, arm),
            Commands.run(gripper::releaseCube, gripper)
                .withTimeout(1)
                .raceWith(new MoveArm(ArmPositions.CONE_HIGH_SCORE, arm)),
            Commands.runOnce(gripper::stopIntake, gripper),
            Commands.runOnce(gripper::opengrip, gripper),
            new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE, arm),
            new MoveArmToPos(ArmPositions.HOME, arm).withTimeout(2.5),
            new FollowPath(P_balancePathBarrier.get(0), drivetrain, true),
            new MoveArmToPos(ArmPositions.CUBE_GROUND_INTERMEDIATE, arm),
            new MoveArm(ArmPositions.CUBE_GROUND, arm).withTimeout(2.0),
            new FollowPath(P_balancePathBarrier.get(1), drivetrain, false)
                .alongWith(
                    new MoveArm(ArmPositions.CUBE_GROUND, arm)
                        .raceWith(Commands.run(gripper::intakeCube, gripper).withTimeout(2.0))),
            Commands.runOnce(gripper::stopIntake, gripper),
            new MoveArmToPos(ArmPositions.CUBE_GROUND_INTERMEDIATE, arm),
            new MoveArmToPos(ArmPositions.HOME, arm),
            new FollowPath(P_balancePathBarrier.get(2), drivetrain, false),
            new FollowPath(P_balancePathBarrier.get(3), drivetrain, false),
            new AutoBalanceMove(drivetrain).withTimeout(4));

    Command autoMidCube =
        // (new CoordinatedArmMovePos(ArmPositions.N_CUBE_HIGH_INTERMEDIATE, arm)
        // .raceWith(Commands.run(gripper::newIdleIntake, gripper)))
        // .andThen(
        new CoordinatedArmMovePos(ArmPositions.N_CUBE_MID, arm)
            .withTimeout(4.0)
            .raceWith(Commands.run(gripper::newIdleIntake, gripper))
            .andThen(Commands.run(gripper::releaseCube, gripper).withTimeout(1))
            .andThen(Commands.runOnce(gripper::stopIntake, gripper))
            // .andThen(new CoordinatedArmMovePos(ArmPositions.N_CONE_HIGH_INTERMEDIATE, arm))
            .andThen(new CoordinatedArmMovePos(ArmPositions.N_HOME, arm).withTimeout(2.0))
            .andThen(
                new CoordinatedArmMovePos(ArmPositions.N_HOME, arm)
                    .alongWith(new AutoMoveSpeed(-2.75, drivetrain).withTimeout(2.25)));

    Command balanceAutoMid =
        Commands.sequence(

            // new CoordinatedArmMovePos(ArmPositions.N_CUBE_HIGH_INTERMEDIATE, arm),
            // new CoordinatedArmMovePos(ArmPositions.N_CUBE_HIGH, arm),
            (new CoordinatedArmMovePos(ArmPositions.N_CONE_MID, arm).withTimeout(2.5))
                .raceWith(Commands.run(gripper::newIdleIntake, gripper)),
            Commands.run(gripper::releaseCube, gripper).withTimeout(1),
            Commands.runOnce(gripper::stopIntake, gripper),
            // Commands.runOnce(gripper::opengrip, gripper),
            new CoordinatedArmMovePos(ArmPositions.N_HOME, arm).withTimeout(2),
            new AutoMoveSpeed(-2.5, drivetrain).withTimeout(1.2),
            new AutoBalanceMove(drivetrain).withTimeout(8.0));

    Command turntest = new AutoBalanceMove(drivetrain).withTimeout(1.0);

    Command balanceAutoLeave =
        Commands.sequence(
            new CoordinatedArmMovePos(ArmPositions.N_CUBE_MID, arm),
            Commands.run(gripper::releaseCube, gripper).withTimeout(1),
            Commands.runOnce(gripper::stopIntake, gripper),
            Commands.runOnce(gripper::opengrip, gripper),
            new CoordinatedArmMovePos(ArmPositions.N_HOME, arm).withTimeout(2),
            new AutoMoveSpeed(-2.5, drivetrain).withTimeout(1.3),
            new AutoBalanceMove(drivetrain).withTimeout(1.5),
            new AutoMoveSpeed(-2.0, drivetrain).withTimeout(0.8),
            // new AutoTurnMove(drivetrain).withTimeout(1.0),
            new AutoMoveSpeed(2.5, drivetrain).withTimeout(1.0),
            new AutoBalanceMove(drivetrain).withTimeout(4));

    List<PathPlannerTrajectory> ChargeStation977Path =
        PathPlanner.loadPathGroup(
            "ChargeStation977",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command ChargeStationAuto =
        Commands.sequence(
            Commands.run(gripper::closegrip, gripper).withTimeout(0.5),
            Commands.runOnce(gripper::stopIntake, gripper),
            new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE_WAY_UP, arm),
            new MoveArmToPos(ArmPositions.CONE_HIGH, arm),
            new MoveArmToPos(ArmPositions.CONE_HIGH_SCORE, arm),
            Commands.run(gripper::releaseCube, gripper)
                .withTimeout(1)
                .raceWith(new MoveArm(ArmPositions.CONE_HIGH_SCORE, arm)),
            Commands.runOnce(gripper::stopIntake, gripper),
            Commands.runOnce(gripper::opengrip, gripper),
            new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE, arm),
            new MoveArmToPos(ArmPositions.HOME, arm).withTimeout(2.5),
            new FollowPath(ChargeStation977Path.get(0), drivetrain, true),
            // new AutoMoveSpeed(-2.5, drivetrain).withTimeout(1.2),

            new AutoBalanceMove(drivetrain).withTimeout(4));

    Command autoHighCone =
        new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE, arm)
            .andThen(new MoveArm(ArmPositions.CONE_HIGH, arm).withTimeout(2.0))
            .andThen(
                Commands.runOnce(gripper::releaseCube, gripper)
                    .withTimeout(1)
                    .andThen(new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE, arm))
                    .andThen(new MoveArm(ArmPositions.HOME, arm)));
    
        List<PathPlannerTrajectory> AutoNewCubeBarrier =
            PathPlanner.loadPathGroup(
                        "NewCubeBarrier",
                        AUTO_MAX_SPEED_METERS_PER_SECOND,
                        AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

//Ethan's Code certified approval team 997 The Comet bot
                    Command NewCubeBarrier =
                        Commands.sequence(

                        Commands.run(gripper::closegrip, gripper).withTimeout(0.4),
            Commands.runOnce(gripper::stopIntake, gripper),
            new CoordinatedArmMovePos(ArmPositions.N_CUBE_HIGH_INTERMEDIATE, arm),
            new CoordinatedArmMovePos(ArmPositions.N_CUBE_HIGH, arm),
            Commands.run(gripper::releaseCube, gripper)
                .withTimeout(0.25),
            Commands.runOnce(gripper::stopIntake, gripper),
            Commands.runOnce(gripper::opengrip, gripper),
            new FollowPath(AutoNewCubeBarrier.get(0), drivetrain, true)
            .alongWith(
                new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND_INTERMEDIATE, arm)),
                
            //drive back while its coming down 
            //changing stop point to not turn until after arm comes down

            new FollowPath(AutoNewCubeBarrier.get(1), drivetrain, false)
            .alongWith(
                new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND, arm)),
           

           // new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND, arm),
            new FollowPath(AutoNewCubeBarrier.get(2), drivetrain, false)
            .alongWith(
                
                new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND, arm)
                .alongWith(
                    Commands.run(gripper::closegrip, gripper).withTimeout(2.0)) 
                
            ),
            Commands.runOnce(gripper::newIdleIntake, gripper),
            new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND_INTERMEDIATE, arm),
            new CoordinatedArmMovePos(ArmPositions.N_HOME, arm)
                        );

                        List<PathPlannerTrajectory> AutoNewCubeBump =
                        PathPlanner.loadPathGroup(
                                    "NewCubeBump",
                                    AUTO_MAX_SPEED_METERS_PER_SECOND,
                                    AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
            
            //Ethan's Code certified approval team 997 The Comet bot
                                Command NewCubeBump =
                                    Commands.sequence(
            
                                    Commands.run(gripper::closegrip, gripper).withTimeout(0.4),
                        Commands.runOnce(gripper::stopIntake, gripper),
                        new CoordinatedArmMovePos(ArmPositions.N_CUBE_HIGH_INTERMEDIATE, arm),
                        new CoordinatedArmMovePos(ArmPositions.N_CUBE_HIGH, arm),
                        Commands.run(gripper::releaseCube, gripper)
                            .withTimeout(0.25),
                        Commands.runOnce(gripper::stopIntake, gripper),
                        Commands.runOnce(gripper::opengrip, gripper),
                        new CoordinatedArmMovePos(ArmPositions.N_CUBE_HIGH_INTERMEDIATE, arm),
                        new CoordinatedArmMovePos(ArmPositions.N_HOME, arm),
                        new FollowPath(AutoNewCubeBump.get(0), drivetrain, true),
                        new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND_INTERMEDIATE, arm),
                        new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND, arm),
                        new FollowPath(AutoNewCubeBump.get(1), drivetrain, false)
                        .alongWith(
                            Commands.run(gripper::closegrip, gripper).withTimeout(2.0)
                            
                        ),
                        Commands.runOnce(gripper::newIdleIntake, gripper),
                        new CoordinatedArmMovePos(ArmPositions.N_CUBE_GROUND_INTERMEDIATE, arm),
                        new CoordinatedArmMovePos(ArmPositions.N_HOME, arm)
                                    );
    // .andThen(Commands.runOnce(drivetrain::enableXstance, drivetrain)).withTimeout(1);
    // new MoveArmToPos(ArmPositions.CUBE_MID, arm)
    // .andThen(new MoveArmToPos(ArmPositions.HOME, arm))
    // .alongWith(new AutoMoveSpeed(1.2, drivetrain, gripper).withTimeout(1.0))

    // .andThen(new AutoBalanceMove(drivetrain, accelerometer)).withTimeout(3));

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    // PathPlannerTrajectory.transformStateForAlliance(null, null)
    // PathPlannerTrajectory.transformTrajectoryForAlliance(null, null)

    // autoChooser.addOption("Cone barrier side B", simpleConeBarrierB);
    // autoChooser.addOption("Cone bump side B", simpleConeBumpB);
    // autoChooser.addOption("Cone barrier side R", simpleConeBarrierR);
    // autoChooser.addOption("Cone bump side R", simpleConeBumpR);
    // autoChooser.addOption("Turntest", turntest);
    // autoChooser.addOption("Barrier2M", barrier2M);
    // autoChooser.addOption("barrierML", barrierML);
    // autoChooser.addOption("BumpML", BumpML);
    // autoChooser.addOption("Bump2M", bump2M);
    // autoChooser.addDefaultOption("Balance Barrier", P_balanceWithAutoBarrier);
    // autoChooser.addDefaultOption("Cube Mid Auto", auto2);

    // autoChooser.addOption("BalanceMID", balanceAutoMid);

    // autoChooser.addOption("Cone Then balance", ChargeStationAuto);
    // autoChooser.addOption("X:ConeCubeBarrierB", autoTest);
    // autoChooser.addOption("X:ConeOverBalanceRB",P_balanceAuto);
    autoChooser.addOption("balanceAutoLeave", balanceAutoLeave);

    // autoChooser.addOption("AutoCubeMidLeave", autoMidCube);
    autoChooser.addOption("NewCubeBarrier", NewCubeBarrier);
    autoChooser.addOption("NewCubeBump", NewCubeBump);
    

    // demonstration of PathPlanner path group with event markers
    // autoChooser.addOption("Test Path", autoTest);

    // "auto" command for tuning the drive velocity PID
    /*
    autoChooser.addOption(
        "Drive Velocity Tuning",
        Commands.sequence(
            Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
            Commands.deadline(
                Commands.waitSeconds(5.0),
                Commands.run(() -> drivetrain.drive(1.5, 0.0, 0.0), drivetrain))));

    // "auto" command for characterizing the drivetrain
    autoChooser.addOption(
        "Drive Characterization",
        new FeedForwardCharacterization(
            drivetrain,
            true,
            new FeedForwardCharacterizationData("drive"),
            drivetrain::runCharacterizationVolts,
            drivetrain::getCharacterizationVelocity));

            */

    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public static boolean safeToDriveFast() {
    SmartDashboard.putBoolean("Safe to Drive Fast", arm.safeToDriveFast());
    return arm.safeToDriveFast();
  }
  /*
  public boolean speedOverridePressed(){
    SmartDashboard.putBoolean("speed Mcde",oi.getSpeedOverrideL().getAsBoolean() || oi.getSpeedOverrideR().getAsBoolean());
    return oi.getSpeedOverrideL().getAsBoolean() || oi.getSpeedOverrideR().getAsBoolean();

  }

  private void setSpeedOverride(boolean state){
    speedOverride = state;

  }
  */

  public static double getAdjustJ3() {
    return adjustJ3;
  }

  public static void adjustJ3(double amount) {
    adjustJ3 = adjustJ3 + amount;
  }
}
