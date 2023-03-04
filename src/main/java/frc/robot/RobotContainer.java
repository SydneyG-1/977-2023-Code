// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOPigeon2;
import frc.lib.team3061.pneumatics.Pneumatics;
import frc.lib.team3061.pneumatics.PneumaticsIO;
import frc.lib.team3061.pneumatics.PneumaticsIORev;
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
import frc.robot.commands.AutoMove1;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.FollowPath;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArmToPos;
import frc.robot.commands.SetGrip;
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
  private static boolean speedOverride = false;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();

  /** Create the container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
            new Pneumatics(new PneumaticsIORev());
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
            new Pneumatics(new PneumaticsIO() {});
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
      new Pneumatics(new PneumaticsIO() {});
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



    //arm.setDefaultCommand(
      //new MoveArm(ArmPositions.HOME, arm));

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
    oi.getFieldRelativeButton()
        .toggleOnTrue(
            Commands.either(
                Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
                Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
                drivetrain::getFieldRelative));

    // reset gyro to 0 degrees
    oi.getResetGyroButton().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    // x-stance
    oi.getXStanceButton().onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    oi.getXStanceButton().onFalse(Commands.runOnce(drivetrain::disableXstance, drivetrain));

    oi.getSpeedOverrideL().onTrue(Commands.runOnce(drivetrain::setSpeedOverride));
    oi.getSpeedOverrideL().onFalse(Commands.runOnce(drivetrain::endSpeedOverride));

    //intake & gripper controls
    oi.getCloseButton().onTrue(
      new ConditionalCommand( 
        Commands.runOnce(gripper::closegrip, gripper), 
        Commands.runOnce(gripper::intakeCube, gripper), 
        ()->oi.getGamePieceType()));
    oi.getCloseButton().onFalse(
      Commands.runOnce(gripper::stopIntake, gripper));
      
    oi.getOpenButton().onTrue(
      new ConditionalCommand(
        Commands.runOnce(gripper::opengrip, gripper),
        Commands.runOnce(gripper::releaseCube, gripper), 
        ()->oi.getGamePieceType()));
    oi.getOpenButton().onFalse(
      Commands.runOnce(gripper::stopIntake, gripper));
    
    oi.getMoveToPickup().onTrue(
      new ConditionalCommand(
        new ConditionalCommand(
          new MoveArm(ArmPositions.CONE_SHELF, arm),
          new MoveArm(ArmPositions.CUBE_SHELF, arm),
          ()-> oi.getGamePieceType()), 
        new ConditionalCommand(
          new MoveArm(ArmPositions.CONE_GROUND,arm),
          new MoveArm(ArmPositions.CUBE_GROUND,arm),
          ()-> oi.getGamePieceType()), 
        ()->oi.getPickupLocation()
      )
    );
    
    oi.getMoveToPickup().onFalse(
      new ConditionalCommand(
        new ConditionalCommand(
          new MoveArm(ArmPositions.CONE_HOME, arm),
          new MoveArm(ArmPositions.CUBE_HOME, arm),
          ()-> oi.getGamePieceType()), 
        new ConditionalCommand(
          new MoveArmToPos(ArmPositions.CONE_GROUND_INTERMEDIATE, arm).andThen(new MoveArm(ArmPositions.CONE_HOME,arm)),
          new MoveArm(ArmPositions.CUBE_HOME,arm),
          ()-> oi.getGamePieceType()), 
        ()->oi.getPickupLocation()
      )
    );


    oi.getMoveToHigh().onTrue(
      new ConditionalCommand(
        new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE, arm).andThen(new MoveArm(ArmPositions.CONE_HIGH, arm)),
        new MoveArmToPos(ArmPositions.CUBE_MID_INTERMEDIATE, arm).andThen(new MoveArm(ArmPositions.CUBE_HIGH, arm)),
        ()->oi.getGamePieceType()
        )
    );
    
    oi.getMoveToHigh().onFalse(
      new ConditionalCommand(
        new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE, arm).andThen(new MoveArm(ArmPositions.HOME, arm)),
        new MoveArmToPos(ArmPositions.CUBE_MID_INTERMEDIATE, arm).andThen(new MoveArm(ArmPositions.HOME, arm)),
        ()->oi.getGamePieceType()
        )
    );

    oi.getMoveToMid().onTrue(
      new ConditionalCommand(
        new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE, arm).andThen(new MoveArm(ArmPositions.CONE_MID, arm)),
        new MoveArm(ArmPositions.CUBE_MID, arm),
        ()->oi.getGamePieceType()
        )
    );
    
    oi.getMoveToMid().onFalse(
      new ConditionalCommand(
        new MoveArmToPos(ArmPositions.CONE_MID_INTERMEDIATE, arm).andThen(new MoveArm(ArmPositions.HOME, arm)),
        new MoveArm(ArmPositions.HOME, arm),
        ()->oi.getGamePieceType()
        )
    );
    
    oi.getMoveToLow().onTrue(
      new ConditionalCommand(
        new MoveArm(ArmPositions.CONE_LOW, arm),
        new MoveArm(ArmPositions.CUBE_LOW, arm),
        ()->oi.getGamePieceType()
        )
    );
    
    oi.getMoveToLow().onFalse(
      new ConditionalCommand(
        new MoveArm(ArmPositions.HOME, arm),
        new MoveArm(ArmPositions.HOME, arm),
        ()->oi.getGamePieceType()
        )
    );

    oi.getSafetyStop().onTrue(Commands.run(arm::allStop, arm));
    oi.getSafetyStop().onFalse((new MoveArm(ArmPositions.HOME, arm)));
  }


  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    AUTO_EVENT_MAP.put("event1", Commands.print("passed marker 1"));
    AUTO_EVENT_MAP.put("event2", Commands.print("passed marker 2"));

    // build auto path commands
    List<PathPlannerTrajectory> auto1Paths =
        PathPlanner.loadPathGroup(
            "StraightAhead",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command autoTest = 
        Commands.sequence(
            new FollowPathWithEvents(
                new FollowPath(auto1Paths.get(0), drivetrain, true),
                auto1Paths.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(5.0),
            Commands.runOnce(drivetrain::disableXstance, drivetrain),
            new FollowPathWithEvents(
                new FollowPath(auto1Paths.get(1), drivetrain, false),
                auto1Paths.get(1).getMarkers(),
                AUTO_EVENT_MAP));

    Command auto2 = new MoveArmToPos(ArmPositions.CUBE_MID, arm).andThen(new MoveArmToPos(ArmPositions.HOME, arm).alongWith( new AutoMove1(drivetrain, gripper).withTimeout(3)));

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    autoChooser.addOption("MoveStraight", new AutoMove1(drivetrain, gripper).withTimeout(4)
    //Commands.runOnce(gripper::releaseCube, gripper).withTimeout(2).andThen(Commands.runOnce(gripper::stopIntake, gripper)).andThen( new AutoMove1(drivetrain).withTimeout(3) 
     );
     autoChooser.addDefaultOption("Cube Mid Auto", auto2);

    

    // demonstration of PathPlanner path group with event markers
    autoChooser.addOption("Test Path", autoTest);

    // "auto" command for tuning the drive velocity PID
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

  public static boolean safeToDriveFast(){
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

  public static double getAdjustJ3(){
    return adjustJ3;
  }

  public static void adjustJ3(double amount){
    adjustJ3 = adjustJ3 + amount;
  }
}
