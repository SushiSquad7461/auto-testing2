// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drive s_drive = new Drive();

  private final Field2d field = new Field2d();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public Trajectory exampleTrajectory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData("field", field);
    // Configure the button bindings
    configureButtonBindings();
  }

  public void updateRobotPose() {
    field.setRobotPose(s_drive.getPose());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // create voltage constraint
    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
          Constants.kDrive.ksVolts, 
          Constants.kDrive.kvVoltSecondsPerMeter,
          Constants.kDrive.kaVoltSecondsSquaredPerMeter), 
        Constants.kDrive.kDriveKinematics, 
        Constants.kDrive.kMaxVoltage);

    // create trajectory config
    TrajectoryConfig config =
      new TrajectoryConfig(Constants.kDrive.kMaxSpeedMetersPerSecond,
                           Constants.kDrive.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.kDrive.kDriveKinematics)
        .addConstraint(autoVoltageConstraint);

    // example trajectory
    exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // start at origin
      new Pose2d(0, 0, new Rotation2d(0)),
      // make s curve
      List.of(
        new Translation2d(1, 1),
        new Translation2d(2, -1)
      ),
      // end 3 meters ahead, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      config
    );
    field.getObject("traj").setTrajectory(exampleTrajectory);

    RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory, 
      s_drive::getPose, 
      new RamseteController(Constants.kDrive.kRamseteB,
                            Constants.kDrive.kRamseteZeta), 
      new SimpleMotorFeedforward(Constants.kDrive.ksVolts, 
                                 Constants.kDrive.kvVoltSecondsPerMeter,
                                 Constants.kDrive.kaVoltSecondsSquaredPerMeter), 
      Constants.kDrive.kDriveKinematics, 
      s_drive::getWheelSpeeds, 
      new PIDController(Constants.kDrive.kPDriveVel, 0, 0), 
      new PIDController(Constants.kDrive.kPDriveVel, 0, 0), // might be good to make this a diff constant 
      s_drive::tankDriveVolts,
      s_drive
    );

    s_drive.resetOdometry(exampleTrajectory.getInitialPose());

    return ramseteCommand.andThen(() -> s_drive.tankDriveVolts(0, 0));

  }
}
