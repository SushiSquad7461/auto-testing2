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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ramsete.RamsetePath;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
  // subsystems
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drive s_drive = new Drive();
  

  // commands
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // auto stuff
  private final Ramsete ramsete = new Ramsete(s_drive);
  private final AutoCommandSelector autoSelector = new AutoCommandSelector(s_drive, ramsete);
  private final Field2d field = new Field2d();
  public Trajectory exampleTrajectory;

  // auto chooser
  private SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<>();;

  public RobotContainer() {
    // put field object to dashboard
    SmartDashboard.putData("field", field);

    // set up chooser
    //autoChooser.setDefaultOption("no auto", null);
    autoChooser.addOption("forward", autoSelector.forward);
    autoChooser.addOption("curve", autoSelector.curve);
    autoChooser.addOption("test", autoSelector.test);

    autoSelector.setInitialDrivePose(autoChooser.getSelected());

    for(RamsetePath p: Ramsete.RamsetePath.values()) {
      field.getObject(p.toString()).setTrajectory(p.getTrajectory());
    }

    SmartDashboard.putData("auto options", autoChooser);
    
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void updateRobotPose() {
    field.setRobotPose(s_drive.getPose());
  }
}
