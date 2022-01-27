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
import edu.wpi.first.networktables.NTSendableBuilder;
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
  private final Drive s_drive;
  
  // commands
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // auto stuff
  private final Ramsete ramsete;
  private final AutoCommandSelector autoSelector;
  private final Field2d field = new Field2d();
  public Trajectory exampleTrajectory;

  // auto chooser
  private SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<>();;

  public RobotContainer() {


    s_drive = new Drive();
    ramsete = new Ramsete(s_drive);
    autoSelector = new AutoCommandSelector(s_drive, ramsete);
    
    // put field object to dashboard
    SmartDashboard.putData("field", field);

    // set up chooser
    autoChooser.setDefaultOption("forward", autoSelector.forward);
    autoChooser.addOption("forward", autoSelector.forward);
    autoChooser.addOption("curve", autoSelector.curve);
    autoChooser.addOption("test", autoSelector.test);
    autoChooser.addOption("circle", autoSelector.circle);
    autoChooser.addOption("L", autoSelector.L);

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

  public void setFieldTrajectory() {
    Trajectory concatTrajectory = new Trajectory();
    for(RamsetePath p : autoSelector.firstTrajectoryMap.get(autoChooser.getSelected())) {
      concatTrajectory = concatTrajectory.concatenate(p.getTrajectory());
    }
    field.getObject(Constants.kOI.TRAJECTORY_NAME).setTrajectory(concatTrajectory);
  }

  public void setInitialPose() {
    autoSelector.setInitialDrivePose(autoChooser.getSelected());
  }
}
