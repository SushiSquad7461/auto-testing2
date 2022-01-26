// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drive;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

<<<<<<< HEAD
=======
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
>>>>>>> 5ddf68ccbaf868b302ec25cee0b53b31003c0a40
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // update odometry visualizer 
    m_robotContainer.updateRobotPose();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
<<<<<<< HEAD
<<<<<<< HEAD
  public void teleopPeriodic() {}
=======
  public void teleopPeriodic() { }
>>>>>>> 897db6a (added choosing autos to code)
=======
  public void teleopPeriodic() { }
>>>>>>> 5ddf68ccbaf868b302ec25cee0b53b31003c0a40

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
