// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import org.opencv.features2d.FlannBasedMatcher;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  
  private final WPI_TalonFX frontLeft, frontRight, backLeft, backRight;
  private final AHRS nav;

  private double zeroOffset = 0; // navx has problems with zeroing, so we save the initial
                                 //   heading at subsystem construction and subtract it
                                 //   from the reported heading when we need to retrieve

  private final DifferentialDrive diffDrive;
  private final DifferentialDriveOdometry odometry;
  boolean navZeroed = false;

  public Drive() {

    // motor setup and config
    frontLeft = new WPI_TalonFX(Constants.kDrive.FRONT_LEFT_ID);
    frontRight = new WPI_TalonFX(Constants.kDrive.FRONT_RIGHT_ID);
    backLeft = new WPI_TalonFX(Constants.kDrive.BACK_LEFT_ID);
    backRight = new WPI_TalonFX(Constants.kDrive.BACK_RIGHT_ID);

    frontLeft.configFactoryDefault();
    frontRight.configFactoryDefault();
    backLeft.configFactoryDefault();
    backRight.configFactoryDefault();

    frontLeft.setInverted(TalonFXInvertType.CounterClockwise);
    frontRight.setInverted(TalonFXInvertType.Clockwise);
    backLeft.setInverted(TalonFXInvertType.CounterClockwise);
    backRight.setInverted(TalonFXInvertType.Clockwise);

    backLeft.follow(frontLeft);
    backRight.follow(frontRight); 

    frontLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);
    
    diffDrive = new DifferentialDrive(frontLeft, frontRight);

    nav = new AHRS(SPI.Port.kMXP);
    nav.enableBoardlevelYawReset(false); // enables software-based reset for navx;
                                         //   doesn't really work

    // odometry stuff
    odometry = new DifferentialDriveOdometry(nav.getRotation2d());
    resetOdometry(new Pose2d());
  }

  @Override
  public void periodic() {
    // once navx finishes calibrating and we haven't already set a zero offset
    //   we set the offset to the current (initial) heading and set navZeroed
    //   to true so we don't accidentally reset the offset during the match
    if (!nav.isCalibrating() && !navZeroed) {
      zeroOffset = nav.getYaw();
      navZeroed = true;
    }

    // the navx's positive direction is clockwise whereas wpilib's is counter-clockwise, so
    //   we have to negate the output of getYaw
    odometry.update(new Rotation2d(Math.toRadians(-(nav.getYaw() - zeroOffset))), 
                    frontLeft.getSelectedSensorPosition() * Constants.kDrive.TICKS_TO_METERS,
                    frontRight.getSelectedSensorPosition() * Constants.kDrive.TICKS_TO_METERS);

    SmartDashboard.putNumber("heading", -(nav.getYaw() - zeroOffset));
    SmartDashboard.putNumber("odometry x", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("odometry y", odometry.getPoseMeters().getY());  }

  @Override
  public void simulationPeriodic() { }

  // get current robot position
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  // return current wheel speeds
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds( 
      frontLeft.getSelectedSensorVelocity(), 
      frontRight.getSelectedSensorVelocity()
    );
  }

  // reset odometry to given pose
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, new Rotation2d(-(nav.getYaw()-zeroOffset)));
  }

  public void setOdometry(Trajectory traj) {
    odometry.resetPosition(traj.getInitialPose(), traj.getInitialPose().getRotation());
  }

  // zero encoders
  public void resetEncoders() {
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeft.setVoltage(leftVolts);
    frontRight.setVoltage(rightVolts);
    diffDrive.feed();
  }

  public double getAverageEncoderDistance() {
    return (frontLeft.getSelectedSensorPosition() + frontRight.getSelectedSensorPosition()) / 2.0;
  }

  // scales maximum drive speed (0 to 1.0)
  public void setMaxOutput(double maxOutput) {
    diffDrive.setMaxOutput(maxOutput);
  }

  // zero navx (doesn't work consistently so we avoid using)
  public void zeroHeading() {
    nav.reset();
  }

  // return heading in degrees (-180 to 180)
  public double getHeading() {
    return nav.getYaw();
    // note: getAngle returns accumulated yaw (can be <0 or >360)
    //   getYaw has a 360 degree period
  }

  // return turn rate deg/sec
  public double getTurnRate() {
    // negative since navx's positive direction is opposite of the expected/wpilib standard
    return -nav.getRate();
  }
}
