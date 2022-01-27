// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import org.opencv.features2d.FlannBasedMatcher;

// import com.revrobotics.CANEncoder;
// import com.revrobotics.CANSparkMax;
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
  
  //private final CANSparkMax frontLeft, frontRight, backLeft, backRight;
  private final WPI_TalonFX frontLeft, frontRight, backLeft, backRight;
  //private final CANEncoder leftEncoder, rightEncoder;
  private final AHRS nav;

  private double zeroOffset = 5;

  private final DifferentialDrive diffDrive;
  private final DifferentialDriveOdometry odometry;
  boolean navZeroed = false;


  public Drive() {

    // configuration
    /*frontLeft = new CANSparkMax(Constants.kDrive.FRONT_LEFT_ID, Constants.kDrive.MOTOR_TYPE);
    frontRight = new CANSparkMax(Constants.kDrive.FRONT_RIGHT_ID, Constants.kDrive.MOTOR_TYPE);
    backLeft = new CANSparkMax(Constants.kDrive.BACK_LEFT_ID, Constants.kDrive.MOTOR_TYPE);
    backRight = new CANSparkMax(Constants.kDrive.BACK_RIGHT_ID, Constants.kDrive.MOTOR_TYPE);

    frontLeft.restoreFactoryDefaults();
    frontRight.restoreFactoryDefaults();
    backLeft.restoreFactoryDefaults();
    backRight.restoreFactoryDefaults();

    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    leftEncoder = frontLeft.getEncoder();
    rightEncoder = frontRight.getEncoder();*/

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
    nav.enableBoardlevelYawReset(false);
    

    // odometry stuff
    odometry = new DifferentialDriveOdometry(nav.getRotation2d());
    resetOdometry(new Pose2d());
  }

  @Override
  public void periodic() {
    if (!nav.isCalibrating() && !navZeroed) {
      zeroOffset = nav.getYaw();
      navZeroed = true;
    }

    odometry.update( new Rotation2d(Math.toRadians(-(nav.getYaw() - zeroOffset))), 
                    /*leftEncoder.getPosition(), 
                    rightEncoder.getPosition());*/
                    frontLeft.getSelectedSensorPosition() * Constants.kDrive.TICKS_TO_METERS,
                    frontRight.getSelectedSensorPosition() * Constants.kDrive.TICKS_TO_METERS);

                    // nav.reset();
    SmartDashboard.putNumber("gyro output", -(nav.getYaw() - zeroOffset));
    SmartDashboard.putNumber("zeroOffset", zeroOffset);
    SmartDashboard.putNumber("odometry x", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("odometry y", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Encoder val ", frontLeft.getSelectedSensorPosition());
    SmartDashboard.putBoolean("calb ", nav.isCalibrating());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // get current robot position
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  // return current wheel speeds
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    //return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    return new DifferentialDriveWheelSpeeds(frontLeft.getSelectedSensorVelocity(), frontRight.getSelectedSensorVelocity());
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
    /*leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);*/
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
  }

  // we might have to make sure setVoltage works the way we expect it to
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeft.setVoltage(leftVolts);
    frontRight.setVoltage(rightVolts);
    diffDrive.feed();
  }

  public double getAverageEncoderDistance() {
    //return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    return (frontLeft.getSelectedSensorPosition() + frontRight.getSelectedSensorPosition()) / 2.0;
  }

  /*public CANEncoder getLeftEncoder() {
    return leftEncoder;
  }

  public CANEncoder getRightEncoder() {
    return rightEncoder;
  }*/

  // scales maximum drive speed (0 to 1.0)
  public void setMaxOutput(double maxOutput) {
    diffDrive.setMaxOutput(maxOutput);
  }

  // zero navx
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
    // note: not sure why this is neg but docs said so
    return -nav.getRate();
  }
}
