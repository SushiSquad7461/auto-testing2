package frc.robot;


import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;


import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.util.List;

public class Ramsete {
    private TrajectoryConfig config;
    private DifferentialDriveVoltageConstraint voltageConstraint;
    private Drive drive;
    private SimpleMotorFeedforward ramseteFF; 
    
    // path enums
    public enum RamsetePath {
        // insert auto paths
        
        // FORWARD("../../../../../PathWeaver/output/forward.wpilib.json"),
        // CURVE("../../../../../PathWeaver/output/curve.wpilib.json");
        FORWARD("paths/output/forward.wpilib.json"),
        CURVE("paths/output/curve.wpilib.json"),
        CIRCLE("paths/output/circle.wpilib.json"),
        L("paths/output/L.wpilib.json");

        private String json;
        RamsetePath(String json) {
            this.json = json;
        }

        public Trajectory getTrajectory() {
            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(json);
                return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                return new Trajectory(); 
            }
        }
    }

    public Ramsete(Drive drive) {
        this.drive = drive;
        ramseteFF = new SimpleMotorFeedforward(
            Constants.kDrive.ksVolts,
            Constants.kDrive.kvVoltSecondsPerMeter,
            Constants.kDrive.kaVoltSecondsSquaredPerMeter);
        config = new TrajectoryConfig(
            Constants.kDrive.MAX_SPEED_METERS_PER_SECOND,
            Constants.kDrive.MAX_ACCEL_METERS_PER_SECOND_SQUARED)
            .setKinematics(Constants.kDrive.DRIVE_KINEMATICS)
            .addConstraint(getVoltageConstraint());

        voltageConstraint = new DifferentialDriveVoltageConstraint(
            ramseteFF,
            Constants.kDrive.DRIVE_KINEMATICS,
            Constants.kDrive.MAX_VOLTAGE);
    }

    public SequentialCommandGroup createRamseteCommand(RamsetePath path) {
        return new RamseteCommand(
            path.getTrajectory(),
            drive::getPose,
            new RamseteController(
                Constants.kDrive.RAMSETE_B,
                Constants.kDrive.RAMSETE_ZETA),
            ramseteFF,
            Constants.kDrive.DRIVE_KINEMATICS,
            drive::getWheelSpeeds,
            new PIDController(
                Constants.kDrive.kPDriveVel,
                Constants.kDrive.kIDrive,
                Constants.kDrive.kDDrive),
            new PIDController(
                Constants.kDrive.kPDriveVel,
                Constants.kDrive.kIDrive,
                Constants.kDrive.kDDrive),
            drive::tankDriveVolts,
            drive)
            .andThen(() -> drive.tankDriveVolts(0, 0));
    }

    public DifferentialDriveVoltageConstraint getVoltageConstraint() {
        return voltageConstraint;
    }

}
