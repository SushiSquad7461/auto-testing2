package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.fasterxml.jackson.databind.SequenceWriter;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;


public class AutoCommandSelector {
    // necessary dependencies
    private final Drive drive;
    private final Ramsete ramsete;

    // sequential command groups
    public final SequentialCommandGroup test;
    public final SequentialCommandGroup forward;
    public final SequentialCommandGroup curve;

    public final Map<SequentialCommandGroup, Trajectory> firstTrajectoryMap;

    public AutoCommandSelector(Drive drive, Ramsete ramsete) {
        // instantiate dependencies
        this.drive = drive;
        this.ramsete = ramsete;

        this.firstTrajectoryMap = new HashMap<SequentialCommandGroup, Trajectory>();
        
        System.out.println("slkdfa");
        // create command groups
        test = new SequentialCommandGroup(
            ramsete.createRamseteCommand(Ramsete.RamsetePath.FORWARD),
            ramsete.createRamseteCommand(Ramsete.RamsetePath.CURVE));
        forward = new SequentialCommandGroup(ramsete.createRamseteCommand(Ramsete.RamsetePath.FORWARD));
        curve = new SequentialCommandGroup(ramsete.createRamseteCommand(Ramsete.RamsetePath.CURVE));
        

        // trajectory map
        firstTrajectoryMap.put(test, Ramsete.RamsetePath.FORWARD.getTrajectory());
        firstTrajectoryMap.put(forward, Ramsete.RamsetePath.FORWARD.getTrajectory());
        firstTrajectoryMap.put(curve, Ramsete.RamsetePath.CURVE.getTrajectory());
    }

    public void setInitialDrivePose(SequentialCommandGroup auto) {
        if(firstTrajectoryMap.containsKey(auto)) {
            drive.setOdometry(firstTrajectoryMap.get(auto));
        }  
    }
}