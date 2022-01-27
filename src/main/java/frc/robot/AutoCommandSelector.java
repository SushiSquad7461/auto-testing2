package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.fasterxml.jackson.databind.introspect.DefaultAccessorNamingStrategy.FirstCharBasedValidator;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Ramsete.RamsetePath;
import frc.robot.subsystems.Drive;


public class AutoCommandSelector {
    // necessary dependencies
    private final Drive drive;
    private final Ramsete ramsete;

    // sequential command groups
    public final SequentialCommandGroup test;
    public final SequentialCommandGroup forward;
    public final SequentialCommandGroup curve;
    public final SequentialCommandGroup circle;
    public final SequentialCommandGroup L;

    public final RamsetePath[] testPath = { Ramsete.RamsetePath.FORWARD, Ramsete.RamsetePath.CURVE };
    public final RamsetePath[] forwardPath = { Ramsete.RamsetePath.FORWARD }; 
    public final RamsetePath[] curvePath = { Ramsete.RamsetePath.CURVE };
    public final RamsetePath[] circlePath = { Ramsete.RamsetePath.CIRCLE };
    public final RamsetePath[] LPath = { Ramsete.RamsetePath.L };

    public final Map<SequentialCommandGroup, RamsetePath[]> firstTrajectoryMap;

    public AutoCommandSelector(Drive drive, Ramsete ramsete) {
        // instantiate dependencies
        this.drive = drive;
        this.ramsete = ramsete;

        this.firstTrajectoryMap = new HashMap<SequentialCommandGroup, RamsetePath[]>();

        // create command groups
        test = new SequentialCommandGroup(
            ramsete.createRamseteCommand(Ramsete.RamsetePath.FORWARD),
            ramsete.createRamseteCommand(Ramsete.RamsetePath.CURVE));
        forward = new SequentialCommandGroup(ramsete.createRamseteCommand(Ramsete.RamsetePath.FORWARD));
        curve = new SequentialCommandGroup(ramsete.createRamseteCommand(Ramsete.RamsetePath.CURVE));
        circle = new SequentialCommandGroup(ramsete.createRamseteCommand(Ramsete.RamsetePath.CIRCLE));
        L = new SequentialCommandGroup(ramsete.createRamseteCommand(Ramsete.RamsetePath.L));
        
        // trajectory map
        firstTrajectoryMap.put(test, testPath);
        firstTrajectoryMap.put(forward, forwardPath);
        firstTrajectoryMap.put(curve, curvePath);
        firstTrajectoryMap.put(circle, circlePath);
        firstTrajectoryMap.put(L, LPath);
    }

    public void setInitialDrivePose(SequentialCommandGroup auto) {
        if(firstTrajectoryMap.containsKey(auto)) {
            drive.setOdometry(firstTrajectoryMap.get(auto)[0].getTrajectory());
        }  
    }
}