package frc.lib.util;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class PathPlannerLoader {
    public static SwerveAutoBuilder autoBuilder;
    public PathPlannerLoader() {
        System.out.println("you dont need to initialize this i think");
    }

    public static CommandBase LoadAutoFromString(Swerve dt, String name, PathConstraints constraints, HashMap<String, Command> eventMap) {
        PathPlannerTrajectory path = PathPlanner.loadPath(name, constraints);
        
        if (autoBuilder.equals(null)) {
            PIDConstants drivePID = new PIDConstants(SwerveConstants.driveKP, SwerveConstants.driveKI, SwerveConstants.driveKD);
            PIDConstants rotationPID = new PIDConstants(SwerveConstants.angleKP, SwerveConstants.angleKI, SwerveConstants.angleKD);
            autoBuilder = new SwerveAutoBuilder(
                dt::getPose,
                dt::resetOdometry,
                SwerveConstants.swerveKinematics,
                drivePID,
                rotationPID,
                dt::setModuleStates,
                eventMap,
                true,
                dt
            );
        }
        return autoBuilder.fullAuto(path);
    }
}
