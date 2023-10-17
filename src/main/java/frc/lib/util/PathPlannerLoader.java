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
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class PathPlannerLoader {
    public static SwerveAutoBuilder autoBuilder;

    public static CommandBase LoadAutoFromString(Swerve dt, String name, PathConstraints constraints, HashMap<String, Command> eventMap) {
        PathPlannerTrajectory path = PathPlanner.loadPath(name, constraints);
        
        if (autoBuilder.equals(null)) {
            PIDConstants drivePID = new PIDConstants(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);
            PIDConstants rotationPID = new PIDConstants(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD);
            autoBuilder = new SwerveAutoBuilder(
                dt::getPose,
                dt::resetOdometry,
                Constants.Swerve.swerveKinematics,
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
