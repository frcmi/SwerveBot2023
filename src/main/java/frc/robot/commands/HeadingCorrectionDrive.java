package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;


public class HeadingCorrectionDrive extends CommandBase {    
    private SwerveSubsystem s_Swerve;    
    private double heading;
    private PIDController headingController;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;

    public HeadingCorrectionDrive(SwerveSubsystem s_Swerve, double heading, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;

        this.heading = heading;
        headingController = new PIDController(.05, 0, 0);
    }

    @Override
    public void execute() {
        
        /* Drive */
        //s_Swerve.drive(new Translation2d(3,0), headingController.calculate(s_Swerve.getYaw().getDegrees(), heading), false, false);
        
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), SwerveConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), SwerveConstants.stickDeadband);

        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), 
            headingController.calculate(s_Swerve.getYaw().getDegrees(), heading), 
            false, 
            true
        );
    }
}