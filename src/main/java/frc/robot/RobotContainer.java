package frc.robot;

import com.pathplanner.lib.auto.SwerveAutoBuilder;
import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.PathPlannerLoader;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Subsystems */
    private final SwerveSubsystem s_Swerve = new SwerveSubsystem();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();
    private final ArmSubsystem s_Arm = new ArmSubsystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> driver.leftBumper().getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    private void configureButtonBindings() {
        /* Drivetrain*/
        driver.back().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        driver.start().onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        
        driver.leftBumper().onTrue(new HeadingCorrectionDrive(
            s_Swerve, 
            180.00,
            () -> driver.getRawAxis(translationAxis), 
            () -> driver.getRawAxis(strafeAxis)
        ));

        // Arm
        //driver.povUp().whileTrue(s_Arm.wristUp());
        //driver.povDown().whileTrue(s_Arm.wristDown());
        driver.povUp().whileTrue(s_Arm.wristUpVolts());
        driver.povDown().whileTrue(s_Arm.wristDownVolts());

        // Intake
        driver.rightTrigger().whileTrue(Commands.parallel(s_Intake.intake(), s_Arm.wristDownEndUpVolts()));
        driver.leftTrigger().whileTrue(s_Intake.l1Shoot());

        driver.a().whileTrue(s_Intake.l1Shoot());
        driver.b().whileTrue(s_Intake.l2Shoot());
        driver.x().whileTrue(s_Intake.l2Shoot());
        driver.y().whileTrue(s_Intake.l3Shoot());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return PathPlannerLoader.LoadAutoFromString(s_Swerve, "Path1", new PathConstraints(1, 0.5), new HashMap<>());
    }
}
