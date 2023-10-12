package frc.robot;

import com.pathplanner.lib.auto.SwerveAutoBuilder;
import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

    /* Driver Buttons */
    // private final JoystickButton resetPositionButton = new JoystickButton(driver, XboxController.Button.kA.value);
    // private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final IntakeSubsystem m_Intake = new IntakeSubsystem();
    private final ArmSubsystem m_Arm = new ArmSubsystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(translationAxis), 
                () -> driver.getRawAxis(strafeAxis), 
                () -> driver.getRawAxis(rotationAxis), 
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
        /* Driver Buttons */
        driver.a().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        driver.y().onTrue(new InstantCommand(s_Swerve::resetModulesToAbsolute));

        // Arm Buttons - Probably need to change these values
        driver.povLeft().onTrue(m_Arm.moveTo(180));
        driver.povUp().onTrue(m_Arm.moveTo(90));
        driver.povRight().onTrue(m_Arm.moveTo(45));
        driver.povDown().onTrue(m_Arm.moveTo(0));

        // Intake
        driver.rightTrigger()
            .onTrue(m_Intake.intake())
            .onFalse(m_Intake.stopCommand());
        driver.leftTrigger()
            .onTrue(m_Intake.reverseIntake())
            .onFalse(m_Intake.stopCommand());
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
