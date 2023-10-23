package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    double bottomPercent = 0;
    double topPercent = 0;
    double holdVolts = 0;

    private final WPI_TalonFX topMotor = new WPI_TalonFX(IntakeConstants.kTopMotorId);
    private final WPI_TalonFX bottomMotor = new WPI_TalonFX(IntakeConstants.kBottomMotorId);
    
    public IntakeSubsystem() {
        topMotor.setNeutralMode(NeutralMode.Coast);
        topMotor.configVoltageCompSaturation(10);
        topMotor.enableVoltageCompensation(true);
        
        bottomMotor.setNeutralMode(NeutralMode.Coast);
        bottomMotor.setInverted(true);
        bottomMotor.configVoltageCompSaturation(10);
        bottomMotor.enableVoltageCompensation(true);
        
        setDefaultCommand(hold());
    }

    @Override
    public void periodic() {
        var currentCommand = this.getCurrentCommand();
        if (currentCommand != null){
            Shuffleboard.getTab("Intake")
            .add("Intake Command", currentCommand.getName());
        } else {
            Shuffleboard.getTab("Intake")
            .add("Intake Command", "");
        }

        Shuffleboard.getTab("Intake")
            .add("Intake Voltage", bottomMotor.getMotorOutputVoltage());

        Shuffleboard.getTab("Arm")
            .add("Bottom Roller Percent", bottomPercent)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1)) // specify the widget here
            .getEntry();

        Shuffleboard.getTab("Arm")
            .add("Top Roller Percent", topPercent)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1)) // specify the widget here
            .getEntry();    

        Shuffleboard.getTab("Intake")
            .add("Hold Volts", holdVolts)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -12, "max", 12)) // specify the widget here
            .getEntry();
    }

    public CommandBase intake(){
        return this.runOnce(
                () -> {bottomMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kIntakeSpeed);
                       topMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kIntakeSpeed);}
        ).andThen(new WaitCommand(0.25));
    }

    public CommandBase l1Shoot(){
        return this.runOnce(
                () -> {bottomMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kLowBottomPercent);
                       topMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kLowTopPercent);}
        ).andThen(new WaitCommand(0.25));
    }

    public CommandBase l2Shoot(){
        return this.runOnce(
                () -> {bottomMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kMidBottomPercent);
                       topMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kMidTopPercent);}
        ).andThen(new WaitCommand(0.25));
    }

    public CommandBase l3Shoot(){
       return this.runOnce(
                () -> {bottomMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kMidBottomPercent);
                       topMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kMidTopPercent);}
        ).andThen(new WaitCommand(0.25));
    }

    public CommandBase stop() {
        return this.runOnce(
            () -> {bottomMotor.set(TalonFXControlMode.PercentOutput, 0);
                   topMotor.set(TalonFXControlMode.PercentOutput, 0);}
        );
    }

    public CommandBase hold() {
        return this.runOnce(
            () -> {bottomMotor.setVoltage(holdVolts);
                   topMotor.setVoltage(holdVolts);}
        );
    }

    public CommandBase stopCommand() {
        return runOnce(this::stop);
    }
}
