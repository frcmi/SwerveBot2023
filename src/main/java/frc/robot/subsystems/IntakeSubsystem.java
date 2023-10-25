package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
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
    private final WPI_TalonFX topMotor = new WPI_TalonFX(IntakeConstants.kTopMotorId);
    private final WPI_TalonFX bottomMotor = new WPI_TalonFX(IntakeConstants.kBottomMotorId);
    
    public IntakeSubsystem() {
        topMotor.setNeutralMode(NeutralMode.Coast);
        bottomMotor.setNeutralMode(NeutralMode.Coast);
        // bottomMotor.setInverted(true); not needed since inverted values is good and inversion by flip

        topMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40.0, 60.0, 0.1));
        bottomMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40.0, 60.0, 0.1));

        setDefaultCommand(hold());
    }

    @Override
    public void periodic() {
        var currentCommand = this.getCurrentCommand();
        if (currentCommand != null){
            SmartDashboard.putString("Arm Command", currentCommand.getName());
        } else {
            SmartDashboard.putString("Arm Command", "");
        }
    }

    public CommandBase intake(){
        return run(
                () -> {bottomMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kIntakePercent);
                       topMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kIntakePercent);}
        ).withName("Intake");
    }

    public CommandBase l1Shoot(){
        return run(
                () -> {bottomMotor.set(TalonFXControlMode.PercentOutput, -IntakeConstants.kLowBottomPercent);
                       topMotor.set(TalonFXControlMode.PercentOutput, -IntakeConstants.kLowTopPercent);}
        ).withName("L1 Shoot");
    }

    public CommandBase l2Shoot(){
        return run(
                () -> {bottomMotor.set(TalonFXControlMode.PercentOutput, -IntakeConstants.kMidBottomPercent);
                       topMotor.set(TalonFXControlMode.PercentOutput, -IntakeConstants.kMidTopPercent);}
        ).withName("L2 Shoot");
    }

    public CommandBase l3Shoot(){
       return run(
                () -> {bottomMotor.set(TalonFXControlMode.PercentOutput, -IntakeConstants.kMidBottomPercent);
                       topMotor.set(TalonFXControlMode.PercentOutput, -IntakeConstants.kMidTopPercent);}
        ).withName("L3 Shoot");
    }

    public CommandBase hold() {
        return run(
            () -> {bottomMotor.set(TalonFXControlMode.Current, IntakeConstants.kHoldAmps);
                   topMotor.set(TalonFXControlMode.Current, IntakeConstants.kHoldAmps);}
        ).withName("Hold");
    }

    /*public CommandBase stop() {
        return this.runOnce(
            () -> {bottomMotor.set(TalonFXControlMode.PercentOutput, 0);
                   topMotor.set(TalonFXControlMode.PercentOutput, 0);}
        );
    }
    
    public CommandBase stopCommand() {
        return runOnce(this::stop);
    }*/
}
