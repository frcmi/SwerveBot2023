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
    private final WPI_TalonFX topMotor = new WPI_TalonFX(IntakeConstants.kTopMotorId);
    private final WPI_TalonFX bottomMotor = new WPI_TalonFX(IntakeConstants.kBottomMotorId);
    
    public IntakeSubsystem() {
        topMotor.setNeutralMode(NeutralMode.Coast);
        
        bottomMotor.setNeutralMode(NeutralMode.Coast);
        // bottomMotor.setInverted(true);
        
        setDefaultCommand(hold());
    }

    @Override
    public void periodic() {

    }

    public CommandBase intake(){
        return this.runOnce(
                () -> {bottomMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kIntakeSpeed);
                       topMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kIntakeSpeed);}
        );
    }

    public CommandBase l1Shoot(){
        return this.runOnce(
                () -> {bottomMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kLowBottomPercent);
                       topMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kLowTopPercent);}
        );
    }

    public CommandBase l2Shoot(){
        return this.runOnce(
                () -> {bottomMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kMidBottomPercent);
                       topMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kMidTopPercent);}
        );
    }

    public CommandBase l3Shoot(){
       return this.runOnce(
                () -> {bottomMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kMidBottomPercent);
                       topMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kMidTopPercent);}
        );
    }

    public CommandBase stop() {
        return this.runOnce(
            () -> {bottomMotor.set(TalonFXControlMode.PercentOutput, 0);
                   topMotor.set(TalonFXControlMode.PercentOutput, 0);}
        );
    }

    public CommandBase hold() {
        return this.runOnce(
            () -> {bottomMotor.setVoltage(IntakeConstants.kHoldVolts);
                   topMotor.setVoltage(IntakeConstants.kHoldVolts);}
        );
    }

    public CommandBase stopCommand() {
        return runOnce(this::stop);
    }
}
