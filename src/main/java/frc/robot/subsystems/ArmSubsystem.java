package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;
public class ArmSubsystem extends SubsystemBase {
    private final WPI_TalonFX leftMotor = new WPI_TalonFX(ArmConstants.kLeftMotorId);
    private final WPI_TalonFX rightMotor = new WPI_TalonFX(ArmConstants.kRightMotorId);

    public ArmSubsystem() {
        leftMotor.setNeutralMode(NeutralMode.Brake);
        leftMotor.setInverted(TalonFXInvertType.Clockwise);
        leftMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40.0, 60.0, 0.1));
        
        rightMotor.follow(leftMotor);
        rightMotor.setInverted(TalonFXInvertType.OpposeMaster);
        rightMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40.0, 60.0, 0.1));
        setDefaultCommand(stow());
    }

    public CommandBase wristDown(){
        return run(() -> leftMotor.set(TalonFXControlMode.PercentOutput, ArmConstants.kArmDownPercent)).withName("Wrist Down");
    }

    public CommandBase wristDownVolts(){
        return run(() -> leftMotor.setVoltage(ArmConstants.kArmDownVolts)).withName("Wrist Down");
    }
    
    public CommandBase wristDownEndUp(){
        return runEnd(() -> leftMotor.set(TalonFXControlMode.PercentOutput, ArmConstants.kArmDownPercent), 
                     () -> wristUp()).withName("Wrist Down");
    }

    public CommandBase wristDownEndUpVolts(){
        return runEnd(() -> leftMotor.setVoltage(ArmConstants.kArmDownPercent), 
                     () -> wristUp()).withName("Wrist Down");
    }

    public CommandBase wristUpVolts(){
        return run(() -> leftMotor.setVoltage(-ArmConstants.kArmUpVolts)).withName("Wrist Up").withTimeout(.5);
    }

    public CommandBase wristUp(){
        return run(() -> leftMotor.set(TalonFXControlMode.PercentOutput, -ArmConstants.kArmUpPercent)).withName("Wrist Up").withTimeout(.5);
    }


    public CommandBase stop() {
        return run(() -> leftMotor.set(TalonFXControlMode.PercentOutput, 0)).withName("Stop");
    }

    public CommandBase stow(){
        return run(() -> leftMotor.setVoltage(ArmConstants.kStowVolts)).withName("Stow"); 
    }

    @Override
    public void periodic() {
        var currentCommand = this.getCurrentCommand();
        if (currentCommand != null){
            SmartDashboard.putString("Wrist Command", currentCommand.getName());
        } else {
            SmartDashboard.putString("Wrist Command", "");
        }
    }
}
