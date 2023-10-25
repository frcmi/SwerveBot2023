package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;
public class ArmSubsystem extends SubsystemBase {
    private final WPI_TalonFX leftMotor = new WPI_TalonFX(ArmConstants.kLeftMotorId);
    private final WPI_TalonFX rightMotor = new WPI_TalonFX(ArmConstants.kRightMotorId);

    public ArmSubsystem() {
        leftMotor.setNeutralMode(NeutralMode.Brake);
        leftMotor.setInverted(TalonFXInvertType.CounterClockwise);
        
        rightMotor.follow(leftMotor);
        rightMotor.setInverted(TalonFXInvertType.OpposeMaster);
        rightMotor.setNeutralMode(NeutralMode.Brake);
        setDefaultCommand(stow());
    }

    public CommandBase wristDown(){
        return run(() -> leftMotor.set(TalonFXControlMode.PercentOutput, ArmConstants.kArmDownSpeed));
    }

    public CommandBase wristUp(){
        return run(() -> leftMotor.set(TalonFXControlMode.PercentOutput, -ArmConstants.kArmUpSpeed));
    }

    public CommandBase stop() {
        return run(() -> leftMotor.set(TalonFXControlMode.PercentOutput, 0));
    }

    public CommandBase stow(){
        return run(() -> leftMotor.setVoltage(ArmConstants.kStowVolts)); 
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
