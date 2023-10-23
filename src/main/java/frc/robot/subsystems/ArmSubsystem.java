package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;
public class ArmSubsystem extends SubsystemBase {
    double StowVolts = 0;

    private final WPI_TalonFX leftMotor = new WPI_TalonFX(ArmConstants.kLeftMotorId);
    private final WPI_TalonFX rightMotor = new WPI_TalonFX(ArmConstants.kRightMotorId);

    public ArmSubsystem() {
        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setInverted(InvertType.FollowMaster);
        rightMotor.follow(leftMotor);
        rightMotor.setNeutralMode(NeutralMode.Coast);
        setDefaultCommand(stow());
    }

    public CommandBase wristDown(){
        rightMotor.setNeutralMode(NeutralMode.Coast);
        leftMotor.setNeutralMode(NeutralMode.Coast);
        return run(() -> leftMotor.set(TalonFXControlMode.PercentOutput, ArmConstants.kArmSpeed));
    }

    public CommandBase wristUp(){
        rightMotor.setNeutralMode(NeutralMode.Brake);
        leftMotor.setNeutralMode(NeutralMode.Brake);
        return run(() -> leftMotor.set(TalonFXControlMode.PercentOutput, -ArmConstants.kArmSpeed));
    }

    public CommandBase stop() {
        return run(() -> leftMotor.set(TalonFXControlMode.PercentOutput, 0));
    }

    public CommandBase stow(){
        rightMotor.setNeutralMode(NeutralMode.Brake);
        leftMotor.setNeutralMode(NeutralMode.Brake);
        return run(() -> leftMotor.setVoltage(StowVolts)); 
    }

    @Override
    public void periodic() {
        var currentCommand = this.getCurrentCommand();
        if (currentCommand != null){
            Shuffleboard.getTab("Arm")
            .add("Wrist Command", currentCommand.getName());
        } else {
            Shuffleboard.getTab("Arm")
            .add("Wrist Command", "");
        }

        Shuffleboard.getTab("Arm")
            .add("Stow Volts", StowVolts)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -12, "max", 12)) // specify the widget here
            .getEntry();

        Shuffleboard.getTab("Arm")
            .add("Wrist Current", leftMotor.getStatorCurrent());

    }
}
