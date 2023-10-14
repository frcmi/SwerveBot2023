package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX leftMotor = new TalonFX(IntakeConstants.kLeftMotorId);
    private final TalonFX rightMotor = new TalonFX(IntakeConstants.kRightMotorId);
    
    public IntakeSubsystem() {
        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.follow(leftMotor);
        rightMotor.setNeutralMode(NeutralMode.Coast);

        //pidController.setGoal(getAngle());
        //pidController.setTolerance(Math.toRadians(1.5));
        // would be optimal to use PID as default to hold position
        // but this lets us sway our arm for intaking cones
        setDefaultCommand(stop());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Voltage", rightMotor.getMotorOutputVoltage());
        var currentCommand = this.getCurrentCommand();
        if (currentCommand != null)
        SmartDashboard.putString("Intake Command", currentCommand.getName());
    }

    public CommandBase intake(){
        return run(() -> leftMotor.set(TalonFXControlMode.PercentOutput, IntakeConstants.kIntakeSpeed));
    }

    public CommandBase outtake(){
        return run(() -> leftMotor.set(TalonFXControlMode.PercentOutput, -IntakeConstants.kIntakeSpeed));
    }

    public CommandBase stop() {
        return run(() -> leftMotor.set(TalonFXControlMode.PercentOutput, 0));
    }

    public CommandBase stopCommand() {
        return runOnce(this::stop);
    }
}
