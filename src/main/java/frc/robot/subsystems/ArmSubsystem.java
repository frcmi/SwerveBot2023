package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.ArmConstants;
public class ArmSubsystem extends SubsystemBase {
    private final TalonFX leftMotor = new TalonFX(ArmConstants.kLeftMotorId);
    private final TalonFX rightMotor = new TalonFX(ArmConstants.kRightMotorId);

    public ArmSubsystem() {
        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setInverted(InvertType.FollowMaster);
        rightMotor.follow(leftMotor);
        rightMotor.setNeutralMode(NeutralMode.Coast);
        setDefaultCommand(stop());
    }

    public CommandBase wristDown(){
        return run(() -> leftMotor.set(TalonFXControlMode.PercentOutput, ArmConstants.kArmSpeed));
    }

    public CommandBase wristUp(){
        return run(() -> leftMotor.set(TalonFXControlMode.PercentOutput, -ArmConstants.kArmSpeed));
    }

    public CommandBase stop() {
        return run(() -> leftMotor.set(TalonFXControlMode.PercentOutput, 0));
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Arm Radians", getAngle());
        //SmartDashboard.putNumber("Wrist Degrees", Math.toDegrees(getAngle()));
        SmartDashboard.putNumber("Wrist Current", leftMotor.getStatorCurrent());
        var currentCommand = this.getCurrentCommand();
        if (currentCommand != null){
            SmartDashboard.putString("Wrist Command", currentCommand.getName());
        } else {
            SmartDashboard.putString("Wrist Command", "");
        }
        // SmartDashboard.putNumber("Arm Encoder", absoluteEncoder.getAbsolutePosition());
        // SmartDashboard.putData("Arm PID", pidController);
        // SmartDashboard.putNumber("Arm PID Error Deg", Math.toDegrees(pidController.getPositionError()));
    }
}
