package frc.robot.subsystems;

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
        leftMotor.follow(rightMotor);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Voltage", rightMotor.getMotorOutputVoltage());
        var currentCommand = this.getCurrentCommand();
        if (currentCommand != null)
        SmartDashboard.putString("Intake Command", currentCommand.getName());
    }

    // Intake cone, release cube
    public CommandBase intake() {
        return setMotor(IntakeConstants.kIntakeSpeed)
                // .until(this::motorOverCurrent)
                // .andThen(Commands.waitSeconds(IntakeConstants.kIntakeTime))
                // .andThen(stopCommand())
                ;
    }

    private CommandBase setMotor(double speed) {
        return Commands.run(() -> rightMotor.set(TalonFXControlMode.PercentOutput, speed), this);
    }

    // Release cone, intake cube
    public CommandBase reverseIntake() {
        return setMotor(IntakeConstants.kIntakeSpeed * -1)
                // .until(this::motorOverCurrent)
                // .andThen(Commands.waitSeconds(IntakeConstants.kIntakeTime))
                // .andThen(stopCommand())
                ;
    }

    public void stop() {
        rightMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public CommandBase stopCommand() {
        return runOnce(this::stop);
    }
}
