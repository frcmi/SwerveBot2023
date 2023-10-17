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
    private final Encoder encoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X);

    private final ProfiledPIDController pidController 
        = new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, 
            new TrapezoidProfile.Constraints(ArmConstants.kMaxVel, ArmConstants.kMaxAccel));
    private final ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

    double lastSpeed = 0;
    double lastTime = Timer.getFPGATimestamp();

    public ArmSubsystem() {
        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setInverted(InvertType.FollowMaster);
        rightMotor.follow(leftMotor);
        rightMotor.setNeutralMode(NeutralMode.Brake);

        encoder.setDistancePerPulse(1);
        //pidController.setGoal(getAngle());
        //pidController.setTolerance(Math.toRadians(1.5));
        // would be optimal to use PID as default to hold position
        // but this lets us sway our arm for intaking cones
        setDefaultCommand(stop());
    }

    public CommandBase wristStow(){
        return run(() -> leftMotor.set(TalonFXControlMode.PercentOutput, -0.3));
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

    /* 
    
    private void setVolts(double percent) {
        double angle = getAngle();
        double kg = feedforward.calculate(angle, 0);
        //SmartDashboard.putNumber("Arm Percentage Input", percent);
        //SmartDashboard.putBoolean("Arm Bounds", !(angle > ArmConstants.maxAngle || angle < ArmConstants.minAngle));
        // Stop movement if outside bounds
        if (angle < ArmConstants.minAngle) 
            percent = Math.max(kg, Math.min(2, percent));
        if (angle > ArmConstants.maxAngle)
            percent = Math.max(-2, Math.min(kg, percent));

        // SmartDashboard.putNumber("Arm Voltage Set", volts);
        // SmartDashboard.putNumber("Arm Voltage Left", leftMotor.getAppliedOutput());
        leftMotor.set(TalonFXControlMode.PercentOutput, percent);
        rightMotor.set(TalonFXControlMode.PercentOutput, percent);
    }

    public double getAngle() {
        return -(encoder.getDistance() * 2 * Math.PI) + ArmConstants.encoderOffset;
    }

    public void setAngle(double goalAngle) {
        double pidOutput = pidController.calculate(getAngle(), goalAngle);
        State setpoint = pidController.getSetpoint();
        double ffOutpout = feedforward.calculate(setpoint.position, setpoint.velocity);
        //SmartDashboard.putNumber("Arm FF Out", ffOutpout);
        //SmartDashboard.putNumber("Arm PID Out", pidOutput);
        //SmartDashboard.putNumber("Arm Goal Volts", pidOutput + ffOutpout);
        setVolts(pidOutput + ffOutpout);  
    }

    public CommandBase moveTo(double angle) {
        return run(() -> setAngle(angle)).until(pidController::atGoal);
    }
    */
}
