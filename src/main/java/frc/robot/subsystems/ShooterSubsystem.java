package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    // Hardware
    private final TalonFX m_topMotor;
    private final TalonFX m_bottomMotor;

    // Control Request (Velocity Closed Loop)
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public ShooterSubsystem() {
        // Initialize motors
        m_topMotor = new TalonFX(Constants.Shooter.kTopMotorId, Constants.Swerve.kCANBus);
        m_bottomMotor = new TalonFX(Constants.Shooter.kBottomMotorId, Constants.Swerve.kCANBus);

        // Create Configuration object
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // 1. Current Limits
        configs.CurrentLimits.StatorCurrentLimit = Constants.Shooter.kCurrentLimit;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;

        // 2. PID Gains (Slot 0)
        configs.Slot0.kP = Constants.Shooter.kP;
        configs.Slot0.kI = Constants.Shooter.kI;
        configs.Slot0.kD = Constants.Shooter.kD;
        configs.Slot0.kS = Constants.Shooter.kS;
        configs.Slot0.kV = Constants.Shooter.kV;

        // 3. Neutral Mode (Coast is usually better for shooters to avoid sudden stops)
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // 4. Apply to Top Motor (Not Inverted)
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_topMotor.getConfigurator().apply(configs);

        // 5. Apply to Bottom Motor (Inverted - Spins Opposite)
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_bottomMotor.getConfigurator().apply(configs);
    }

    /**
     * Run motors at target velocity
     */
    public void runShooter() {
        // Set both motors to 5000 RPM (converted to RPS)
        // Since we configured Inverted states above, Positive velocity makes them spin "forward" relative to the mechanism
        m_topMotor.setControl(m_request.withVelocity(Constants.Shooter.kTargetRPS));
        m_bottomMotor.setControl(m_request.withVelocity(Constants.Shooter.kTargetRPS));
    }

    /**
     * Stop motors
     */
    public void stopShooter() {
        m_topMotor.stopMotor();
        m_bottomMotor.stopMotor();
    }

    /**
     * Command to run shooter while button is held
     */
    public Command runShooterCommand() {
        return this.runEnd(
            this::runShooter,
            this::stopShooter
        );
    }
}