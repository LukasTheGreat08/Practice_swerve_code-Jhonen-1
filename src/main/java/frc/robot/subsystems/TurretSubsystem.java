package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; // Pointing to the new file

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX m_turretMotor;
    private final DutyCycleOut m_request = new DutyCycleOut(0);

    public TurretSubsystem() {
        // Use Constants.Turret for ID and Constants.Swerve for CANBus
        m_turretMotor = new TalonFX(Constants.Turret.kMotorId, Constants.Swerve.kCANBus);

        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.CurrentLimits.StatorCurrentLimit = Constants.Turret.kCurrentLimit;
        configs.CurrentLimits.StatorCurrentLimitEnable = Constants.Turret.kEnableCurrentLimit;

        if (Constants.Turret.kInverted) {
            configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        } else {
            configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_turretMotor.getConfigurator().apply(configs);
    }

    public void setSpeed(double speed) {
        m_turretMotor.setControl(m_request.withOutput(speed));
    }

    public void stop() {
        m_turretMotor.setControl(m_request.withOutput(0));
    }

    public Command turnLeftCommand() {
        return this.runEnd(
            () -> this.setSpeed(Constants.Turret.kManualSpeed), 
            this::stop
        );
    }

    public Command turnRightCommand() {
        return this.runEnd(
            () -> this.setSpeed(-Constants.Turret.kManualSpeed), 
            this::stop
        );
    }
}