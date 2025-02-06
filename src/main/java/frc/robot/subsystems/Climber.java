package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(Constants.kClimberMotorId, Constants.kSwerveCanBus);
    private final TorqueCurrentFOC m_request = new TorqueCurrentFOC(0);
    public Climber() {
        m_motor.getConfigurator().apply(new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive) // TODO TBD
                .withNeutralMode(NeutralModeValue.Brake)
            ).withSlot0(new Slot0Configs()
                .withKS(0)
                .withKP(0)
            )
        );
    }

    public Command climb() {
        return startEnd(() -> m_motor.setControl(m_request.withOutput(1)), () -> m_motor.setControl(m_request.withOutput(0)));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}