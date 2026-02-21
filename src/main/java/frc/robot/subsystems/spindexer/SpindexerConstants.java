package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.util.Map;

public class SpindexerConstants {
  public static enum SpindexerState {
    STOPPED,
    RUNNING
  }

  public static record StateConfig(double voltage) {
    public static final Map<SpindexerState, StateConfig> SPINDEXER_STATE_MAP =
        Map.of(
            SpindexerState.STOPPED, new StateConfig(0),
            SpindexerState.RUNNING, new StateConfig(1.5));
  }

  public static final TalonFXConfiguration SPINDEXER_CONFIG = new TalonFXConfiguration();
  public static final Slot0Configs SPINDEXER_SLOT0_CONFIGS = SPINDEXER_CONFIG.Slot0;

  public static final int SPINDEXER_MOTOR_ID = 30;
  public static final int LAUNCHER_CURRENT_LIMIT = 20;

  public static final TalonFXConfiguration spindexerConfig() {
    SPINDEXER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    SPINDEXER_CONFIG.CurrentLimits.StatorCurrentLimit = LAUNCHER_CURRENT_LIMIT;

    SPINDEXER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    SPINDEXER_CONFIG.CurrentLimits.SupplyCurrentLimit = LAUNCHER_CURRENT_LIMIT;

    SPINDEXER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    SPINDEXER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    SPINDEXER_SLOT0_CONFIGS.kP = 0.1;
    SPINDEXER_SLOT0_CONFIGS.kI = 0.0;
    SPINDEXER_SLOT0_CONFIGS.kD = 0.0;
    SPINDEXER_SLOT0_CONFIGS.kS = 0.0;
    SPINDEXER_SLOT0_CONFIGS.kV = 0.0;

    return SPINDEXER_CONFIG;
  }
}
