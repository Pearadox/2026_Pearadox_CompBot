package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class TurretIOReal extends TurretIOTalonFX {
    private final CANcoder absoluteEncoder;
    private final CANcoderConfiguration cancoderConfigs;
    private final BaseStatusSignal positionSignal;

    public TurretIOReal() {
        absoluteEncoder = new CANcoder(TurretConstants.TURRET_CANCODER_ID);

        cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = TurretConstants.TURRET_CANCODER_OFFSET_ROTS;
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        PhoenixUtil.tryUntilOk(5, () -> absoluteEncoder.getConfigurator().apply(cancoderConfigs, 0.25));

        positionSignal = absoluteEncoder.getAbsolutePosition(false);
        BaseStatusSignal.setUpdateFrequencyForAll(Constants.LOOP_FREQUENCY, positionSignal);

        absoluteEncoder.optimizeBusUtilization();

        PhoenixUtil.registerSignals(false, positionSignal);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        super.updateInputs(inputs);

        inputs.cancoderPosition = positionSignal.getValueAsDouble();
        inputs.cancoderConnected = BaseStatusSignal.isAllGood(positionSignal);
    }
}
