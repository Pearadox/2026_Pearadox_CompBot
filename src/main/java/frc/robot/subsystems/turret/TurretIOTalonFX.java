package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.robot.util.PearadoxTalonFX;

public abstract class TurretIOTalonFX implements TurretIO {
    protected final PearadoxTalonFX turretMotor;
    protected final MotionMagicVoltage turretMMRequest = new MotionMagicVoltage(0);

    protected TurretIOTalonFX() {
        turretMotor = new PearadoxTalonFX(TurretConstants.TURRET_ID, TurretConstants.getTurretConfig());
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretData = turretMotor.getData();
    }

    @Override
    public void runPosition(double setpointRots, double ffVolts) {
        turretMotor.setControl(turretMMRequest.withPosition(setpointRots).withFeedForward(ffVolts));
    }
}
