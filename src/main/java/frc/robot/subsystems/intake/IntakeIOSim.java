package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO{
    private PearadoxTalonFX rollerMotor;
    private TalonFXConfiguration rollerConfigs;
    private TalonFXSimState rollerSimState;

    private SingleJointedArmSim rollerSim = new SingleJointedArmSim(
        DCMotor.getKrakenX44(1),
        IntakeConstants.GEARING,
        SingleJointedArmSim.estimateMOI(IntakeConstants.LENGTH_METERS, IntakeConstants.MASS_KG),
        IntakeConstants.LENGTH_METERS,
        IntakeConstants.SIM_MIN_ANGLE_RADS,
        IntakeConstants.SIM_MAX_ANGLE_RADS,
        true,
        IntakeConstants.SIM_STARTING_ANGLE_RADS,
         0);


    public IntakeIOSim() {
        rollerMotor = new PearadoxTalonFX(IntakeConstants.ROLLER_ID, rollerConfigs);
        rollerConfigs = new TalonFXConfiguration();
        rollerConfigs.Slot0 = IntakeConstants.ROLLER_SLOT0_CONFIGS;

        rollerMotor.getConfigurator().apply(rollerConfigs.Slot0);
        rollerSimState = rollerMotor.getSimState();

         BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.UPDATE_FREQ_SEC,
                rollerMotor.getMotorVoltage(),
                rollerMotor.getVelocity(),
                rollerMotor.getSupplyCurrent(),
                rollerMotor.getStatorCurrent());
    }

        public void updateInputs(IntakeIOInputsAutoLogged inputs) {
            // update sim
            // intake pivot sim still needs to be added
            updateSim();
            inputs.rollerVoltage = rollerMotor.getMotorVoltage().getValueAsDouble();
            inputs.rollerVelocity = rollerMotor.getVelocity().getValueAsDouble();
            inputs.rollerSupplyCurrent = rollerMotor.getSupplyCurrent().getValueAsDouble();
            inputs.rollerStatorCurrent = rollerMotor.getStatorCurrent().getValueAsDouble();

        }

        public void runRollersVolts(double volts) {
        rollerMotor.setVoltage(volts);
    }

        public void updateSim() {
            rollerSimState.setSupplyVoltage(12);
            rollerSim.setInputVoltage(rollerSimState.getMotorVoltage());

             rollerSimState.setRawRotorPosition(
                Units.radiansToRotations(rollerSim.getAngleRads() * IntakeConstants.GEARING));
            rollerSimState.setRotorVelocity(
                Units.radiansToRotations(rollerSim.getVelocityRadPerSec()) * IntakeConstants.GEARING);
            rollerSim.update(Constants.UPDATE_FREQ_SEC);
        }


}
