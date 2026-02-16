package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants;

public class IntakeIOSim extends IntakeIOTalonFX {

    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(
        DCMotor.getKrakenX44(1),
        IntakeConstants.GEARING,
        SingleJointedArmSim.estimateMOI(IntakeConstants.LENGTH_METERS, IntakeConstants.MASS_KG),
        IntakeConstants.LENGTH_METERS,
        IntakeConstants.SIM_MIN_ANGLE_RADS,
        IntakeConstants.SIM_MAX_ANGLE_RADS,
        false,
        IntakeConstants.SIM_STARTING_ANGLE_RADS);

    // private PearadoxTalonFX pivotMotor;
    // private TalonFXConfiguration pivotConfigs;
    private TalonFXSimState pivotSimState;

    public IntakeIOSim() {
        pivotSimState = pivotMotor.getSimState();
    }


    // public IntakeIOSim() {
    //     pivotMotor = new PearadoxTalonFX(IntakeConstants.ROLLER_1_LEADER_ID, pivotConfigs);
    //     pivotConfigs = new TalonFXConfiguration();
    //     pivotConfigs.Slot0 = IntakeConstants.ROLLER_SLOT0_CONFIGS;

    //     pivotMotor.getConfigurator().apply(pivotConfigs.Slot0);
    //     pivotSimState = pivotMotor.getSimState();

    //      BaseStatusSignal.setUpdateFrequencyForAll(
    //             Constants.UPDATE_FREQ_SEC,
    //             pivotMotor.getMotorVoltage(),
    //             pivotMotor.getVelocity(),
    //             pivotMotor.getSupplyCurrent(),
    //             pivotMotor.getStatorCurrent());
    // }

        // @Override
        // public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        //     update sim
        //     intake pivot sim still needs to be added
        //     updateSim();
        //     inputs.rollerVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
        //     inputs.rollerVelocity = pivotMotor.getVelocity().getValueAsDouble();
        //     inputs.rollerSupplyCurrent = pivotMotor.getSupplyCurrent().getValueAsDouble();
        //     inputs.rollerStatorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();

        // }

        // public void runRollersVolts(double volts) {
        // pivotMotor.setVoltage(volts);

        public void updateInputs(IntakeIOInputs inputs) {
            super.updateInputs(inputs);

            pivotSimState.setSupplyVoltage(12);

            pivotSim.setInputVoltage(pivotSimState.getMotorVoltage());
            pivotSim.update(Constants.UPDATE_FREQ_SEC);

             pivotSimState.setRawRotorPosition(
                Units.radiansToRotations(pivotSim.getAngleRads() * IntakeConstants.GEARING));
            pivotSimState.setRotorVelocity(
                Units.radiansToRotations(pivotSim.getVelocityRadPerSec()) * IntakeConstants.GEARING);
        }


}
