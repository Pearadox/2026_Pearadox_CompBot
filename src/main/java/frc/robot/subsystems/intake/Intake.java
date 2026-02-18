package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.intake.IntakeConstants.StateConfig;
import frc.robot.util.LoggedTunableNumber;

public class Intake extends SubsystemBase{

    private IntakeIO io;

    public IntakeState intakeState = IntakeState.STOWED;
    public static double adjust = 0;


    public Intake(IntakeIO io) {
        this.io = io;
    }

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private static LoggedTunableNumber loggedIntakeRollerVoltage = new LoggedTunableNumber("Intake/Voltage", 4.0);

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("Intake", inputs);
        
        Logger.recordOutput("Intake/State", intakeState.toString());
        io.runRollersVolts(StateConfig.INTAKE_STATE_MAP.get(intakeState).voltage() + adjust);
        io.runPositionDegrees(StateConfig.INTAKE_STATE_MAP.get(intakeState).angleDeg());
        MechVisualizer.getInstance().updatePositionDegrees(Units.rotationsToDegrees(inputs.pivotMotorData.position() / IntakeConstants.GEARING));
        
        Logger.recordOutput("Intake/Target Position Degrees", StateConfig.INTAKE_STATE_MAP.get(intakeState).angleDeg());
        Logger.recordOutput("Intake/VoltageOut", inputs.rollerMotorData.appliedVolts());
        Logger.recordOutput("Intake/Current Position Degrees", Units.rotationsToDegrees(inputs.pivotMotorData.position() / IntakeConstants.GEARING));

        // UNCOMMENT WHEN TESTING INTAKE TO TUNE VOLTAGE!
        // if(loggedIntakeRollerVoltage.hasChanged(hashCode())) { inputs.rollerVoltage = loggedIntakeRollerVoltage.get(); }

        // if (intakeState == IntakeState.INTAKING) {
        //     io.runRollersVolts(loggedIntakeRollerVoltage.getAsDouble());
        // } else {
        //     io.runRollersVolts(0);
        // }
    }

     public void setStowed() {
        intakeState = IntakeState.STOWED;
     }

     public void setIntaking() {
        intakeState = IntakeState.INTAKING;
     }

     public void setOuttaking() {
        intakeState = IntakeState.OUTTAKING;
     }

     public IntakeState getIntakeState() {
         return intakeState;
     }


     public void adjustSpeed(double adjustBy) {
        adjust += adjustBy;
     }

    public void resetAdjust() {
            adjust = 0;
     }


}
