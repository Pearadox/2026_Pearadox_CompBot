package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.intake.IntakeConstants.StateConfig;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SmarterDashboard;

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
        
        SmarterDashboard.putString("Intake/State", intakeState.toString());
        io.runRollersVolts(StateConfig.INTAKE_STATE_MAP.get(intakeState).voltage() + adjust);
        SmarterDashboard.putNumber("Intake/VoltageOut", inputs.rollerVoltage);

        // UNCOMMENT WHEN TESTING INTAKE TO TUNE VOLTAGE!s
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


     public void adjustSpeed(double adjustBy) {
        adjust += adjustBy;
     }

    public void resetAdjust() {
            adjust = 0;
     }


}
