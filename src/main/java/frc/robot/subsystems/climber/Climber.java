package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.derive;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants.ClimberState;
import frc.robot.util.SmarterDashboard;

public class Climber extends SubsystemBase {
    ClimberState climberState = ClimberState.RAISED;

    private ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("Climber", inputs);
        SmarterDashboard.putNumber("Climber/VoltageOut", inputs.appliedVolts);
        SmarterDashboard.putNumber("Climber/PositionRots", inputs.positionRots);

        io.runPosition(climberState.getClimberPositionRotations());
    }

    public void climb() {
        climberState = ClimberState.LOWERED;
    }

    public void descend() {
        climberState = ClimberState.RAISED;
    }
}