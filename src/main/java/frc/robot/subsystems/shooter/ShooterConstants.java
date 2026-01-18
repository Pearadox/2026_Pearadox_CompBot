// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class ShooterConstants {

    public static enum ShooterState {
        OFF(0.0),
        MANUAL(5.0),
        AUTO_SCORE(0.0),
        AUTO_PASS(0.0);

        private double defaultVoltage;
        private double adjust;

        private ShooterState(double voltage) {
            defaultVoltage = voltage;
            adjust = 0;
        }

        public double getVoltage() {
            if (this == OFF) return 0.0;
            else if (this == MANUAL) return defaultVoltage + adjust;
            else {
                return -1; // TODO: get aiming to work?
            }
        }

        public double getAdjust() {
            return adjust;
        }

        public void adjustVoltage(double adjustBy) {
            adjust += adjustBy;
        }

        public void resetAdjust() {
            adjust = 0;
        }

        public String toString() {
            switch (this) {
                case OFF : return "OFF";
                case MANUAL : return "MANUAL";
                case AUTO_SCORE : return "AUTO_SCORE";
                case AUTO_PASS : return "AUTO_PASSING";
                default : return null;
            }
        }
    }

    public static final int ROLLER_1_CAN_ID = 21;

    public static final int ROLLER_2_CAN_ID = 22;

    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

    public static final int CURRENT_LIMIT = 20;

    public static final boolean INVERTED = true;

    public static final int GEARING = 0;

    public static final TalonFXConfiguration CONFIG = new TalonFXConfiguration();
    public static final Slot0Configs SLOT0_CONFIGS = CONFIG.Slot0;

    public static final TalonFXConfiguration shooterConfig() {
        CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        CONFIG.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;

        CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        CONFIG.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;

        CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        return CONFIG;
    } // TODO: maybe update PearadoxTalonFX with a constructor that takes in ID and a TalonFXConfiguration

    public static final Slot0Configs get_shooter_config() {
        SLOT0_CONFIGS.kP = 0.1;
        SLOT0_CONFIGS.kI = 0.0;
        SLOT0_CONFIGS.kD = 0.0;

        return SLOT0_CONFIGS;
    }

}