package net.tecdroid.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import net.tecdroid.util.*;

public class ClimberController {
    private final TalonFX leftClimberMotor;
    private final TalonFX rightClimberMotor;
    private final Config climberConfig;


    public ClimberController(Config climberConfig) {
        this.climberConfig = climberConfig;


        leftClimberMotor = new TalonFX(climberConfig.Identifiers.leftClimberMotorID.getId());
        rightClimberMotor = new TalonFX(climberConfig.Identifiers.rightClimberMotorID.getId());


        configureMotor(leftClimberMotor, climberConfig.PhysicalDescription.leftClimberMotorGearRatio,
                climberConfig.Conventions.leftClimberMotorRotationalPositiveDirection,
                climberConfig.Limits.leftClimberMotorCurrentLimit);

        configureMotor(rightClimberMotor, climberConfig.PhysicalDescription.rightClimberMotorGearRatio,
                climberConfig.Conventions.rightClimberMotorRotationalPositiveDirection,
                climberConfig.Limits.rightClimberMotorCurrentLimit);
    }

    public void enableClimber(AngularVelocity motorAngularVelocity) {
        leftClimberMotor.setControl(new VelocityVoltage(motorAngularVelocity));
        rightClimberMotor.setControl(new VelocityVoltage(motorAngularVelocity));
    }

    public void stopClimber() {
        leftClimberMotor.setVoltage(0.0);
        rightClimberMotor.setVoltage(0.0);
    }

    private void configureMotor(TalonFX motor, GearRatio gearRatio, RotationalDirection direction, Current currentLimit) {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0.withKP(4.8).withKI(0).withKD(0.1).withKS(0.25).withKV(0.12).withKA(0.01);
        TalonFXConfiguration talonFxConfigs = new TalonFXConfiguration();
        MotionMagicConfigs motionMagicConfigs = talonFxConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80;
        motionMagicConfigs.MotionMagicAcceleration = 160;
        motionMagicConfigs.MotionMagicJerk = 1600;
        Object m_talonFx = new Object();
        
        final  MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        m_talonFx.setControl(m_request.withPosition(100));
        

                                                                                


        motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake).withInverted(
                gearRatio.transformRotation(direction).toInvertedValue());

        motorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(currentLimit);

        motorConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(climberConfig.ControlConstants.climberRampRate);

        motor.clearStickyFaults();
        motor.getConfigurator().apply(motorConfig);

    }





    public record DeviceIdentifiers(NumericId leftClimberMotorID, NumericId rightClimberMotorID) {}
    public record DeviceProperties(MotorProperties leftClimberMotorProperties, MotorProperties rightClimberMotorProperties) {}
    public record DeviceLimits(Current leftClimberMotorCurrentLimit, Current rightClimberMotorCurrentLimit) {}
    public record DeviceConventions(RotationalDirection leftClimberMotorRotationalPositiveDirection,
                                    RotationalDirection rightClimberMotorRotationalPositiveDirection) {}
    public record PhysicalDescription(GearRatio leftClimberMotorGearRatio, GearRatio rightClimberMotorGearRatio) {}
    public record ControlConstants(Time climberRampRate) {}
    public record Config(DeviceIdentifiers Identifiers, DeviceProperties Properties, DeviceLimits Limits,
                         DeviceConventions Conventions, PhysicalDescription PhysicalDescription, ControlConstants ControlConstants) {}
}
