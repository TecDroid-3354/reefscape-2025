package net.tecdroid.subsystems.drivetrain;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import java.util.Arrays;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static net.tecdroid.subsystems.drivetrain.SwerveDriveUtil.convertFromDriveLinearVelocityToDriveAngularVelocity;

public class SwerveTuner {

    private final SwerveDrive drivetrain;
    private Voltage currentVoltage;

    public SwerveTuner(SwerveDrive drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void applyVoltage(Voltage voltage) {
        drivetrain.setVoltage(new VoltageOut(voltage));
    }

    public Command determineKs(AngularVelocity tolerance) {
        return new FunctionalCommand(
                () -> {
                    currentVoltage = Volts.of(0.0);
                },
                () -> {
                    applyVoltage(currentVoltage);
                    currentVoltage = currentVoltage.plus(Volts.of(0.001));
                    SmartDashboard.putNumber("[KS ROUTINE] CURRENT VOLTAGE", currentVoltage.in(Volts));
                    SmartDashboard.putNumber("[KS ROUTINE] CURRENT VELOCITY", moduleVelocityAverage().in(RotationsPerSecond));

                },
                (Boolean b) -> {
                    applyVoltage(Volts.of(0.0));
                    SmartDashboard.putNumber("[KS ROUTINE] DETERMINED KS", currentVoltage.in(Volts));
                },
                () -> moduleVelocityAverage().gt(tolerance)
        );
    }

    public Command determineKv(AngularVelocity targetVelocity, AngularVelocity tolerance) {
        AngularVelocity lowerBound = targetVelocity.minus(tolerance);
        AngularVelocity upperBound = targetVelocity.plus(tolerance);

        return new FunctionalCommand(
                () -> {
                    currentVoltage = Volts.of(0.0);
                },
                () -> {
                    AngularVelocity driveVelocity = moduleVelocityAverage();

                    applyVoltage(currentVoltage);

                    SmartDashboard.putNumber("[KV ROUTINE] CURRENT VOLTAGE", currentVoltage.in(Volts));
                    SmartDashboard.putNumber("[KV ROUTINE] CURRENT VELOCITY", moduleVelocityAverage().in(RotationsPerSecond));

                    if (driveVelocity.lt(lowerBound)) {
                        currentVoltage = currentVoltage.plus(Volts.of(0.001));
                    }

                    if (driveVelocity.gt(upperBound)) {
                        currentVoltage = currentVoltage.minus(Volts.of(0.001));
                    }

                },
                (Boolean b) -> {
                    applyVoltage(Volts.of(0.0));
                    SmartDashboard.putNumber("[KV ROUTINE] DETERMINED KV", currentVoltage.in(Volts));
                },
                () -> {
                    return moduleVelocityAverage().gt(lowerBound) && moduleVelocityAverage().lt(upperBound);
                }
        );
    }

    public AngularVelocity moduleVelocityAverage() {
        SwerveModule[] modules = drivetrain.getModules();
        AngularVelocity[] velocities = Arrays.stream(modules).map((SwerveModule it) -> convertFromDriveLinearVelocityToDriveAngularVelocity(it.getWheelLinearVelocity())).toArray(AngularVelocity[]::new);
        return (velocities[0].plus(velocities[1]).plus(velocities[2]).plus(velocities[3])).div(4.0);
    }
}
