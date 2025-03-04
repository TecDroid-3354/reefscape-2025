package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final PIDController pidController;
    private final DigitalInput limitSwitch;

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double MAX_HEIGHT = 100.0;  // Valor de ejemplo


    

    public void setClimberPosition(double targetHeight) {
        if (targetHeight > MAX_HEIGHT) targetHeight = MAX_HEIGHT;
        double power = pidController.calculate(getCurrentHeight(), targetHeight);
        setPower(power);
    }

    public void manualControl(double speed) {
        if (!limitSwitch.get() || speed < 0) {
            setPower(speed);
        } else {
            stop();
        }
    }

    private void setPower(double power) {
        leftMotor.set(ControlMode.PercentOutput, power);
        rightMotor.set(ControlMode.PercentOutput, -power); // Motores en direcciones opuestas
    }

    public void stop() {
        leftMotor.set(ControlMode.PercentOutput, 0);
        rightMotor.set(ControlMode.PercentOutput, 0);
    }

    private double getCurrentHeight() {
        return 0; // Aquí iría el encoder para medir la altura
    }

    @Override
    public void periodic() {
        // Aquí puedes agregar telemetría si lo necesitas
    }
}
