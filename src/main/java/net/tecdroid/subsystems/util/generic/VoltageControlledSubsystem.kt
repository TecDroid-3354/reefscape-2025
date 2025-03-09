package net.tecdroid.subsystems.util.generic

import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.util.units.volts

interface VoltageControlledSubsystem {
    fun setVoltage(voltage: Voltage)
    fun setVoltageCommand(voltage: Voltage): Command = Commands.runOnce({ setVoltage(voltage) })
    fun stop() = setVoltage(0.0.volts)
    fun stopCommand(): Command = Commands.runOnce(::stop)
}

