package net.tecdroid.subsystems.generic

import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.kt.volts

interface VoltageControlledSubsystem {
    fun setVoltage(voltage: Voltage)
    fun setVoltageCommand(voltage: Voltage) = Commands.runOnce({setVoltage(voltage) })
    fun stop() = setVoltage(0.0.volts)
    fun stopCommand() = Commands.runOnce(::stop)
}

