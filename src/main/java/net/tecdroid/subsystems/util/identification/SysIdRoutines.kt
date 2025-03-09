package net.tecdroid.subsystems.util.identification

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine

class SysIdRoutines(
    val quasistaticForward: Command,
    val quasistaticBackward: Command,
    val dynamicForward: Command,
    val dynamicBackward: Command
) {}

abstract class GenericSysIdRoutine {
    abstract val routine: SysIdRoutine
    protected val voltage: MutVoltage = Volts.mutable(0.0)

    private fun createQuasistaticTest(direction: SysIdRoutine.Direction) = routine.quasistatic(direction)
    private fun createDynamicTest(direction: SysIdRoutine.Direction) = routine.dynamic(direction)

    fun createTests() = SysIdRoutines(
        quasistaticForward = createQuasistaticTest(SysIdRoutine.Direction.kForward),
        quasistaticBackward = createQuasistaticTest(SysIdRoutine.Direction.kReverse),
        dynamicForward = createDynamicTest(SysIdRoutine.Direction.kForward),
        dynamicBackward = createDynamicTest(SysIdRoutine.Direction.kReverse),
    )
}

abstract class LinearSysIdRoutine: GenericSysIdRoutine() {
    protected val position: MutDistance = Meters.mutable(0.0)
    protected val velocity: MutLinearVelocity = MetersPerSecond.mutable(0.0)
}

abstract class AngularSysIdRoutine: GenericSysIdRoutine() {
    protected val position: MutAngle = Radians.mutable(0.0)
    protected val velocity: MutAngularVelocity = RadiansPerSecond.mutable(0.0)
}
