package net.tecdroid.subsystems.elevatorjoint

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import net.tecdroid.wrappers.ThroughBoreAbsoluteEncoder

class ElevatorJointIOSimulation: ElevatorJointIO {
    override fun getTargetAngle(): Angle {
        TODO("Not yet implemented")
    }

    override fun getAbsoluteEncoderInstance(): ThroughBoreAbsoluteEncoder {
        TODO("Not yet implemented")
    }

    override fun getMotorPosition(): Angle {
        TODO("Not yet implemented")
    }

    override fun getMotorVelocity(): AngularVelocity {
        TODO("Not yet implemented")
    }

    override fun getMotorPower(): Double {
        TODO("Not yet implemented")
    }
}