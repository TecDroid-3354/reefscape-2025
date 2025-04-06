package net.tecdroid.subsystems.climber.climberElevator

import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Time
import net.tecdroid.util.*

import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.DistanceUnit
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.mechanical.Reduction
import net.tecdroid.safety.MeasureLimits
import net.tecdroid.util.amps
import net.tecdroid.util.rotations
import net.tecdroid.util.seconds

data class ClimberElevatorConfig (
    val motorControllerId: NumericId,
    val motorDirection: RotationalDirection,
    val motorCurrentLimit: Current,

    val shaftDiameter: Circle,
    val reduction: Reduction,

    val measureLimits: MeasureLimits<DistanceUnit>,
    val controlGains: ControlGains,
    val motionTargets: LinearMotionTargets
)

public val climberElevatorConfig = ClimberElevatorConfig(
    motorControllerId = NumericId(82), // TODO 82 IS JUST AN ARBITRARY VALUE. CHECK THE REAL NUM. ID
    motorDirection = RotationalDirection.Counterclockwise, // TODO VERIFY DIRECTION WHEN THE CLIMBER IS SET
    motorCurrentLimit = 40.0.amps,

    shaftDiameter = Circle.fromRadius(Inches.of(0.5)), // This shaft (flecha) is the one motors are turning around
    reduction = Reduction(0.0), // TODO ask someone from mecanica for the reduction --> maybe GearRatio = GearRatio(288, 1, 0)

    measureLimits = MeasureLimits(
        0.0.meters, // TODO: min absolute
        0.0.meters, // TODO: min relative
        0.0.meters, // TODO: max relative
        0.0.meters // TODO: max absolute
    ),

    controlGains = ControlGains( // TODO: GET THEM FROM SYSID
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ),

    motionTargets = LinearMotionTargets(
        cruiseVelocity = 0.2.meters.per(Second),
        accelerationTimePeriod = 0.25.seconds,
        jerkTimePeriod = 0.1.seconds
    )
)

/* reductions from the past climber
object ClimberConfigs {
    val climberConfiguration: ClimberController.Config = Config(

        Structure().climberPhysicalDescription,
    )

    private class Structure {
        // Setting a common physical gear ratio for both motors
        private val climberMotorsGR: GearRatio = GearRatio(288, 1, 0)
        private val encoderOffset: Angle = Units.Rotations.of(0.538)

        val climberPhysicalDescription: ClimberController.PhysicalDescription = PhysicalDescription(
            climberMotorsGR, encoderOffset
        )
    }
    }
}*/