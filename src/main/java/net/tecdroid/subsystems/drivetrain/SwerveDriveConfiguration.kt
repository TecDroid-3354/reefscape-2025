package net.tecdroid.subsystems.drivetrain

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Inches
import net.tecdroid.subsystems.drivetrain.SwerveDrive.DeviceIdentifiers
import net.tecdroid.util.*
import net.tecdroid.util.geometry.Square

object SwerveDriveConfiguration {
    object Conventions {
        val spatialConvention = SpatialConvention(
            LongitudinalDirection.Front,
            TransversalDirection.Left,
            VerticalDirection.Up,
            RotationalDirection.Counterclockwise
        )
    }

    object DeviceIds {
        private val imuId = NumericId(1)
        internal val deviceIdentifiers = DeviceIdentifiers(imuId)
    }

    object Structure {
        private val chassisDimensions = Square(Inches.of(22.3))
        private val scaledChassisDimensions = chassisDimensions.scale(0.5)

        internal val moduleOffsets = arrayOf(
            Translation2d(
                Conventions.spatialConvention.translateFront(scaledChassisDimensions.length),
                Conventions.spatialConvention.translateRight(scaledChassisDimensions.width)
            ),

            Translation2d(
                Conventions.spatialConvention.translateFront(scaledChassisDimensions.length),
                Conventions.spatialConvention.translateLeft(scaledChassisDimensions.width)
            ),

            Translation2d(
                Conventions.spatialConvention.translateBack(scaledChassisDimensions.length),
                Conventions.spatialConvention.translateLeft(scaledChassisDimensions.width)
            ),

            Translation2d(
                Conventions.spatialConvention.translateBack(scaledChassisDimensions.length),
                Conventions.spatialConvention.translateRight(scaledChassisDimensions.width)
            )
        )

    }

    private val moduleConfig =
        SwerveDrive.ModuleConfig(SwerveModuleConfiguration.configurations.zip(Structure.moduleOffsets))

    val configuration = SwerveDrive.Config(moduleConfig, DeviceIds.deviceIdentifiers)
}