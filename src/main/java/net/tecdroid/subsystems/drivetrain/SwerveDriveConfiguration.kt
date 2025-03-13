package net.tecdroid.subsystems.drivetrain

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Distance
import net.tecdroid.util.*
import net.tecdroid.util.geometry.Rectangle
import net.tecdroid.util.geometry.Square

data class SwerveDriveConfig(
    val longestDiagonal: Distance,
    val moduleConfigs: List<Pair<SwerveModuleConfig, Translation2d>>,
    val imuId: NumericId
)

fun makeSquareChassisConfig(convention: SpatialConvention, moduleConfigs: List<SwerveModuleConfig>, geometry: Rectangle, imuId: NumericId): SwerveDriveConfig {
    val scaledGeometry = geometry.scale(0.5)

    return SwerveDriveConfig(
        geometry.diagonalLength,
        moduleConfigs.zip(listOf(
            Translation2d(
                convention.translateFront(scaledGeometry.length),
                convention.translateRight(scaledGeometry.width)
            ),

            Translation2d(
                convention.translateFront(scaledGeometry.length),
                convention.translateLeft(scaledGeometry.width)
            ),

            Translation2d(
                convention.translateBack(scaledGeometry.length),
                convention.translateLeft(scaledGeometry.width)
            ),

            Translation2d(
                convention.translateBack(scaledGeometry.length),
                convention.translateRight(scaledGeometry.width)
            )

        )),
        imuId
    )
}

val swerveDriveConfiguration = makeSquareChassisConfig(
    convention = SpatialConvention(
        longitudinalDirection = LongitudinalDirection.Front,
        transversalDirection = TransversalDirection.Left,
        verticalDirection = VerticalDirection.Up,
        rotationalDirection = RotationalDirection.Counterclockwise
    ),
    moduleConfigs = listOf(frontRightModuleConfig, frontLeftModuleConfig, backLeftModuleConfig, backRightModuleConfig),
    geometry = Square(Inches.of(27.5)),
    imuId = NumericId(1)
)
