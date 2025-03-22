package net.tecdroid.subsystems.drivetrain

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Distance
import net.tecdroid.util.*
import net.tecdroid.util.Rectangle
import net.tecdroid.util.Square

data class SwerveDriveConfig(
    val longestDiagonal: Distance,
    val moduleConfigs: List<Pair<SwerveModuleConfig, Translation2d>>,
    val imuId: NumericId
)

fun makeSquareChassisConfig(convention: SpatialConvention, moduleConfigs: List<SwerveModuleConfig>, wheelGeometry: Rectangle, chassisGeometry: Rectangle, imuId: NumericId): SwerveDriveConfig {
    val scaledGeometry = wheelGeometry.scale(0.5)

    return SwerveDriveConfig(
        chassisGeometry.diagonalLength,
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
    wheelGeometry = Square(Inches.of(22.229226)),
    chassisGeometry = Square(Inches.of(27.5)),
    imuId = NumericId(1)
)
