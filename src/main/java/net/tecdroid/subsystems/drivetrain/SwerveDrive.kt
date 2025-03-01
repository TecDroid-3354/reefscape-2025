package net.tecdroid.subsystems.drivetrain

import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import net.tecdroid.kt.toRotation2d
import net.tecdroid.util.NumericId
import net.tecdroid.util.geometry.Rectangle
import kotlin.math.PI

class SwerveDrive(private val config: Config) : SubsystemBase() {
    private val gyro = Pigeon2(config.identifierConfig.imuId.id)

    private val modules = config.moduleConfigurations.moduleConfigs.map { SwerveModule(it.first) }

    private val kinematics =
        SwerveDriveKinematics(*config.moduleConfigurations.moduleConfigs.map { it.second }.toTypedArray())
    private val odometry = SwerveDriveOdometry(kinematics, heading.toRotation2d(), modulePositions.toTypedArray())

    init {
        this.configureImuInterface()
    }

    override fun periodic() {
        updateOdometry()
    }

    private fun setModuleTargetStates(vararg states: SwerveModuleState) {
        require(states.size == modules.size) { "You must provide as many states as there are modules" }

        for (i in states.indices) {
            modules[i].setTargetState(states[i])
        }
    }

    fun drive(chassisSpeeds: ChassisSpeeds) {
        val desiredStates = kinematics.toSwerveModuleStates(chassisSpeeds)
        setModuleTargetStates(*desiredStates)
    }

    fun matchModuleSteeringEncodersToAbsoluteEncoders() {
        for (module in modules) {
            module.matchSteeringEncoderToAbsoluteEncoder()
        }
    }

    // //////// //
    // Odometry //
    // //////// //

    private fun updateOdometry() {
        odometry.update(heading.toRotation2d(), modulePositions.toTypedArray())
    }

    var pose: Pose2d
        get() = odometry.poseMeters
        set(newPose) {
            odometry.resetPosition(heading.toRotation2d(), modulePositions.toTypedArray(), newPose)
        }

    var heading: Angle
        get() = gyro.yaw.value
        set(angle) {
            gyro.setYaw(angle)
        }

    val moduleStates: List<SwerveModuleState>
        get() = modules.map { SwerveModuleState(it.wheelLinearVelocity, it.wheelAzimuth.toRotation2d()) }

    val modulePositions: List<SwerveModulePosition>
        get() = modules.map { SwerveModulePosition(it.wheelLinearDisplacement, it.wheelAzimuth.toRotation2d()) }

    val maxLinearVelocity: LinearVelocity =
        modules.map { it.wheelMaxLinearVelocity }.minByOrNull { it.`in`(MetersPerSecond) }!!

    val maxAngularVelocity: AngularVelocity =
        Rotations.one().div((config.physicalDescription.dimensions.diagonalLength * PI) / maxLinearVelocity);


    // ///////////// //
    // Configuration //
    // ///////////// //

    private fun configureImuInterface() {
        val imuConfiguration = Pigeon2Configuration()

        with(imuConfiguration) {
            // Do stuff
        }

        gyro.clearStickyFaults()
        gyro.configurator.apply(imuConfiguration)
    }

    /**
     * Stores Module-Translation pairs to construct the drive's kinematics
     */
    data class ModuleConfigurations(val moduleConfigs: List<Pair<SwerveModule.Config, Translation2d>>)

    /**
     * Stores the device identifiers that the swerve drive requires
     */
    data class DeviceIdentifiers(val imuId: NumericId)

    data class 
    PhysicalDescription(val dimensions: Rectangle)

    data class Config(
        val moduleConfigurations: ModuleConfigurations,
        val identifierConfig: DeviceIdentifiers,
        val physicalDescription: PhysicalDescription
    )

    val talons = modules.map { it.talon }
}
