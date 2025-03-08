package net.tecdroid.subsystems.drivetrain

import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import net.tecdroid.kt.toRotation2d
import kotlin.math.PI

class SwerveDrive(private val config: SwerveDriveConfig) : SubsystemBase() {
    private val gyro = Pigeon2(config.imuId.id)

    private val modules = config.moduleConfigs.map { SwerveModule(it.first) }

    private val kinematics =
        SwerveDriveKinematics(*config.moduleConfigs.map { it.second }.toTypedArray())
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

    private fun setModuleTargetAzimuth(angle: Angle) {
        for (module in modules) {
            module.setTargetState(SwerveModuleState(MetersPerSecond.of(0.0), angle.toRotation2d()))
        }
    }

    fun setModuleTargetAzimuthCommand(angle: Angle): Command {
        return Commands.runOnce({
            setModuleTargetAzimuth(angle)
        }, this)
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
        Rotations.one().div((config.longestDiagonal * PI) / maxLinearVelocity);


    // ///////////// //
    // Configuration //
    // ///////////// //

    private fun configureImuInterface() {
        val imuConfiguration = Pigeon2Configuration()

        gyro.clearStickyFaults()
        gyro.configurator.apply(imuConfiguration)
    }

}
