package net.tecdroid.subsystems.drivetrain

import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import net.tecdroid.constants.subsystemTabName
import net.tecdroid.subsystems.util.generic.WithAbsoluteEncoders
import net.tecdroid.util.units.degrees
import net.tecdroid.util.units.rotations
import net.tecdroid.util.units.toRotation2d
import kotlin.math.PI

class SwerveDrive(private val config: SwerveDriveConfig) : SubsystemBase(), Sendable, WithAbsoluteEncoders {
    private val imu = Pigeon2(config.imuId.id)

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

    override fun matchRelativeEncodersToAbsoluteEncoders() {
        for (module in modules) {
            module.matchRelativeEncodersToAbsoluteEncoders()
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
        get() = imu.yaw.value
        set(angle) {
            imu.setYaw(angle)
        }

    fun setHeadingCommand(angle: Angle) = Commands.runOnce({ heading = angle });

    val moduleStates: List<SwerveModuleState>
        get() = modules.map { SwerveModuleState(it.wheelLinearVelocity, it.wheelAzimuth.toRotation2d()) }

    val modulePositions: List<SwerveModulePosition>
        get() = modules.map { SwerveModulePosition(it.wheelLinearDisplacement, it.wheelAzimuth.toRotation2d()) }

    val maxLinearVelocity: LinearVelocity =
        modules.map { it.wheelMaxLinearVelocity }.minByOrNull { it.`in`(MetersPerSecond) }!!

    val maxAngularVelocity: AngularVelocity =
        Rotations.one().div((config.longestDiagonal * PI) / maxLinearVelocity);

    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            addDoubleProperty("Heading (Degrees)", { heading.`in`(Degrees) }) {}
        }
    }

    fun publishToShuffleboard() {
        val tab = Shuffleboard.getTab(subsystemTabName)
        tab.add("Swerve Drive", this)
    }


    // ///////////// //
    // Configuration //
    // ///////////// //

    private fun configureImuInterface() {
        val imuConfiguration = Pigeon2Configuration()

        with(imuConfiguration) {
            MountPose.withMountPoseYaw((-90.0).degrees)
        }

        imu.clearStickyFaults()
        imu.configurator.apply(imuConfiguration)
    }

}