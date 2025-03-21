package net.tecdroid.subsystems.drivetrain

import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import net.tecdroid.util.units.degrees
import net.tecdroid.util.units.toRotation2d
import kotlin.math.PI

class SwerveDrive(private val config: SwerveDriveConfig) : SubsystemBase() {
    val imu = Pigeon2(config.imuId.id)
    private val modules = config.moduleConfigs.map { SwerveModule(it.first) }
    private val kinematics = SwerveDriveKinematics(*config.moduleConfigs.map { it.second }.toTypedArray())

    private val stateStdDevs = VecBuilder.fill(0.005, 0.005, Units.degreesToRadians(2.0))
    private val visionStdDevs = VecBuilder.fill(0.005, 0.005, Units.degreesToRadians(2.0))

    val poseEstimator = SwerveDrivePoseEstimator(
        kinematics, heading.toRotation2d(), modulePositions.toTypedArray(), Pose2d(),
        stateStdDevs, visionStdDevs
    )

    private val field = Field2d()

    var pose: Pose2d
        get() = poseEstimator.estimatedPosition
        set(value) { poseEstimator.resetPosition(heading.toRotation2d(), modulePositions.toTypedArray(), value) }

    var heading: Angle
        get() = imu.yaw.value
        set(angle) {
            imu.setYaw(angle)
        }

    val speeds: ChassisSpeeds
        get() = ChassisSpeedBridge.ktToJavaArrayChassisSpeeds(kinematics, modules.map { it.state })

    init {
        this.configureImuInterface()
        matchRelativeEncodersToAbsoluteEncoders()
        SmartDashboard.putData("Field", field)
    }

    override fun periodic() {
        poseEstimator.update(heading.toRotation2d(), modulePositions.toTypedArray())
        field.robotPose = pose
    }

     // Core //

    private fun setModuleTargetStates(vararg states: SwerveModuleState) {
        require(states.size == modules.size) { "You must provide as many states as there are modules" }

        for (i in states.indices) {
            modules[i].setTargetState(states[i])
        }
    }

    // Drive //

    fun driveFieldOriented(chassisSpeeds: ChassisSpeeds) {
        val fieldOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, heading.toRotation2d())
        driveRobotOriented(fieldOrientedSpeeds)
    }

    fun driveRobotOriented(chassisSpeeds: ChassisSpeeds) {
        val desiredStates = kinematics.toSwerveModuleStates(chassisSpeeds)
        setModuleTargetStates(*desiredStates)
    }

    // Utilities //

    private fun matchRelativeEncodersToAbsoluteEncoders() {
        for (module in modules) {
            module.matchRelativeEncodersToAbsoluteEncoders()
        }
    }

    fun zeroHeading() {
        heading = 0.0.degrees
    }

    /*fun updatePoseEstimationWithLimelight(limelight: Limelight, visionMeasurementPose: Pose2d, visionTimeStamp: Double, currentTimeSeconds: Time) {
        poseEstimator.addVisionMeasurement(visionMeasurementPose, visionTimeStamp)
        poseEstimator.updateWithTime(currentTimeSeconds.`in`(Seconds), heading.toRotation2d(), modulePositions.toTypedArray())
    }*/

    // Misc //

    val modulePositions: List<SwerveModulePosition>
        get() = modules.map { SwerveModulePosition(it.wheelLinearDisplacement, it.wheelAzimuth.toRotation2d()) }

    val maxLinearVelocity: LinearVelocity = modules.map { it.wheelMaxLinearVelocity }.minByOrNull { it.`in`(MetersPerSecond) }!!
    val maxAngularVelocity: AngularVelocity = Rotations.one().div((config.longestDiagonal * PI) / maxLinearVelocity);
    val maxChassisSpeeds = ChassisSpeeds(maxLinearVelocity, maxLinearVelocity, maxAngularVelocity)

    // Configuration //

    private fun configureImuInterface() {
        val imuConfiguration = Pigeon2Configuration()

        with(imuConfiguration) {
            MountPose.withMountPoseYaw((-90.0).degrees)
        }

        imu.clearStickyFaults()
        imu.configurator.apply(imuConfiguration)
    }

    // Commands //

    fun driveRobotRelativeCommand(chassisSpeeds: ChassisSpeeds) : Command {
        return Commands.runOnce({ driveRobotOriented(chassisSpeeds) })
    }

    fun driveFieldOrientedCommand(chassisSpeeds: ChassisSpeeds) : Command {
        return Commands.runOnce({ driveFieldOriented(chassisSpeeds) })
    }

    fun setHeadingCommand(angle: Angle) = Commands.runOnce({ heading = angle })

    fun zeroHeadingCommand() = Commands.runOnce({ zeroHeading() })

}