@file:Suppress("MemberVisibilityCanBePrivate", "unused")

package net.tecdroid.subsystems.drivetrain

import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N3
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
import net.tecdroid.safety.pidOutputRange
import net.tecdroid.util.degrees
import net.tecdroid.util.toRotation2d
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.sqrt

class SwerveDrive(private val config: SwerveDriveConfig) : SubsystemBase() {
    val imu = Pigeon2(config.imuId.id)
    val modules = config.moduleConfigs.map { SwerveModule(it.first) }
    val kinematics = SwerveDriveKinematics(*config.moduleConfigs.map { it.second }.toTypedArray())

    val stateStdDevs: Vector<N3> = VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(2.0))
    val visionStdDevs: Vector<N3> = VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(2.0))

    val poseEstimator = SwerveDrivePoseEstimator(
        kinematics, heading.toRotation2d(), modulePositions.toTypedArray(), Pose2d(),
        stateStdDevs, visionStdDevs
    )

    val xRateLimiter = SlewRateLimiter(config.accelerationPeriod.asFrequency().`in`(Hertz), -config.decelerationPeriod.asFrequency().`in`(Hertz), 0.0)
    val yRateLimiter = SlewRateLimiter(config.accelerationPeriod.asFrequency().`in`(Hertz), -config.decelerationPeriod.asFrequency().`in`(Hertz), 0.0)
    val wRateLimiter = SlewRateLimiter(config.accelerationPeriod.asFrequency().`in`(Hertz), -config.decelerationPeriod.asFrequency().`in`(Hertz), 0.0)

    val directionPIDController = PIDController(config.directionControlGains.p, config.directionControlGains.i, config.directionControlGains.d)
    val lastDirection = Radians.mutable(0.0)

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
        pose = Pose2d()
        SmartDashboard.putData("Field", field)
        directionPIDController.enableContinuousInput(-PI, PI)
    }

    override fun periodic() {
        poseEstimator.update(heading.toRotation2d(), modulePositions.toTypedArray())
        field.robotPose = pose
        SmartDashboard.putNumber("VW", speeds.omegaRadiansPerSecond)
    }

     // Core //

    private fun setModuleTargetStates(vararg states: SwerveModuleState) {
        require(states.size == modules.size) { "You must provide as many states as there are modules" }

        for (i in states.indices) {
            modules[i].setTargetState(states[i])
        }
    }

    // Drive //

    fun driveRobotOriented(chassisSpeeds: ChassisSpeeds) {
        val speeds = transformSpeeds(chassisSpeeds)
        val desiredStates = kinematics.toSwerveModuleStates(speeds)
        setModuleTargetStates(*desiredStates)
        SmartDashboard.putNumber("PASSIN", chassisSpeeds.omegaRadiansPerSecond)
    }

    fun driveFieldOriented(chassisSpeeds: ChassisSpeeds) {
        val fieldOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, heading.toRotation2d())
        driveRobotOriented(fieldOrientedSpeeds)
    }

    fun driveDirected(chassisSpeeds: ChassisSpeeds, angle: Angle) {
        chassisSpeeds.omegaRadiansPerSecond = 0.0

        val output = directionPIDController.calculate(heading.`in`(Radians), angle.`in`(Radians)).coerceIn(pidOutputRange)
        val angularVelocity = maxAngularVelocity * output
        lastDirection.mut_replace(angle)

        driveFieldOriented(
            ChassisSpeeds(
                MetersPerSecond.of(chassisSpeeds.vxMetersPerSecond),
                MetersPerSecond.of(chassisSpeeds.vyMetersPerSecond),
                angularVelocity
            )
        )
    }

    fun stop() {
        driveRobotOriented(ChassisSpeeds(0.0, 0.0, 0.0))
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
    val maxSpeeds = ChassisSpeeds(maxLinearVelocity, maxLinearVelocity, maxAngularVelocity)

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
        return Commands.runOnce({ driveRobotOriented(chassisSpeeds) }, this)
    }

    fun driveFieldOrientedCommand(chassisSpeeds: ChassisSpeeds) : Command {
        return Commands.runOnce({ driveFieldOriented(chassisSpeeds) }, this)
    }

    fun stopCommand(): Command = Commands.runOnce({ stop() }, this)

    fun setHeadingCommand(angle: Angle): Command = Commands.runOnce({ heading = angle }, this)

    fun zeroHeadingCommand(): Command = Commands.runOnce({ zeroHeading() }, this)

    fun transformSpeeds(speeds: ChassisSpeeds): ChassisSpeeds {
        SmartDashboard.putNumber("RPASSIN", speeds.omegaRadiansPerSecond)
        val maxMagnitude = sqrt(maxSpeeds.vxMetersPerSecond * maxSpeeds.vxMetersPerSecond + maxSpeeds.vyMetersPerSecond * maxSpeeds.vyMetersPerSecond)
        val currentMagnitude = sqrt(speeds.vxMetersPerSecond * speeds.vxMetersPerSecond + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond)

        val newSpeeds = if (currentMagnitude > maxMagnitude) {
            ChassisSpeeds(
                (speeds.vxMetersPerSecond / currentMagnitude) * maxSpeeds.vxMetersPerSecond,
                (speeds.vyMetersPerSecond / currentMagnitude) * maxSpeeds.vyMetersPerSecond,
                (speeds.omegaRadiansPerSecond / currentMagnitude) * maxSpeeds.omegaRadiansPerSecond,
            )
        } else speeds

        newSpeeds.vxMetersPerSecond = xRateLimiter.calculate(newSpeeds.vxMetersPerSecond)
        newSpeeds.vyMetersPerSecond = yRateLimiter.calculate(newSpeeds.vyMetersPerSecond)
        newSpeeds.omegaRadiansPerSecond = wRateLimiter.calculate(newSpeeds.omegaRadiansPerSecond)


        return newSpeeds
    }

    fun vector2dToLinearVelocity(x: Double, y: Double): ChassisSpeeds = ChassisSpeeds(
        maxLinearVelocity * x, maxLinearVelocity * y, RadiansPerSecond.zero()
    )

    companion object {
        fun vector2dToTargetAngle(x: Double, y: Double, lastAngle: Angle): Angle {
            return if (x.absoluteValue >= 0.6 || y.absoluteValue >= 0.6) -(Radians.of(atan2(y, x)) - Radians.of(PI / 2.0))
            else lastAngle
        }
    }

}