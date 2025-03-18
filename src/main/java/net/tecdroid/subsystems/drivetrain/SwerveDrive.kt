package net.tecdroid.subsystems.drivetrain

import choreo.trajectory.SwerveSample
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import net.tecdroid.constants.subsystemTabName
import net.tecdroid.util.units.degrees
import net.tecdroid.util.units.toRotation2d
import kotlin.math.PI


class SwerveDrive(private val config: SwerveDriveConfig) : SubsystemBase(), Sendable {
    private val imu = Pigeon2(config.imuId.id)
     val modules = config.moduleConfigs.map { SwerveModule(it.first) }
    private val kinematics = SwerveDriveKinematics(*config.moduleConfigs.map { it.second }.toTypedArray())
    private val odometry = SwerveDriveOdometry(kinematics, heading.toRotation2d(), modulePositions.toTypedArray())
     val field = Field2d();

    val forwardsPid = PIDController(0.1, 0.0, 0.0)
    val sidewaysPid = PIDController(0.1, 0.0, 0.0)
    val thetaPid = PIDController(0.1, 0.0, 0.0)

    init {
        this.configureImuInterface()
        matchRelativeEncodersToAbsoluteEncoders()

        thetaPid.enableContinuousInput(-PI, PI)
        SmartDashboard.putData("Field", field)
    }

    fun followTrajectory(sample: SwerveSample) {
        val speeds = ChassisSpeeds(
            sample.vx + forwardsPid.calculate(pose.x, sample.x),
            sample.vy + sidewaysPid.calculate(pose.y, sample.y),
            sample.omega + thetaPid.calculate(pose.rotation.radians, sample.heading)
        )

        drive(speeds)
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

    fun driveFieldOriented(chassisSpeeds: ChassisSpeeds) {
        val fieldOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, heading.toRotation2d())
        drive(fieldOrientedSpeeds)
    }

    fun driveFieldOrientedCMD(chassisSpeeds: ChassisSpeeds) : Command {
        return Commands.runOnce({ driveFieldOriented(chassisSpeeds) })
    }

    fun drive(chassisSpeeds: ChassisSpeeds) {
        val desiredStates = kinematics.toSwerveModuleStates(chassisSpeeds)
        setModuleTargetStates(*desiredStates)
    }

    fun driveCMD(chassisSpeeds: ChassisSpeeds) : Command {
        return Commands.runOnce({ drive(chassisSpeeds) })
    }

    fun matchRelativeEncodersToAbsoluteEncoders() {
        for (module in modules) {
            module.matchRelativeEncodersToAbsoluteEncoders()
        }
    }

    private fun updateOdometry() {
        odometry.update(heading.toRotation2d(), modulePositions.toTypedArray())
        pose = odometry.poseMeters
        field.robotPose = pose
    }

    fun resetOdometry(pose: Pose2d) {
        odometry.resetPosition(heading.toRotation2d(), modulePositions.toTypedArray(), pose)
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

    fun setHeadingCommand(angle: Angle) = Commands.runOnce({ heading = angle })

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
        tab.add("field", field)
        for ((i, m) in modules.withIndex()) {
            tab.add("Module $i", m)
        }
    }

    private fun configureImuInterface() {
        val imuConfiguration = Pigeon2Configuration()

        with(imuConfiguration) {
            MountPose.withMountPoseYaw((-90.0).degrees)
        }

        imu.clearStickyFaults()
        imu.configurator.apply(imuConfiguration)
    }

    fun setPower(power: Double) {
        for (module in modules) {
            module.setPower(power)
        }
    }

    fun alignModules(): Command {
        return Commands.runOnce({ for (module in modules) module.align() })
    }

}