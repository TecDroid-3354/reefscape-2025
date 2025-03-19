package net.tecdroid.core

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Hertz
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.constants.GenericConstants.driverControllerId
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.subsystems.drivetrain.swerveDriveConfiguration
import net.tecdroid.subsystems.elevator.elevatorConfig
import net.tecdroid.subsystems.elevatorjoint.elevatorJointConfig
import net.tecdroid.subsystems.intake.intakeConfig
import net.tecdroid.subsystems.wrist.wristConfig
import net.tecdroid.systems.ArmOrders
import net.tecdroid.systems.ArmPoses
import net.tecdroid.systems.ArmSystem
import net.tecdroid.util.units.degrees
import net.tecdroid.util.units.seconds


class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerve = SwerveDrive(swerveDriveConfiguration)
    private val arm = ArmSystem(wristConfig, elevatorConfig, elevatorJointConfig, intakeConfig)
    private var isNormalMode = true
    private val pollNormalMode = { isNormalMode }
    private var isLow = false
    private val pollIsLow = { isLow }
    private val makeLow = { Commands.runOnce({ isLow = true }) }
    private val makeHigh = { Commands.runOnce({ isLow = false }) }

    // Swerve Control
    private val accelerationPeriod = 0.1.seconds
    private val decelerationPeriod = accelerationPeriod

    private val da = accelerationPeriod.asFrequency()
    private val dd = -decelerationPeriod.asFrequency()

    private val longitudinalRateLimiter = SlewRateLimiter(da.`in`(Hertz), dd.`in`(Hertz), 0.0)
    private val transversalRateLimiter = SlewRateLimiter(da.`in`(Hertz), dd.`in`(Hertz), 0.0)
    private val angularRateLimiter = SlewRateLimiter(da.`in`(Hertz), dd.`in`(Hertz), 0.0)

    var vx = { swerve.maxLinearVelocity * longitudinalRateLimiter.calculate(controller.leftY * 0.85) }
    var vy = { swerve.maxLinearVelocity * transversalRateLimiter.calculate(controller.leftX * 0.85) }
    var vw = { swerve.maxAngularVelocity * angularRateLimiter.calculate(controller.rightX * 0.85) }

    init {
        //linkPoses()
        swerve.heading = 0.0.degrees
    }

    fun autonomousInit() {

    }

    fun teleopInit() {
        linkMovement()
    }

    private fun linkMovement() {
        controller.start().onTrue(swerve.setHeadingCommand(0.0.degrees))
        swerve.defaultCommand = Commands.run({ swerve.driveFieldOriented(ChassisSpeeds(vx(), vy(), vw()))}, swerve)
    }

    private fun linkPoses() {
        controller.povLeft().onTrue(Commands.runOnce({ isNormalMode = !isNormalMode }))

        controller.y().onTrue(
            Commands.either(
                arm.setPoseCommand(
                    ArmPoses.L4.pose,
                    ArmOrders.JEW.order
                ),
                arm.setPoseCommand(
                    ArmPoses.Barge.pose,
                    ArmOrders.JEW.order
                ).andThen(makeHigh()),
                pollNormalMode
            )
        )

        controller.b().onTrue(
            Commands.either(
                arm.setPoseCommand(
                    ArmPoses.L3.pose,
                    ArmOrders.JEW.order
                ),
                arm.setPoseCommand(
                    ArmPoses.A2.pose,
                    if (pollIsLow()) ArmOrders.JWE.order else ArmOrders.EWJ.order
                ).andThen(makeHigh()),
                pollNormalMode
            )
        )

        controller.a().onTrue(
            Commands.either(
                arm.setPoseCommand(
                    ArmPoses.L2.pose,
                    ArmOrders.EJW.order
                ),
                arm.setPoseCommand(
                    ArmPoses.A1.pose,
                    if (pollIsLow()) ArmOrders.JWE.order else ArmOrders.EWJ.order
                ).andThen(makeHigh()),
                pollNormalMode
            )
        )

        controller.x().onTrue(
            Commands.either(
                arm.setPoseCommand(
                    ArmPoses.CoralStation.pose,
                    ArmOrders.EJW.order
                ),
                arm.setPoseCommand(
                    ArmPoses.Processor.pose,
                    ArmOrders.EWJ.order
                ).andThen(makeLow()),
                pollNormalMode
            )
        )

        controller.rightBumper().onTrue(arm.enableIntake()).onFalse(arm.disableIntake())
        controller.leftBumper().onTrue(arm.enableOuttake()).onFalse(arm.disableIntake())
    }

    val autonomousCommand: Command = Commands.none()
}





