package net.tecdroid.core

import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import net.tecdroid.util.units.degrees

class Robot : TimedRobot() {

    private val container = RobotContainer()
    val timer = Timer()

    override fun robotInit() {
        DriverStation.silenceJoystickConnectionWarning(true)
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun disabledPeriodic() {
    }

    override fun disabledExit() {
    }

    override fun autonomousInit() {
        timer.restart()
        timer.reset()
        timer.start()

    }

    override fun autonomousPeriodic() {
        for (module in container.swerve.drive.modules) {
            module.setTargetAngle(0.0.degrees)
        }

        val power = 0.25

        if (timer.get() < 1.5) {
            container.swerve.drive.modules[0].setPower(power)
            container.swerve.drive.modules[1].setPower(power)
            container.swerve.drive.modules[2].setPower(power)
            container.swerve.drive.modules[3].setPower(-power)


        } else {
            container.swerve.drive.setPower(0.0)
        }

    }

    override fun teleopInit() {
        container.initial()
        SmartDashboard.putData("VX: ") { container.vx() }
        SmartDashboard.putData("VY: ") { container.vy() }
        SmartDashboard.putData("VW: ") { container.vw() }
    }

    override fun teleopPeriodic() { }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {
    }

    override fun simulationInit() {
    }

    override fun simulationPeriodic() {
    }
}
