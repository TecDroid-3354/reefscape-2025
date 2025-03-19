package net.tecdroid.systems

import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj2.command.button.Trigger
import net.tecdroid.core.RobotContainer
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.subsystems.drivetrain.SwerveDriveConfig
import net.tecdroid.util.units.radians


class SwerveSystem(swerveDriveConfig: SwerveDriveConfig) {
     val drive = SwerveDrive(
        config = swerveDriveConfig
    )


    private val heading
        get() = drive.heading

    init {
        drive.matchRelativeEncodersToAbsoluteEncoders()
        drive.publishToShuffleboard()
    }

    fun linkReorientationTrigger(trigger: Trigger) {
        trigger.onTrue(drive.setHeadingCommand(0.0.radians))
    }
}