package net.tecdroid.subsystems.drivetrain;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveDriveTab {
    private static final String TAB_NAME = "Swerve Drive";

    private static final int MODULE_HEIGHT = 3;
    private static final int MODULE_WIDTH  = 2;
    private static final int MODULE_STRAFE = 2;

    private static final int DRIVE_HEIGHT = 3;
    private static final int DRIVE_WIDTH  = 3;

    private final ShuffleboardTab tab;
    private int moduleCount = 0;

    SwerveDriveTab() {
        tab = Shuffleboard.getTab(TAB_NAME);
    }

    public void publishDrive(SwerveDrive swerveDrive) {
        tab.add("Swerve", swerveDrive).withSize(DRIVE_WIDTH, DRIVE_HEIGHT)
                            .withPosition(0, 3);
    }

    public void publishModules(SwerveModule... modules) {
        for (SwerveModule module : modules) {
            tab.add("Module: " + module.getModuleDigit(), module).withSize(MODULE_WIDTH, MODULE_HEIGHT)
                           .withPosition(moduleCount * MODULE_STRAFE, 0);

            moduleCount++;
        }
    }
}
