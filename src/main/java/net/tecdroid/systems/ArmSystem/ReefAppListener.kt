package net.tecdroid.systems.ArmSystem

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import net.tecdroid.util.NumericId
import net.tecdroid.vision.limelight.systems.LimeLightChoice

data class BranchChoice (
    var apriltagId: NumericId,
    var levelPose: PoseCommands,
    var sideChoice: LimeLightChoice
)
class ReefAppListener: SubsystemBase() {
    // Network table
    private val table = NetworkTableInstance.getDefault().getTable("ReefAppData")

    // Branch choice object
    val branchChoice = BranchChoice(NumericId(0), PoseCommands.L2, LimeLightChoice.Right)

    private fun shuffleboardData() {
        val tab = Shuffleboard.getTab("Driver Tab")
        tab.addString("Branch Choice", {
            branchChoice.apriltagId.toString() + " " + branchChoice.levelPose.toString() + " " + branchChoice.sideChoice.toString()
        })

    }

    init {
        shuffleboardData()
    }

    override fun periodic() {
        val apriltagId = table.getEntry("ApriltagId").getString("Apriltag id not found")
        val level = table.getEntry("Level").getString("Level not found")
        val side = table.getEntry("Side").getString("Side not found")

        // Select apriltag id
        DriverStation.getAlliance().takeIf { it.isPresent }?.get()?.let {
            if (it == Alliance.Red) {
                branchChoice.apriltagId = when(apriltagId) {
                    "1" -> NumericId(8)
                    "2" -> NumericId(7)
                    "3" -> NumericId(6)
                    "4" -> NumericId(11)
                    "5" -> NumericId(10)
                    "6" -> NumericId(9)
                    else -> NumericId(0)
                }
            } else if (it == Alliance.Blue) {
                branchChoice.apriltagId = when(apriltagId) {
                    "1" -> NumericId(17)
                    "2" -> NumericId(18)
                    "3" -> NumericId(19)
                    "4" -> NumericId(20)
                    "5" -> NumericId(21)
                    "6" -> NumericId(22)
                    else -> NumericId(0)
                }
            }
        }

        // Select level
        when (level) {
            "L2" -> branchChoice.levelPose = PoseCommands.L2
            "L3" -> branchChoice.levelPose = PoseCommands.L3
            "L4" -> branchChoice.levelPose = PoseCommands.L4
            else -> branchChoice.levelPose = PoseCommands.L2
        }

        // Select side
        when (side) {
            "right" -> branchChoice.sideChoice = LimeLightChoice.Right
            "left" -> branchChoice.sideChoice = LimeLightChoice.Left
            else -> branchChoice.sideChoice = LimeLightChoice.Right
        }

    }
}