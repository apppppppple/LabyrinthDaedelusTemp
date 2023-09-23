package bot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser

class RobotContainer {
	private val driver = CommandXboxController(0)
	private val operator = CommandXboxController(1)

	private val swerve = Swerve()
	private val intake = Intake()
	private val elevator = Elevator()
	private val rope = Rope()
	private val chooser = SendableChooser<Command>()

	private fun preset(e: Double, r: Double) = elevator.goToPos(e).alongWith(rope.goToPos(r+0.5))

	init {
		swerve.setDefaultCommand(RunCommand({
			swerve.drive(
				Translation2d(
					MathUtil.applyDeadband(-driver.getLeftY(), 0.05) * Swerve.maxLinVel,
					MathUtil.applyDeadband(-driver.getLeftX(), 0.05) * Swerve.maxLinVel),
				MathUtil.applyDeadband(-driver.getRightX(), 0.05) * Swerve.maxAngVel,
				true)
		}, swerve))

		// shelf cube
		operator.a().whileTrue(preset(19.75, 23.214))
		// shelf cone
		operator.b().whileTrue(preset(24.65, 36.643))
		// cube mid
		operator.x().whileTrue(preset(45.68, 65.17))
		// cube high
		operator.y().whileTrue(preset(74.85, 65.26))

		operator.start().whileTrue(preset(0.0, -0.5))

		operator.leftBumper().whileTrue(RunCommand(intake::intake, intake))
		operator.rightBumper().whileTrue(RunCommand(intake::eject, intake))

		driver.start().onTrue(InstantCommand({ swerve.setPose(Pose2d()) }, swerve))
	}
	
	/// Autos
	init {
		chooser.setDefaultOption("do nothing", InstantCommand({}));
		val taxi = RunCommand({ swerve.drive(Translation2d(-1.0, 0.0), 0.0, false) }, swerve).withTimeout(4.0)

		val putCubeHigh =
			preset(74.85, 65.26)
			.withTimeout(1.0)
			.andThen(
				RunCommand(intake::eject, intake)
				.withTimeout(1.0))
			.andThen(
				preset(0.0, -0.5)
				.withTimeout(1.0))

		val putCubeMid =
			preset(45.68, 65.17)
			.withTimeout(1.0)
			.andThen(
				RunCommand(intake::eject, intake)
				.withTimeout(1.0))
			.andThen(
				preset(0.0, -0.5)
				.withTimeout(1.0))

		chooser.addOption("put cube high", putCubeHigh)
		chooser.addOption("put cube mid", putCubeMid)
		chooser.addOption("put cube high, taxi", putCubeHigh.andThen(taxi))
		chooser.addOption("put cube mid, taxi", putCubeMid.andThen(taxi))
		chooser.addOption("just taxi", taxi)
	}

	fun getAutonomousCommand() = chooser.getSelected()

}
