package bot

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem

// TODO: varargs
class BetterProxy(private val sup : () -> Command, req: Subsystem) : CommandBase() {
	private var cmd: Command? = null

	init { addRequirements(req) }

	override fun initialize() {
		cmd = sup()
		cmd?.schedule()
	}

	override fun end(interrupted: Boolean) {
		if (interrupted) cmd?.cancel()
		cmd = null
	}

	override fun isFinished() = !(cmd?.isScheduled() ?: false)

	override fun runsWhenDisabled() = true
}
