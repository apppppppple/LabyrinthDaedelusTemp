package bot

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.PIDCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.Trigger
import java.util.function.BooleanSupplier

class AutoBalance(swerve: Swerve, t: Translation2d) : SequentialCommandGroup(
	swerve.run({ swerve.drive(t, 0.0, false) })
		.until(Trigger(BooleanSupplier { swerve.getTiltMagnitude() > 0.2 }))
		.withTimeout(1.0),
	PIDCommand(PIDController(2.1, 0.005, 5000.0).apply { setTolerance(0.04, 0.05) }, swerve::getTiltMagnitude, 0.0, { output -> null }, swerve)
		.andThen(swerve.run(swerve::lockModules)))
{
}
