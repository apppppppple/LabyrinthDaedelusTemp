package bot

import com.revrobotics.CANSparkMax


import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard


import com.kauailabs.navx.frc.AHRS

import com.revrobotics.CANSparkMax.IdleMode
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.SparkMaxAbsoluteEncoder.Type

import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem

// TODO: Velocity based manual control, requires multiple pid slots
class Rope : TrapezoidProfileSubsystem(TrapezoidProfile.Constraints(600.0, 600.0)) {
	private val motor = CANSparkMax(6, MotorType.kBrushless).apply {
		restoreFactoryDefaults()
		setIdleMode(IdleMode.kBrake)
		setSmartCurrentLimit(30)
		enableVoltageCompensation(11.0)
	}

	private val encoder = motor.getEncoder()
	private val pid = motor.getPIDController().apply {
		setFeedbackDevice(encoder)
		setP(4e-2)
		setI(0.75e-6)
		setD(0.75e-4)
		setOutputRange(-1.0, 1.0)
	}

	init { motor.burnFlash() }

	init { setDefaultCommand(run({ setGoalRevolutions(encoder.getPosition()) })) }

	override fun useState(setpoint: TrapezoidProfile.State) {
		pid.setReference(setpoint.position / (2.0 * Math.PI), CANSparkMax.ControlType.kPosition)
	}

	fun setGoalRevolutions(pos: Double) {
		setGoal(pos * (2.0 * Math.PI))
	}

	fun goToPos(pos: Double) = run({ setGoalRevolutions(pos) })

	override fun periodic() {
		super.periodic()
		SmartDashboard.putNumber("Rope Position", encoder.getPosition())
	}
}
