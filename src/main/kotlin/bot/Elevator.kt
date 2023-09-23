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

class Elevator : TrapezoidProfileSubsystem(TrapezoidProfile.Constraints(1250.0, 1000.0)) {
	private val leftM = CANSparkMax(4, MotorType.kBrushless)
	private val rightM = CANSparkMax(5, MotorType.kBrushless)

	init {
		arrayOf(leftM, rightM).forEach {
			it.restoreFactoryDefaults()
			it.setIdleMode(IdleMode.kBrake)
			it.setSmartCurrentLimit(30)
			it.enableVoltageCompensation(11.0)
		}
		rightM.follow(leftM, true)
	}

	private val elevatorM = leftM
	private val elevatorE = leftM.getEncoder()
	private val elevatorPID = leftM.getPIDController().apply {
		setFeedbackDevice(elevatorE)
		setP(5e-2)
		setI(1e-6)
		setD(1e-4)
		setOutputRange(-1.0, 1.0)
	}

	init { arrayOf(leftM,rightM).forEach(CANSparkMax::burnFlash) }

	init { setDefaultCommand(run({ setGoalRevolutions(elevatorE.getPosition()) })) }

	override fun useState(setpoint: TrapezoidProfile.State) {
		elevatorPID.setReference(setpoint.position / (2.0 * Math.PI), CANSparkMax.ControlType.kPosition)
	}

	fun setGoalRevolutions(pos: Double) {
		setGoal(pos * (2.0 * Math.PI))
	}

	fun goToPos(pos: Double) = run({ setGoalRevolutions(pos) })

	override fun periodic() {
		super.periodic()
		SmartDashboard.putNumber("Elevator Position", elevatorE.getPosition())
	}
}
