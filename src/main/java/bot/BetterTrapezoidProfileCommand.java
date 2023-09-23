// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bot;

import static bot.BetterTrapezoidProfile.State;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A command that runs a {@link BetterTrapezoidProfile}. Useful for smoothly controlling mechanism motion.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class BetterTrapezoidProfileCommand extends CommandBase {
  private final BetterTrapezoidProfile m_profile;
  private final Consumer<State> m_output;
  private final Supplier<State> m_goal;
  private final Supplier<State> m_currentState;
  private final boolean m_newAPI; // TODO: Remove
  private final Timer m_timer = new Timer();

  /**
   * Creates a new TrapezoidProfileCommand that will execute the given {@link BetterTrapezoidProfile}.
   * Output will be piped to the provided consumer function.
   *
   * @param profile The motion profile to execute.
   * @param output The consumer for the profile output.
   * @param goal The supplier for the desired state
   * @param currentState The current state
   * @param requirements The subsystems required by this command.
   */
  public BetterTrapezoidProfileCommand(
      BetterTrapezoidProfile profile,
      Consumer<State> output,
      Supplier<State> goal,
      Supplier<State> currentState,
      Subsystem... requirements) {
    m_profile = requireNonNullParam(profile, "profile", "TrapezoidProfileCommand");
    m_output = requireNonNullParam(output, "output", "TrapezoidProfileCommand");
    m_goal = goal;
    m_currentState = currentState;
    m_newAPI = true;
    addRequirements(requirements);
  }


  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  @SuppressWarnings("removal")
  public void execute() {
    if (m_newAPI) {
      m_output.accept(m_profile.calculate(m_timer.get(), m_goal.get(), m_currentState.get()));
    } else {
      m_output.accept(m_profile.calculate(m_timer.get()));
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_profile.totalTime());
  }
}
