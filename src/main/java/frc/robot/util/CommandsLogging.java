// Copyright (c) 2026 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

/**
 * A utility that heavily abuses the {@link edu.wpi.first.wpilibj.Alert}s system
 * to allow for
 * visualization of what {@link Command}s are running at a time, and what
 * {@link Subsystem}s those
 * commands require.
 *
 * <p>
 * Originally by Harry from 1683. Heavily modified to support things like
 * distinguishing between
 * default/non-default commands and recursively logging subcommands.
 */
public class CommandsLogging {
  private static final Set<Command> runningNonInterrupters = new HashSet<>();
  public static final Map<Command, Command> runningInterrupters = new HashMap<>();
  private static final Map<Subsystem, Command> requiredSubsystems = new HashMap<>();

  public static void commandStarted(final Command command) {
    if (!runningInterrupters.containsKey(command)) {
      runningNonInterrupters.add(command);
    }

    for (final Subsystem subsystem : command.getRequirements()) {
      requiredSubsystems.put(subsystem, command);
    }
  }

  public static void commandEnded(final Command command) {
    runningNonInterrupters.remove(command);
    runningInterrupters.remove(command);

    for (final Subsystem subsystem : command.getRequirements()) {
      requiredSubsystems.remove(subsystem);
    }
  }

  private static final Field wrapperCommandField;
  private static final Field sequenceSubCommandsField;
  private static final Field sequenceCommandIndexField;
  private static final Field parallelSubCommandsField;
  private static final Field deadlineSubCommandsField;
  private static final Field deadlineCommandField;
  private static final Field raceSubCommandsField;
  private static final Field conditionalOnTrueField;
  private static final Field conditionalOnFalseField;
  private static final Field conditionalSelectedField;
  private static final Field selectSubCommandsField;
  private static final Field selectSelectedField;
  private static final Field repeatSubCommandField;

  static {
    // Grab all the Fields we're gonna use ahead of time
    try {
      wrapperCommandField = WrapperCommand.class.getDeclaredField("m_command");
      wrapperCommandField.setAccessible(true);

      sequenceSubCommandsField = SequentialCommandGroup.class.getDeclaredField("m_commands");
      sequenceSubCommandsField.setAccessible(true);
      sequenceCommandIndexField = SequentialCommandGroup.class.getDeclaredField("m_currentCommandIndex");
      sequenceCommandIndexField.setAccessible(true);

      parallelSubCommandsField = ParallelCommandGroup.class.getDeclaredField("m_commands");
      parallelSubCommandsField.setAccessible(true);

      deadlineSubCommandsField = ParallelDeadlineGroup.class.getDeclaredField("m_commands");
      deadlineSubCommandsField.setAccessible(true);
      deadlineCommandField = ParallelDeadlineGroup.class.getDeclaredField("m_deadline");
      deadlineCommandField.setAccessible(true);

      raceSubCommandsField = ParallelRaceGroup.class.getDeclaredField("m_commands");
      raceSubCommandsField.setAccessible(true);

      conditionalOnTrueField = ConditionalCommand.class.getDeclaredField("m_onTrue");
      conditionalOnTrueField.setAccessible(true);
      conditionalOnFalseField = ConditionalCommand.class.getDeclaredField("m_onFalse");
      conditionalOnFalseField.setAccessible(true);
      conditionalSelectedField = ConditionalCommand.class.getDeclaredField("m_selectedCommand");
      conditionalSelectedField.setAccessible(true);

      selectSubCommandsField = SelectCommand.class.getDeclaredField("m_commands");
      selectSubCommandsField.setAccessible(true);
      selectSelectedField = SelectCommand.class.getDeclaredField("m_selectedCommand");
      selectSelectedField.setAccessible(true);

      repeatSubCommandField = RepeatCommand.class.getDeclaredField("m_command");
      repeatSubCommandField.setAccessible(true);
    } catch (NoSuchFieldException e) {
      throw new RuntimeException(e);
    }
  }

  private static final HashMap<Command, String> nameCache = new HashMap<>();

  public static String getCommandName(Command command) {
    // If the name was already computed, grab that cached name
    if (nameCache.containsKey(command)) {
      return nameCache.get(command);
    }

    final StringBuilder subCommandsNameBuilder = new StringBuilder();

    if (command instanceof WrapperCommand wrapperCommand) {
      try {
        var wrappedCommand = (Command) wrapperCommandField.get(wrapperCommand);

        // If the wrapping doesn't modify getName(), then get the wrapped command's name
        // If it does modify getName(), then the else statement in the if-else chain
        // covers that
        if (wrappedCommand.getName().equals(wrapperCommand.getName())) {
          return getCommandName(wrappedCommand);
        }
      } catch (IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    }

    // Pain.
    // For all cases in this giant if-else chain, build a String representing the
    // type and
    // subcommands of each command group.
    // Recurse as needed for nested command groups.
    // If the name of the command is modified, the command type is a WrapperCommand
    // and thus
    // hits the else statement
    String finalName;
    if (command instanceof SequentialCommandGroup sequence) {
      try {
        // noinspection unchecked
        var subCommands = (List<Command>) sequenceSubCommandsField.get(sequence);

        subCommandsNameBuilder.append("sequence(");
        int j = 1;
        for (final var subCommand : subCommands) {
          subCommandsNameBuilder.append(getCommandName(subCommand));
          if (j < subCommands.size()) {
            subCommandsNameBuilder.append(", ");
          }

          j++;
        }
        subCommandsNameBuilder.append(")");
        finalName = subCommandsNameBuilder.toString();
      } catch (IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    } else if (command instanceof ParallelCommandGroup parallel) {
      try {
        // noinspection unchecked
        var subCommands = (Map<Command, Boolean>) parallelSubCommandsField.get(parallel);

        subCommandsNameBuilder.append("parallel(");
        int j = 1;
        for (final var subCommand : subCommands.keySet()) {
          subCommandsNameBuilder.append(getCommandName(subCommand));
          if (j < subCommands.size()) {
            subCommandsNameBuilder.append(", ");
          }

          j++;
        }
        subCommandsNameBuilder.append(")");
        finalName = subCommandsNameBuilder.toString();
      } catch (IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    } else if (command instanceof ParallelDeadlineGroup deadlineGroup) {
      try {
        // noinspection unchecked
        var subCommands = ((Map<Command, Boolean>) deadlineSubCommandsField.get(deadlineGroup)).keySet();

        var deadline = (Command) deadlineCommandField.get(deadlineGroup);
        subCommandsNameBuilder.append(getCommandName(deadline));

        subCommandsNameBuilder.append(".deadlineFor(");
        int j = 1;
        for (final var subCommand : subCommands) {
          if (deadline.equals(subCommand)) {
            continue;
          }
          subCommandsNameBuilder.append(getCommandName(subCommand));
          if (j < subCommands.size()) {
            subCommandsNameBuilder.append(", ");
          }

          j++;
        }
        subCommandsNameBuilder.append(")");
        finalName = subCommandsNameBuilder.toString();

      } catch (IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    } else if (command instanceof ParallelRaceGroup race) {
      try {
        // noinspection unchecked
        var subCommands = (Set<Command>) raceSubCommandsField.get(race);

        subCommandsNameBuilder.append("race(");
        int j = 1;
        for (final var subCommand : subCommands) {
          subCommandsNameBuilder.append(getCommandName(subCommand));
          if (j < subCommands.size()) {
            subCommandsNameBuilder.append(", ");
          }

          j++;
        }
        subCommandsNameBuilder.append(")");
        finalName = subCommandsNameBuilder.toString();
      } catch (IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    } else if (command instanceof ConditionalCommand conditional) {
      try {
        var onTrue = (Command) conditionalOnTrueField.get(conditional);
        var onFalse = (Command) conditionalOnFalseField.get(conditional);

        subCommandsNameBuilder.append("either(");
        subCommandsNameBuilder.append(getCommandName(onTrue));
        subCommandsNameBuilder.append(", ");
        subCommandsNameBuilder.append(getCommandName(onFalse));
        subCommandsNameBuilder.append(")");
        finalName = subCommandsNameBuilder.toString();
      } catch (IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    } else if (command instanceof SelectCommand<?> select) {
      try {
        // noinspection unchecked
        var subCommands = ((Map<?, Command>) selectSubCommandsField.get(select)).values();

        subCommandsNameBuilder.append("select(");
        int j = 1;
        for (final var subCommand : subCommands) {
          subCommandsNameBuilder.append(getCommandName(subCommand));
          if (j < subCommands.size()) {
            subCommandsNameBuilder.append(", ");
          }

          j++;
        }
        subCommandsNameBuilder.append(")");
        finalName = subCommandsNameBuilder.toString();
      } catch (IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    } else if (command instanceof RepeatCommand repeat) {
      try {
        var subCommand = (Command) repeatSubCommandField.get(repeat);

        finalName = "repeat(" + getCommandName(subCommand) + ")";
      } catch (IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    } else {
      finalName = command.getName();
    }

    final StringBuilder subsystemsBuilder = new StringBuilder();
    int i = 1;
    for (final var subsystem : command.getRequirements()) {
      subsystemsBuilder.append(subsystem.getName());
      if (i < command.getRequirements().size()) {
        subsystemsBuilder.append(", ");
      }

      i++;
    }

    if (i > 1) {
      finalName += " (" + subsystemsBuilder + ")";
    }

    nameCache.put(command, finalName);
    return finalName;
  }

  public static void logRunningCommands() {
    Logger.recordOutput("CommandScheduler/Running/.type", "Alerts");

    final ArrayList<String> runningCommands = new ArrayList<>();
    final ArrayList<String> runningDefaultCommands = new ArrayList<>();
    for (final Command command : runningNonInterrupters) {
      ArrayList<String> commandsList = null;
      boolean isDefaultCommand = false;
      for (Subsystem subsystem : command.getRequirements()) {
        if (subsystem.getDefaultCommand() == command) {
          commandsList = runningDefaultCommands;
          isDefaultCommand = true;
          break;
        }
      }
      if (!isDefaultCommand) {
        commandsList = runningCommands;
      }
      addCommand(command, getCommandName(command), commandsList);
    }
    Logger.recordOutput(
        "CommandScheduler/Running/warnings", runningCommands.toArray(new String[0]));
    Logger.recordOutput(
        "CommandScheduler/Running/infos", runningDefaultCommands.toArray(new String[0]));

    final String[] interrupters = new String[runningInterrupters.size()];
    int j = 0;
    for (final Map.Entry<Command, Command> entry : runningInterrupters.entrySet()) {
      final Command interrupter = entry.getKey();
      final Command interrupted = entry.getValue();

      interrupters[j] = getCommandName(interrupter) + " interrupted " + getCommandName(interrupted);
      j++;
    }

    Logger.recordOutput("CommandScheduler/Running/errors", interrupters);
  }

  private static void addCommand(
      Command command, String parentCommandName, ArrayList<String> commandsList) {
    if (command instanceof WrapperCommand wrapperCommand) {
      // Grab the command being wrapped by the WrapperCommand
      try {
        addCommand(
            (Command) wrapperCommandField.get(wrapperCommand), parentCommandName, commandsList);
        return; // Don't need to log the name of the parent
      } catch (IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    }

    // Log the parent too
    commandsList.add(parentCommandName);
    // For all cases in this giant if-else chain:
    // Recursively grab all subcommands till there's no subcommand, then log
    // that command's name.
    if (command instanceof SequentialCommandGroup sequence) {
      try {
        // noinspection unchecked
        var subCommands = (List<Command>) sequenceSubCommandsField.get(sequence);
        var index = (Integer) sequenceCommandIndexField.get(sequence);

        // Handle case where index is -1 (before sequence starts) or out of bounds
        if (index < 0 || index >= subCommands.size()) {
          return;
        }
        var subCommand = subCommands.get(index);
        addCommand(subCommand, parentCommandName + ": " + getCommandName(subCommand), commandsList);
      } catch (IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    } else if (command instanceof ParallelCommandGroup parallel) {
      try {
        // noinspection unchecked
        var subCommands = ((Map<Command, Boolean>) parallelSubCommandsField.get(parallel)).keySet();

        for (var subCommand : subCommands) {
          if (!subCommand.isFinished()) {
            addCommand(
                subCommand, parentCommandName + ": " + getCommandName(subCommand), commandsList);
          }
        }
      } catch (IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    } else if (command instanceof ParallelDeadlineGroup deadlineGroup) {
      try {
        var deadline = (Command) deadlineCommandField.get(deadlineGroup);
        // noinspection unchecked
        var subCommands = ((Map<Command, Boolean>) deadlineSubCommandsField.get(deadlineGroup)).keySet();

        addCommand(deadline, parentCommandName + ": " + getCommandName(deadline), commandsList);
        for (var subCommand : subCommands) {
          if (!subCommand.isFinished() && !deadline.equals(subCommand)) {
            addCommand(
                subCommand, parentCommandName + ": " + getCommandName(subCommand), commandsList);
          }
        }
      } catch (IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    } else if (command instanceof ParallelRaceGroup race) {
      try {
        // noinspection unchecked
        var subCommands = (Set<Command>) raceSubCommandsField.get(race);

        for (var subCommand : subCommands) {
          addCommand(
              subCommand, parentCommandName + ": " + getCommandName(subCommand), commandsList);
        }
      } catch (IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    } else if (command instanceof ConditionalCommand conditional) {
      try {
        var selected = (Command) conditionalSelectedField.get(conditional);

        addCommand(selected, parentCommandName + ": " + getCommandName(selected), commandsList);
      } catch (IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    } else if (command instanceof SelectCommand<?> select) {
      try {
        var selected = (Command) selectSelectedField.get(select);

        addCommand(selected, parentCommandName + ": " + getCommandName(selected), commandsList);
      } catch (IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    } else if (command instanceof RepeatCommand repeat) {
      try {
        var subCommand = (Command) repeatSubCommandField.get(repeat);

        addCommand(subCommand, parentCommandName + ": " + getCommandName(subCommand), commandsList);
      } catch (IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    }
  }

  public static void logRequiredSubsystems() {
    Logger.recordOutput("CommandScheduler/Subsystems/.type", "Alerts");

    final String[] subsystems = new String[requiredSubsystems.size()];
    {
      int i = 0;
      for (final Map.Entry<Subsystem, Command> entry : requiredSubsystems.entrySet()) {
        final Subsystem required = entry.getKey();
        final Command command = entry.getValue();

        subsystems[i] = required.getName() + " (" + command.getName() + ")";
        i++;
      }
    }
    Logger.recordOutput("CommandScheduler/Subsystems/infos", subsystems);
  }
}
