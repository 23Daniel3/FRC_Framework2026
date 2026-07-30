package frc.lib.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.logger.LoggedSequentialDashboardChooser;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

public class AutoEngine {
  private final LoggedSequentialDashboardChooser<Supplier<Command>> chooser;

  public AutoEngine(String name, int slots) {
    this.chooser = new LoggedSequentialDashboardChooser<>(name, slots);
  }

  public void addPart(String name, Supplier<Command> commandSupplier) {
    Objects.requireNonNull(commandSupplier, "Command supplier cannot be null");
    chooser.addOption(name, commandSupplier);
  }

  public void addDefaultPart(String name, Supplier<Command> commandSupplier) {
    Objects.requireNonNull(commandSupplier, "Command supplier cannot be null");
    chooser.addDefaultOption(name, commandSupplier);
  }

  public LoggedSequentialDashboardChooser<Supplier<Command>> getChooser() {
    return chooser;
  }

  public Command build() {
    return Commands.defer(
        () -> {
          List<Supplier<Command>> selectedSuppliers = chooser.get();

          if (selectedSuppliers == null || selectedSuppliers.isEmpty()) {
            return Commands.none();
          }

          // Prevents array reallocation overhead during iteration
          List<Command> commands = new ArrayList<>(selectedSuppliers.size());

          for (Supplier<Command> supplier : selectedSuppliers) {
            if (supplier != null) {
              Command cmd = supplier.get();
              if (cmd != null) {
                commands.add(cmd);
              }
            }
          }
          return Commands.sequence(commands.toArray(new Command[0]));
        },
        java.util.Set.of());
  }
}
