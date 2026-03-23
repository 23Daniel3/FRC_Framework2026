package frc.lib.logger;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import frc.robot.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Tunable number compatível com as Unidades da WPILib.
 * O valor no Dashboard é mantido na unidade definida no construtor.
 */
@SuppressWarnings({"rawtypes", "unchecked"})
public class LoggedTunableUnit {
  private static final String tableKey = "/Tuning";
  private static final double EPSILON = 1e-6;

  private final String key;
  private final Unit unit; 
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * @param dashboardKey Nome no Dashboard.
   * @param defaultValue Medida inicial (ex: Amps.of(40))
   */
  public LoggedTunableUnit(String dashboardKey, Measure defaultValue) {
    this.unit = defaultValue.unit();
    // Monta a chave com o símbolo: "Subsystems/Flywheel/CurrentLimit (A)"
    this.key = tableKey + "/" + dashboardKey + " (" + unit.symbol() + ")";
    initDefault(defaultValue);
  }

  public void initDefault(Measure defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue.in(unit);
      if (Constants.tuningMode) {
        dashboardNumber = new LoggedNetworkNumber(key, this.defaultValue);
      }
    }
  }

  /**
   * Retorna o valor numérico puro na unidade original.
   */
  public double getRaw() {
    if (!hasDefault) return 0.0;
    return Constants.tuningMode ? dashboardNumber.get() : defaultValue;
  }

  /**
   * Retorna a Measure já "convertida" para o tipo correto (Current, Voltage, Velocity, etc).
   * O Java infere o tipo T automaticamente com base no método que está chamando este get().
   */
  public <T extends Measure> T get() {
    return (T) unit.of(getRaw());
  }

  public boolean hasChanged(int id) {
    double currentValue = getRaw();
    Double lastValue = lastHasChangedValues.get(id);

    if (lastValue == null || Math.abs(currentValue - lastValue) > EPSILON) {
      lastHasChangedValues.put(id, currentValue);
      return lastValue != null;
    }
    return false;
  }

  public static void ifChanged(int id, Runnable action, LoggedTunableUnit... numbers) {
    if (Arrays.stream(numbers).anyMatch(n -> n.hasChanged(id))) {
      action.run();
    }
  }
}