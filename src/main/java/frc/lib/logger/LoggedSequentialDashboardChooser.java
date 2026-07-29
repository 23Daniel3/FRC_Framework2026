package frc.lib.logger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.PeriodicSystem;
import java.util.*;
import java.util.function.Predicate;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Chooser sequencial com N slots e suporte a filtros dinâmicos por slot.
 *
 * <p>- Mantém-se genérico (V pode ser Supplier<Command>, etc). - Publica cada SendableChooser e só
 * o substitui quando o conjunto de opções visíveis muda, para evitar spam de publishers no
 * NetworkTables.
 *
 * <p>Observação: chame periodicamente {@link #periodic()} (ex.: robotPeriodic()).
 *
 * <p>NOTE: foram adicionados métodos públicos para permitir atualizações em cascata por quem
 * coordena múltiplos choosers (ex: AutoTrajectories).
 */
public class LoggedSequentialDashboardChooser<V> extends PeriodicSystem {
  private final String keyPrefix;
  private final int numSlots;
  private final SendableChooser<String>[] choosers;
  private final String[] selectedValues;
  private final Map<String, V> options = new LinkedHashMap<>(); // mantém ordem de inserção
  private String defaultOptionKey = null;

  private final Map<Integer, Predicate<String>> slotFilters = new HashMap<>();
  private final List<Set<String>> visibleOptionsBySlot = new ArrayList<>();

  private final LoggableInputs inputs =
      new LoggableInputs() {
        @Override
        public void toLog(LogTable table) {
          for (int i = 0; i < numSlots; i++) {
            table.put(keyPrefix + "_" + i, selectedValues[i]);
          }
        }

        @Override
        public void fromLog(LogTable table) {
          for (int i = 0; i < numSlots; i++) {
            selectedValues[i] = table.get(keyPrefix + "_" + i, selectedValues[i]);
          }
        }
      };

  @SuppressWarnings("unchecked")
  public LoggedSequentialDashboardChooser(String keyPrefix, int numSlots) {
    if (numSlots <= 0) throw new IllegalArgumentException("numSlots must be > 0");
    this.keyPrefix = Objects.requireNonNull(keyPrefix);
    this.numSlots = numSlots;
    this.choosers = (SendableChooser<String>[]) new SendableChooser[numSlots];
    this.selectedValues = new String[numSlots];

    for (int i = 0; i < numSlots; i++) {
      choosers[i] = new SendableChooser<>();
      visibleOptionsBySlot.add(new LinkedHashSet<>());
      // publica cada chooser UMA vez; atualizações subsequentes ocorrem somente se o conjunto mudar
      SmartDashboard.putData(keyPrefix + "_" + i, choosers[i]);
    }
  }

  public void addOption(String optionKey, V value) {
    if (optionKey == null) throw new IllegalArgumentException("optionKey cannot be null");
    options.put(optionKey, value);
    // periodic() vai recalcular e publicar se necessário
  }

  public void addDefaultOption(String optionKey, V value) {
    addOption(optionKey, value);
    defaultOptionKey = optionKey;
  }

  public void setFilter(int slot, Predicate<String> filter) {
    checkSlot(slot);
    if (filter == null) slotFilters.remove(slot);
    else slotFilters.put(slot, filter);
    // periodic() aplicará o filtro e publicará se necessário
  }

  public void clearFilter(int slot) {
    checkSlot(slot);
    slotFilters.remove(slot);
  }

  public int getNumSlots() {
    return numSlots;
  }

  public String getSelectedKey(int slot) {
    checkSlot(slot);
    return selectedValues[slot];
  }

  public List<V> get() {
    List<V> result = new ArrayList<>();
    for (int i = 0; i < numSlots; i++) result.add(options.get(selectedValues[i]));
    return result;
  }

  public V get(int slot) {
    checkSlot(slot);
    return options.get(selectedValues[slot]);
  }

  public SendableChooser<String>[] getSendableChoosers() {
    return choosers;
  }

  /**
   * Deve ser chamado periodicamente (ex.: robotPeriodic()). Ordem: 1) lê as seleções atuais
   * (chooser.getSelected()) 2) recalcula visíveis e substitui o SendableChooser do slot apenas se o
   * conjunto visível mudou 3) processa logger
   *
   * <p>Observação: este método mantém o comportamento histórico. Para coordenação entre vários
   * choosers (cascade), usar os métodos públicos {@link #readSelectionsFromDashboard()}, {@link
   * #refreshSlot(int)}, {@link #refreshAllSlots()} e {@link #processInputs()}.
   */
  @Override
  public void periodic() {
    // 1) lê seleções atuais do dashboard
    if (!Logger.hasReplaySource()) {
      for (int i = 0; i < numSlots; i++) {
        try {
          selectedValues[i] = choosers[i].getSelected();
        } catch (Exception e) {
          // mantém valor anterior em caso de erro
        }
      }
    }

    // 2) atualiza visíveis/publicações SOMENTE se mudou
    for (int i = 0; i < numSlots; i++) {
      refreshSlotIfNeeded(i);
    }

    // 3) logger
    Logger.processInputs(keyPrefix, inputs);
  }

  /* ---------- novos helpers públicos para coordenação entre choosers ---------- */

  /**
   * Lê as seleções atuais do Dashboard em selectedValues[] sem mudar visíveis/publicações. Útil
   * para coletar o estado antes de executar refreshes em cadeia.
   */
  public void readSelectionsFromDashboard() {
    if (!Logger.hasReplaySource()) {
      for (int i = 0; i < numSlots; i++) {
        try {
          selectedValues[i] = choosers[i].getSelected();
        } catch (Exception e) {
          // mantém valor anterior em caso de erro
        }
      }
    }
  }

  /** Atualiza somente o slot especificado (republica se o conjunto visível mudou). */
  public void refreshSlot(int slot) {
    checkSlot(slot);
    refreshSlotIfNeeded(slot);
  }

  /** Atualiza todos os slots (equivalente ao passo 2 da periodic). */
  public void refreshAllSlots() {
    for (int i = 0; i < numSlots; i++) {
      refreshSlotIfNeeded(i);
    }
  }

  /** Executa o processamento do logger (equivalente ao passo 3 da periodic). */
  public void processInputs() {
    Logger.processInputs(keyPrefix, inputs);
  }

  /* ---------- helpers privados ---------- */

  private void refreshSlotIfNeeded(int slot) {
    checkSlot(slot);
    Set<String> newVisible = computeVisibleOptions(slot);
    Set<String> oldVisible = visibleOptionsBySlot.get(slot);

    if (Objects.equals(oldVisible, newVisible)) {
      return; // sem mudança -> evita re-publicação
    }

    // constrói um novo chooser contendo somente as opções visíveis
    SendableChooser<String> newChooser = new SendableChooser<>();
    boolean defaultSet = false;

    if (newVisible.isEmpty()) {
      if (defaultOptionKey != null && options.containsKey(defaultOptionKey)) {
        newChooser.setDefaultOption(defaultOptionKey, defaultOptionKey);
        defaultSet = true;
      }
    } else {
      for (String k : newVisible) {
        if (!defaultSet && defaultOptionKey != null && defaultOptionKey.equals(k)) {
          newChooser.setDefaultOption(k, k);
          defaultSet = true;
        } else {
          newChooser.addOption(k, k);
        }
      }
    }

    // substitui o chooser publicado (publicação só acontece quando houve mudança)
    choosers[slot] = newChooser;
    SmartDashboard.putData(keyPrefix + "_" + slot, newChooser);

    // tenta preservar a seleção anterior se ainda for válida (atualiza selectedValues)
    String prev = selectedValues[slot];
    if (prev != null && newVisible.contains(prev)) {
      // mantem prev
    } else {
      // se prev não for válido, coloca defaultOptionKey (se houver) ou null
      if (defaultOptionKey != null && newVisible.contains(defaultOptionKey)) {
        selectedValues[slot] = defaultOptionKey;
      } else {
        selectedValues[slot] = null;
      }
    }

    visibleOptionsBySlot.set(slot, newVisible);
  }

  private Set<String> computeVisibleOptions(int slot) {
    Predicate<String> filter = slotFilters.get(slot);
    Set<String> out = new LinkedHashSet<>();
    for (String k : options.keySet()) {
      if (filter == null || filter.test(k)) out.add(k);
    }
    return out;
  }

  private void checkSlot(int slot) {
    if (slot < 0 || slot >= numSlots)
      throw new IndexOutOfBoundsException("slot out of range: " + slot);
  }
}
