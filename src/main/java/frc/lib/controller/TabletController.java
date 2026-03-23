package frc.lib.controller;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * A class that interfaces with the tablet controller to handle button inputs. This class uses
 * NetworkTables to retrieve button states from the tablet.
 */
public class TabletController {

  private final Trigger A, B, C, D, E, F, G, H, I, J, K, L;
  private final Trigger L1, L2, L3, L4;
  private final Trigger direita, esquerda;

  private boolean conectado;

  private NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private NetworkTable table = ntInstance.getTable("ButtonCommands");

  /**
   * Constructor for the TabletController class. Initializes all button triggers based on entries in
   * the NetworkTable.
   */
  public TabletController() {
    A = new Trigger(() -> table.getEntry("botaoA").getBoolean(false));
    B = new Trigger(() -> table.getEntry("botaoB").getBoolean(false));
    C = new Trigger(() -> table.getEntry("botaoC").getBoolean(false));
    D = new Trigger(() -> table.getEntry("botaoD").getBoolean(false));
    E = new Trigger(() -> table.getEntry("botaoE").getBoolean(false));
    F = new Trigger(() -> table.getEntry("botaoF").getBoolean(false));
    G = new Trigger(() -> table.getEntry("botaoG").getBoolean(false));
    H = new Trigger(() -> table.getEntry("botaoH").getBoolean(false));
    I = new Trigger(() -> table.getEntry("botaoI").getBoolean(false));
    J = new Trigger(() -> table.getEntry("botaoJ").getBoolean(false));
    K = new Trigger(() -> table.getEntry("botaoK").getBoolean(false));
    L = new Trigger(() -> table.getEntry("botaoL").getBoolean(false));

    L1 = new Trigger(() -> table.getEntry("botaoL1").getBoolean(false));
    L2 = new Trigger(() -> table.getEntry("botaoL2").getBoolean(false));
    L3 = new Trigger(() -> table.getEntry("botaoL3").getBoolean(false));
    L4 = new Trigger(() -> table.getEntry("botaoL4").getBoolean(false));

    direita = new Trigger(() -> table.getEntry("botaoDireita").getBoolean(false));
    esquerda = new Trigger(() -> table.getEntry("botaoEsquerda").getBoolean(false));

    conectado = ntInstance.isConnected();
    Logger.recordOutput("TabletController/isConnected", conectado);
  }

  /**
   * Returns the Trigger associated with button A.
   *
   * @return Trigger for button A
   */
  @AutoLogOutput(key = "TabletController/A")
  public Trigger A() {
    return A;
  }

  /**
   * Returns the Trigger associated with button B.
   *
   * @return Trigger for button B
   */
  @AutoLogOutput(key = "TabletController/B")
  public Trigger B() {
    return B;
  }

  /**
   * Returns the Trigger associated with button C.
   *
   * @return Trigger for button C
   */
  @AutoLogOutput(key = "TabletController/C")
  public Trigger C() {
    return C;
  }

  /**
   * Returns the Trigger associated with button D.
   *
   * @return Trigger for button D
   */
  @AutoLogOutput(key = "TabletController/D")
  public Trigger D() {
    return D;
  }

  /**
   * Returns the Trigger associated with button E.
   *
   * @return Trigger for button E
   */
  @AutoLogOutput(key = "TabletController/E")
  public Trigger E() {
    return E;
  }

  /**
   * Returns the Trigger associated with button F.
   *
   * @return Trigger for button F
   */
  @AutoLogOutput(key = "TabletController/F")
  public Trigger F() {
    return F;
  }

  /**
   * Returns the Trigger associated with button G.
   *
   * @return Trigger for button G
   */
  @AutoLogOutput(key = "TabletController/G")
  public Trigger G() {
    return G;
  }

  /**
   * Returns the Trigger associated with button H.
   *
   * @return Trigger for button H
   */
  @AutoLogOutput(key = "TabletController/H")
  public Trigger H() {
    return H;
  }

  /**
   * Returns the Trigger associated with button I.
   *
   * @return Trigger for button I
   */
  @AutoLogOutput(key = "TabletController/I")
  public Trigger I() {
    return I;
  }

  /**
   * Returns the Trigger associated with button J.
   *
   * @return Trigger for button J
   */
  @AutoLogOutput(key = "TabletController/J")
  public Trigger J() {
    return J;
  }

  /**
   * Returns the Trigger associated with button K.
   *
   * @return Trigger for button K
   */
  @AutoLogOutput(key = "TabletController/K")
  public Trigger K() {
    return K;
  }

  /**
   * Returns the Trigger associated with button L.
   *
   * @return Trigger for button L
   */
  @AutoLogOutput(key = "TabletController/L")
  public Trigger L() {
    return L;
  }

  /**
   * Returns the Trigger associated with button L1.
   *
   * @return Trigger for button L1
   */
  @AutoLogOutput(key = "TabletController/L1")
  public Trigger L1() {
    return L1;
  }

  /**
   * Returns the Trigger associated with button L2.
   *
   * @return Trigger for button L2
   */
  @AutoLogOutput(key = "TabletController/L2")
  public Trigger L2() {
    return L2;
  }

  /**
   * Returns the Trigger associated with button L3.
   *
   * @return Trigger for button L3
   */
  @AutoLogOutput(key = "TabletController/L3")
  public Trigger L3() {
    return L3;
  }

  /**
   * Returns the Trigger associated with button L4.
   *
   * @return Trigger for button L4
   */
  @AutoLogOutput(key = "TabletController/L4")
  public Trigger L4() {
    return L4;
  }

  /**
   * Returns the Trigger associated with the right button.
   *
   * @return Trigger for the right button
   */
  @AutoLogOutput(key = "TabletController/direita")
  public Trigger direita() {
    return direita;
  }

  /**
   * Returns the Trigger associated with the left button.
   *
   * @return Trigger for the left button
   */
  @AutoLogOutput(key = "TabletController/esquerda")
  public Trigger esquerda() {
    return esquerda;
  }
}
