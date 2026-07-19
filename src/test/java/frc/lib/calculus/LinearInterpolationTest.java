package frc.lib.calculus;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.lib.calculus.LinearInterpolation.Point;
import org.junit.jupiter.api.Test;

/**
 * A interpolacao linear e a base dos mapas de calibracao do shooter (distancia -> RPM/angulo);
 * errar o clamp ou o segmento significa atirar errado em toda a faixa.
 */
class LinearInterpolationTest {

  private final LinearInterpolation map =
      new LinearInterpolation(new Point(1.0, 100.0), new Point(2.0, 200.0), new Point(4.0, 300.0));

  @Test
  void returnsExactValuesAtKnownPoints() {
    assertEquals(100.0, map.calculate(1.0), 1e-9);
    assertEquals(200.0, map.calculate(2.0), 1e-9);
    assertEquals(300.0, map.calculate(4.0), 1e-9);
  }

  @Test
  void interpolatesWithinSegments() {
    assertEquals(150.0, map.calculate(1.5), 1e-9);
    // segmento 2..4 tem inclinacao diferente (50/unidade): pega o segmento certo
    assertEquals(250.0, map.calculate(3.0), 1e-9);
  }

  @Test
  void clampsOutsideRange() {
    assertEquals(100.0, map.calculate(0.0), 1e-9, "abaixo do primeiro ponto: clamp no y inicial");
    assertEquals(300.0, map.calculate(10.0), 1e-9, "acima do ultimo ponto: clamp no y final");
  }
}
