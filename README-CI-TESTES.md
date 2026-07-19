# CI + Testes — instruções de instalação

Este pacote assume o layout padrão GradleRIO, com o código-fonte em `src/main/java/frc/`
(as pastas `lib/`, `robot/`, `game/` do template). Descompacte **na raiz do repositório**:

```
<repo>/
  .github/workflows/ci.yml        <- deste pacote
  src/main/java/frc/...           <- o template (já existente)
  src/test/java/frc/...           <- deste pacote
  build.gradle, gradlew, ...
```

## build.gradle

Os templates GradleRIO recentes já vêm com JUnit 5 configurado. Confira se o seu
`build.gradle` contém (e adicione se não tiver):

```gradle
dependencies {
    // ... deps existentes ...
    testImplementation 'org.junit.jupiter:junit-jupiter:5.10.1'
    testRuntimeOnly 'org.junit.platform:junit-platform-launcher'
}

test {
    useJUnitPlatform()
    systemProperty 'junit.jupiter.extensions.autodetection.enabled', 'true'
}
```

Os testes usam o HAL simulado da WPILib (`HAL.initialize` + `SimHooks`), que o
`wpi.java.configureTestEnvironment` do GradleRIO já habilita por padrão.

## Rodando local

```
./gradlew test           # só os testes
./gradlew build          # compila tudo + testes (o mesmo que o CI roda)
```

## O que está coberto

| Suíte | O que protege |
| --- | --- |
| `StateMachineTest` | Ordem global>local, um salto por ciclo, exclusões do `addRequestTransition`, `transitionAfter` em **segundos** (regressão do bug µs), reset do tempo por estado, `validateComplete`, `forceState` |
| `StateSubsystemTest` | `bindRequest` + `atGoal()` derivado, não-reentrada em estados intermediários, troca de request no meio do caminho — com subsistema fake, sem hardware |
| `SetpointTrackerTest` | Tolerância inclusiva e erro absoluto (usado pelo Shooter) |
| `LinearInterpolationTest` | Segmento correto e clamp nas bordas (mapas de calibração do shooter) |
| `Polygon2dTest` | `contains`/`distanceTo`/centro, incluindo polígono côncavo (zonas do campo) |

## Próximas suítes candidatas

Testáveis com HAL simulado, mas exigem mais montagem: `ShotOnTheMoveCalculator` (robô parado →
mira aponta para o alvo; robô em movimento → compensação no sentido oposto), a FSM geral da
`SuperStructure` com subsistemas fake, e `DynamicSlewRateLimiter` com `SimHooks.stepTiming`.
Não testável em unidade: qualquer coisa que dependa de CTRE/REV reais (IOHardware) — isso é
papel da simulação física e do bench test.
