# Robô 3D no AdvantageScope

Esta pasta guarda o robô customizado do AdvantageScope: o CAD da equipe com as peças móveis
seguindo os mecanismos de verdade. Funciona igual na simulação, no robô e no replay de um log,
porque as poses vêm dos inputs dos subsistemas.

```
advantagescope/
  Robot_Techmaker2026/
    config.json          configuração do robô (feita à mão, versionada)
    model.glb            o chassi e tudo que não se mexe (exportado do CAD)
    model_0.glb          o intake (peça móvel 0)
    config.generated.json  gerado pela simulação, não versionado
```

## 1. Exportar o CAD

1. Deixe a montagem do robô na **posição inicial** dos mecanismos, a mesma em que o código
   considera ângulo ou altura zero (no intake, `INTAKE_START_POSITION`).
2. A origem da montagem deve ser o **centro do chassi, no chão**. Se não for, ajuste `position`
   no `config.json` e some o mesmo valor em cada `zeroedPosition`.
3. Exporte o chassi sem as peças móveis como `model.glb` (glTF binário).
4. Exporte cada peça móvel em um arquivo separado, `model_0.glb`, `model_1.glb` e assim por diante,
   **na mesma posição em que ela está na montagem** (não mova a origem). No Onshape, um jeito é
   suprimir as outras peças antes de exportar.

O guia oficial de conversão está em
[Converting Onshape & STEP Files to glTF](https://docs.advantagescope.org/more-features/custom-assets/gltf-convert)
e o formato do `config.json` em
[Custom Assets](https://docs.advantagescope.org/more-features/custom-assets/).

Arquivos `.glb` grandes deixam o AdvantageScope lento. Se passarem de uns 50 MB, simplifique o
modelo (sem parafusos, sem furos pequenos) e pense em não versionar os `.glb` (ou usar Git LFS).

## 2. Registrar a peça no código

Cada peça móvel é registrada uma vez, em `RobotContainer.configureVisualizer()`, com o número do
seu modelo e o ponto em volta do qual ela gira:

```java
// model_0.glb: o intake, girando em volta do eixo do pivot.
RobotVisualizer.component(
    0,
    MechanismPoses.pivot(IntakeConstants.PIVOT_AXIS, Axis.PITCH, intake::getPivotAngle),
    IntakeConstants.PIVOT_AXIS);
```

- `PIVOT_AXIS`: um ponto do eixo de rotação, medido no CAD em metros (X para frente, Y para a
  esquerda, Z para cima, a partir do centro do chassi no chão).
- `Axis.PITCH` com ângulo positivo levanta a frente; `Axis.YAW` gira no sentido anti-horário
  visto de cima (torretas).
- Para elevadores use `MechanismPoses.slider(inicio, direcao, subsistema::getAltura)`, e para uma
  peça montada em outra (braço no carro do elevador) use `MechanismPoses.attach(pai, filho)`.

Os valores de `PIVOT_AXIS` e `PIVOT_DEPLOY_DEGREES` em `IntakeConstants` são provisórios: meça no
CAD e troque.

## 3. Gerar o `config.json`

Rode a simulação uma vez (`./gradlew simulateJava`). Ela escreve
`Robot_Techmaker2026/config.generated.json` com um componente por peça registrada, já com o
`zeroedPosition` que leva o eixo de cada peça para a origem (é assim que o AdvantageScope espera
uma peça "zerada"). Copie a lista `components` para o `config.json`.

As rotações padrão (`x 90°`, depois `z 90°`) servem para modelos exportados do Onshape. Se o
robô aparecer deitado ou virado, ajuste `rotations` e `zeroedRotations` juntos.

## 4. Ver no AdvantageScope

1. No AdvantageScope, use **App > Use Custom Assets Folder** (no macOS o menu se chama
   **AdvantageScope**) e escolha esta pasta `advantagescope/`, a que contém `Robot_Techmaker2026`.
   Assim o modelo atualiza junto com o `git pull`.
2. Abra uma aba **3D Field**, arraste a pose do robô
   (`RealOutputs/Subsystems/Drivetrain/Odometry/Pose`) e escolha o modelo **Techmaker 2026**.
3. Arraste `RealOutputs/Visualizer/Components` para cima do robô e escolha **Components**.

Se uma peça girar em volta do ponto errado, o `PIVOT_AXIS` não bate com o CAD. Se girar para o
lado errado, troque o sinal de `PIVOT_DEPLOY_DEGREES`.
