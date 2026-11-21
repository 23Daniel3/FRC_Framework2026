# Visão

A visão tem dois níveis: um pronto para qualquer equipe, na biblioteca, e o da Techmaker, com os
mapas de desvio padrão ajustados pela equipe e regras mais rígidas.

## Nível 1: `AprilTagVision` (biblioteca)

Funciona com Limelight sem precisar do `LimelightHelpers`:

```java
vision =
    new AprilTagVision(
            drivetrain::addVisionMeasurement, drivetrain::getPose, () -> drivetrain.getState().Speeds)
        .withCamera("Front", new VisionIOLimelight("limelight-front", drivetrain::getRotation))
        .withCamera("Back", new VisionIOLimelight("limelight-back", drivetrain::getRotation), 1.5)
        .withFilter(
            new VisionFilter()
                .field(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded))
                .maxDistance(4.0));
```

- `VisionIOLimelight` lê toda imagem nova da fila do NetworkTables, então nenhuma se perde e
  nenhuma é usada duas vezes, e entrega as poses MegaTag 1 e MegaTag 2.
- `VisionFilter` descarta, com motivo: pose fora do campo, robô fora do chão ou inclinado, tags
  longe demais, tag única ambígua (MegaTag 1), giro rápido demais para o MegaTag 2 e, se ligado
  com `maxJump(metros)`, o "salto" de uma tag única para longe da estimativa enquanto ela é
  confiável. Depois de 1 s sem nenhuma pose aceita a estimativa deixa de ser confiável e o robô se
  reencontra.
- `VisionStdDevs` calcula o desvio padrão por distância² dividido pelo número de tags. No MegaTag
  2 o ângulo é o do próprio giroscópio, então o desvio do ângulo é ignorado.
- O número no `withCamera` (1.5 acima) multiplica os desvios de uma câmera menos confiável.

No log, cada câmera tem `Vision/<câmera>/AcceptedPoses`, `RejectedPoses`, `LastRejection`,
`AcceptedCount`, `RejectedCount` e `TagPoses`, e uma câmera que para de responder aparece nos
alertas de hardware. No replay, use `new VisionIO() {}` em cada câmera.

## Nível 2: a visão da Techmaker

`examples/season-2026-rebuilt/.../subsystems/vision/Vision.java` mantém o subsistema e os mapas da
equipe (distância, velocidade, ambiguidade) e soma as regras da biblioteca:

- **Cada imagem é julgada uma vez.** A Limelight é lida como "último valor" a cada ciclo; antes,
  a mesma imagem entrava no estimador em vários ciclos seguidos, e o estimador confiava demais
  nela.
- **Limites do campo de 2026**, robô no chão, giro rápido e salto de tag única acima de
  `MAX_SINGLE_TAG_JUMP` (1 m).
- **Ângulo de verdade só do MegaTag 1 com 2 ou mais tags.** Nos outros casos a pose traz o ângulo
  do giroscópio, e o desvio do ângulo vai para "ignorado": corrigir o giroscópio com ele mesmo só
  injetava o atraso da câmera.
- `Subsystems/Vision/<CÂMERA>/RejectReason` no log diz por que cada imagem ficou de fora.

Para afinar: olhe `RejectReason` e as poses no AdvantageScope depois de um treino. Se uma regra
descarta imagens boas demais, afrouxe a constante dela em `VisionConstants`.

## Campo

`VisionConstants.APRILTAG_LAYOUT` usa o campo AndyMark de 2026, como a equipe usava em 2025.
Troque para `k2026RebuiltWelded` se o evento usar o campo soldado. Os limites do filtro vêm desse
layout.
