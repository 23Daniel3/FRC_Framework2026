# Arquivo morto — referência histórica

Código removido do template, preservado **fora do build** (extensão `.java.txt` para o Gradle
não compilar). Serve como referência de padrões de temporadas passadas; a história completa,
com contexto e diffs, está no git (uma tag por temporada, ex.: `v2026-final`).

Regra: nada aqui volta para o build por copy-paste direto — os arquivos referenciam APIs que já
mudaram. Extraia o *padrão* e reescreva sobre a base atual.

## 2025-2026

| Arquivo | Por que saiu | O que usar no lugar |
| --- | --- | --- |
| `AlignToReef.java.txt` + `AlignToReefConstants.java.txt` | Jogo 2025 (Reefscape); alinhamento a pose fixa com 3 PIDs | `MoveFieldRelative` / `MoveWithAutopilot` + `TunableControls` cobrem o padrão de forma genérica |
| `ConveyorCommands.java.txt`, `IntakeCommands.java.txt`, `ShooterCommands.java.txt` | Escreviam requests de subsistema diretamente; no Modelo A eram sobrescritas a cada ciclo pelo relay (inutilizáveis e enganosas) | A SuperStructure escreve requests nos `onEnter`; intenção externa só via `SuperStructureCommands` |
| `AutoTrajetorys.java.txt` | Autos L/R duplicados, `.until(() -> false)`, passo final inalcançável, typo no nome | `AutoTrajectories` parametrizada (`shootCollectShoot(pathName)`, `aimAndShoot(min)`) |
| `MoveWithAutopilotAimHub.java.txt` | Era um `.bkp` solto no repositório | Composição `driveCore` + `RotationStrategy` (ver `DrivetrainCommands.joystickDriveAimHub`) |
