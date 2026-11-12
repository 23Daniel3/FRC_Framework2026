# FRC Framework 2026 + Ciprix

[![CI/CD](https://github.com/23Daniel3/FRC_Framework2026/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/23Daniel3/FRC_Framework2026/actions/workflows/ci.yml)

Framework de robôs FRC em Java baseado em **máquinas de estado e requests**. O repositório é
organizado como os grandes frameworks (WPILib, por exemplo): a biblioteca fica separada dos
projetos de robô, e os projetos de robô ficam em pastas de templates e exemplos.

| Pasta | O que é |
| --- | --- |
| [`ciprix/`](./ciprix) | A biblioteca **Ciprix**, dividida em módulos (`core`, `ctre`, `phoenix5`, `rev`, `pathplanner`, `processor`). É o que as equipes instalam como vendordep. |
| [`examples/season-2026-rebuilt/`](./examples/season-2026-rebuilt) | O robô completo da Techmaker (9047) na temporada 2026, usando a Ciprix. É o exemplo de referência do padrão. |
| `templates/` | Projetos mínimos prontos para copiar e começar um robô novo. |
| `docs/` | Guias de arquitetura, simulação, visão e migração. |

Cada template e cada exemplo é um **projeto GradleRIO completo**: abra a pasta do projeto no
VS Code da WPILib (por exemplo `examples/season-2026-rebuilt`), e os comandos de sempre
(`Build Robot Code`, `Simulate Robot Code`, `Deploy Robot Code`) funcionam normalmente.

---

## O padrão (não muda)

1. O piloto fala com `SuperStructureCommands.manageRequests(...)`, que escreve um `RobotRequest`.
2. A máquina de estados geral da `SuperStructure` decide o estado do robô.
3. No `onEnter` de cada estado geral, a `SuperStructure` define o request de cada subsistema.
4. Cada subsistema é um `StateSubsystem<Request, State, Inputs, IO>` com a sua própria máquina
   de estados declarativa (`StateMachine` + `bindRequest(...)`), e só fala com o hardware pela
   camada IO.
5. O comando padrão de cada subsistema é o **relay** gerado por `@AutoCommandFactory`
   (`XxxCommands.defaultCommand(superStructure, xxx)`), que entrega o request da `SuperStructure`.
6. O copiloto é o backup manual: os comandos `XxxCommands.<request>(xxx)` sobrepõem o request
   de um subsistema enquanto o botão está pressionado.

## Como a Ciprix é consumida

Os projetos de robô usam a Ciprix como qualquer biblioteca de fornecedor: pelos arquivos
`vendordeps/Ciprix*.json`. Dentro deste repositório, o `settings.gradle` de cada projeto faz
`includeBuild('../../ciprix')`, então o Gradle troca os artefatos publicados
(`org.ciprix:ciprix-*`) pelo código-fonte local automaticamente. Editou a biblioteca, o robô
já compila com a mudança, sem publicar nada.

Um projeto copiado para fora do repositório não tem a pasta `ciprix/` ao lado, e passa a baixar
a versão publicada indicada nos vendordeps.

| Vendordep | Artefato | Conteúdo |
| --- | --- | --- |
| `Ciprix.json` | `ciprix-core` | FSM, `StateSubsystem`, abstrações de motor e encoder, simulação, logging, utilitários |
| `Ciprix-CTRE.json` | `ciprix-ctre` | TalonFX, TalonFXS, CANcoder, diagnóstico de barramento CAN (Phoenix 6) |
| `Ciprix-Phoenix5.json` | `ciprix-phoenix5` | TalonSRX e VictorSPX (Phoenix 5) |
| `Ciprix-REV.json` | `ciprix-rev` | SPARK MAX, SPARK Flex e encoders REV |
| `Ciprix-PathPlanner.json` | `ciprix-pathplanner` | Integrações com PathPlanner e Choreo |

O processador de anotações (`ciprix-processor`) entra pelo `build.gradle` do projeto:

```groovy
annotationProcessor "org.ciprix:ciprix-processor:$ciprixJson.version"
```

## Comandos úteis

```bash
# Biblioteca: formatação, build e testes de todos os módulos
cd ciprix && ./gradlew build

# Robô de exemplo: build, testes e simulação
cd examples/season-2026-rebuilt && ./gradlew build
cd examples/season-2026-rebuilt && ./gradlew simulateJava

# Gerar os vendordeps com a versão atual (saem em ciprix/build/vendordeps)
cd ciprix && ./gradlew generateVendordeps
```

O CI roda exatamente esses builds para a biblioteca e para cada projeto de robô.
