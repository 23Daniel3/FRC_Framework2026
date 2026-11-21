# Arquitetura

## O padrão

O robô inteiro é uma conversa de requests entre camadas, cada uma com a sua máquina de estados:

1. **Piloto.** `SuperStructureCommands.manageRequests(...)` é o comando padrão da
   `SuperStructure`: lê os botões a cada ciclo e escreve um `RobotRequest` por uma tabela de
   prioridade (a primeira linha verdadeira vence).
2. **SuperStructure.** A máquina de estados geral (`SuperStructureBase`) decide o estado do robô.
   `bindRequest(request, estadoDeEntrada, estadosQueJaAtendem...)` diz por onde ela entra para
   atender cada request. No `onEnter` de cada estado geral, ela define o request de cada
   subsistema.
3. **Subsistemas.** Cada um é um `StateSubsystem<Request, State, Inputs, IO>` com a sua máquina
   de estados (`fsm.state(...)`, `bindRequest(...)`). `atGoal()` diz se o estado pedido já foi
   alcançado, e a SuperStructure usa isso nas transições.
4. **Relay.** O comando padrão de cada subsistema é gerado pelo `@AutoCommandFactory`
   (`XxxCommands.defaultCommand(superStructure, xxx)`): copia `superStructure.getXxxRequest()`
   para o subsistema todo ciclo.
5. **Copiloto.** O backup manual: `XxxCommands.<request>(xxx)` sobrepõe o request de um
   subsistema enquanto o botão estiver pressionado (`override`, que dura um ciclo e é renovado pelo
   comando). O request da SuperStructure continua guardado e o `atGoal()` continua relativo a ele,
   então a FSM geral nunca avança por causa de um override. Soltou o botão, o subsistema volta a
   seguir a SuperStructure sozinho.
6. **Hardware só pela IO.** O subsistema nunca fala com um motor direto: chama
   `io.controlXxx()`, que devolve um `MotorController`. No replay a IO é vazia e os inputs vêm do
   log, então o mesmo código roda no robô, na simulação e no replay.

## Recursos da máquina de estados

Tudo em `org.ciprix.interfaces.fsm.StateMachine`:

| Recurso | Uso |
| --- | --- |
| `transitionTo(alvo, condição)` | Transição local do estado |
| `transitionTo(alvo, condição, segundos)` | Só dispara se a condição ficou verdadeira por esse tempo (debounce) |
| `waitFor(condição)` / `waitFor(segundos)` | Espera antes de seguir para o próximo estado |
| `timeoutAfter(segundos[, estadoDeSegurança])` | Estado que demora demais vai para o estado de segurança |
| `transitionToSecurityState(condição)` e `addSecurityTransition(condição)` | Caminhos de falha; o request que falhou fica bloqueado até ser solto, para a FSM não voltar para o estado que acabou de falhar |
| `regularTime(min, max)` | Tempo normal do estado, com estatísticas por estado no log |
| `inState(...)` | `Trigger` ativo em um ou mais estados (LEDs, vibração do controle) |
| `toMermaid()` | Diagrama da máquina, também gravado no log uma vez |

## Biblioteca e robô

A Ciprix (`ciprix/`) tem o que serve para qualquer equipe: FSM, `StateSubsystem`, motores com
intenção e saúde, simulação física, visão, energia, autoteste, visualização 3D e utilitários. O
robô (`examples/season-2026-rebuilt/`) tem o que é da Techmaker: os subsistemas, a SuperStructure
desta temporada, as tabelas de tiro, os mapas de desvio padrão da visão e os botões.

Regra prática: se outra equipe usaria igual, vai para a biblioteca; se depende do robô ou do jogo,
fica no projeto do robô.

## Criar um subsistema

O passo a passo, com um exemplo que compila, está no
[esqueleto](../templates/skeleton/README.md#adicionar-um-subsistema).

## Motores

Os motores da Ciprix guardam a **intenção** do código:

- Um comando só vai para o hardware quando muda, e é reenviado a cada 0,5 s. Um estado que repete
  `runVelocity(...)` todo ciclo não enche o barramento CAN.
- Um motor que nunca recebeu comando (um seguidor, por exemplo) nunca é tocado.
- O un-jam automático (`withStallReversal(...)`) inverte o motor por um instante e depois devolve
  o último comando que o código pediu.
- A saúde (conectado, falhas, travado, girando solto, temperatura) vem nos inputs e nos alertas.

O mesmo `MotorConfig` serve para Talon FX, Talon FXS, SPARK MAX/Flex, Talon SRX/Victor SPX, PWM e
para o motor simulado.
