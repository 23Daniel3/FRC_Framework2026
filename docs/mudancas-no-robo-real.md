# Mudanças no robô real

Esta versão reescreveu a camada de motores, a visão e vários serviços da biblioteca. A lógica do
robô (estados, requests, botões) é a mesma, mas algumas coisas se comportam diferente no hardware.
Leia antes do primeiro deploy e teste com o robô suspenso.

## Motores (todos os fornecedores)

1. **Comandos só quando mudam.** Um comando vai para o controlador quando muda e é reenviado a
   cada 0,5 s. Antes, cada chamada em todo ciclo virava tráfego no CAN.
2. **Motor nunca comandado não é tocado.** Seguidores e motores só lidos ficam como foram
   configurados.
3. **O un-jam devolve o motor.** Depois de inverter, o motor volta ao último comando do código.
   Antes, ele não voltava sozinho.
4. **Saúde real.** "Conectado" agora é o controlador respondendo de verdade, e não "sem falha
   ativa". Motores nunca lidos não são julgados.
5. **Aviso de temperatura.** Acima de 85 °C (padrão) o motor marca `hasProblem` e gera alerta.
6. **Gerenciamento de energia.** Abaixo de 9,5 V o conveyor e o roller do intake (prioridade
   `LOW`) têm o limite de corrente reduzido, e a aceleração do drivetrain fica mais suave.

## CTRE (Phoenix 6)

7. Percent output no Talon FX/FXS é `VoltageOut(percent × tensão nominal)`, então a mesma
   porcentagem dá a mesma força com a bateria cheia ou cansada.
8. Só o sensor do rotor é zerado no boot.
9. Slot 3 não existe no Talon: um pedido de slot 3 vai para o slot 0 com aviso. Motion Magic usa
   os parâmetros de cada slot.
10. Limite de corrente atingido aparece como informação, não como falha.
11. Corrigidos: falhas só eram lidas com o motor desconectado, o seguidor era configurado antes do
    líder, a rampa não era aplicada, a configuração era reescrita a cada chamada, o arranjo do
    Talon FXS era sobrescrito e o `SensorToMechanismRatio` era ignorado.

## CTRE (Phoenix 5) e PWM

12. Talon SRX usa a corrente do estator, aplica o fator de posição, zera no boot e aplica as
    rampas.
13. Percent output de SRX, SPX e PWM é compensado para a tensão nominal.

## REV

14. kS, kA e kG (kCos para braços) vão para o SPARK no boot, não só o kV.
15. Mudanças em tempo de execução não são gravadas na flash a cada alteração.
16. Falhas antigas são limpas no boot.
17. O offset do encoder absoluto é preservado (antes era sobrescrito).
18. Status frames na frequência do loop; SPARK Flex com encoder externo funciona.

## Visão (robô da equipe)

19. **Cada imagem entra no estimador uma vez.** Antes a mesma imagem entrava em vários ciclos
    seguidos, e a pose "grudava" mais forte do que devia.
20. **Novas rejeições:** fora do campo de 2026, giro rápido demais para o MegaTag 2 e salto de tag
    única acima de 1 m enquanto a estimativa é confiável.
21. **Ângulo:** com 2 ou mais tags o ângulo vem do MegaTag 1 (antes, só com exatamente 2); nos
    outros casos a visão não corrige mais o giroscópio.
22. O layout de AprilTags agora é o do campo de 2026 (AndyMark).

## Robô e dashboard

23. `EJECT` (tudo invertendo) no POV direito do piloto; o copiloto inverte só o intake no bumper
    direito.
24. Botões `SelfTest/...` no dashboard (só com o robô habilitado e os mecanismos livres).
25. Checklist da fila: um alerta de aviso lista o que falta enquanto o robô está desabilitado.
26. O pivot do intake simulado se chama `Intake/Intake` (NEO), igual ao hardware.
27. Uma exceção em um serviço da biblioteca é reportada uma vez e não derruba o código do robô.

## Para conferir na primeira ligada

- Sentido de giro e velocidade de cada mecanismo com o robô suspenso.
- Shooter: `kV` do kicker (0,01727 V/RPM satura antes da velocidade máxima) e o PID do flywheel
  (kP 0, kD 0,013).
- Alertas de hardware: nenhum dispositivo deveria aparecer em `Hardware/Disconnected`.
- Visão: `RejectReason` de cada câmera parado no campo (deveria ficar vazio com tags à vista).
