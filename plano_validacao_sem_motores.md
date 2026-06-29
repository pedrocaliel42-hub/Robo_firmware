# Plano de Validacao Sem Motores - ROBO_6DOF

> Objetivo: validar o firmware atual sem motores, sem A4988 energizado e sem o robo mecanico montado.
> A posicao `POS` nesta fase e estimada por software: ela confirma comandos aceitos e passos enviados, nao confirma movimento fisico real.

## 1. Preparacao

- Usar a raiz do projeto na extensao ESP-IDF: `C:\Users\pedro\Robo_firmware`.
- Fazer `Build`, `Flash` e abrir o monitor serial.
- Manter A4988 e motores desconectados no primeiro teste.
- Se for medir sinais, usar LED com resistor ou analisador logico nos GPIOs STEP/DIR.

## 2. Boot e contrato serial

Comandos:

```text
PING
POS?
```

Esperado:

```text
OK_INIT_ROBOT
OK_PONG
POS,0.00,0.00,90.00,0.00,0.00,0.00,0,0.0,0.0,0.0
```

Aceite:

- O ESP32 nao reinicia.
- `PING` responde rapido.
- `POS?` retorna q1..q6, garra e XYZ no formato esperado pela IHM.

## 3. START e streaming POS

Comandos:

```text
START
POSOFF
POSON
POSOFF
```

Esperado:

```text
OK_START
OK_POS_OFF
OK_POS_ON
OK_POS_OFF
```

Aceite:

- `START` arma o robo.
- `POSON` liga telemetria continua.
- `POSOFF` desliga telemetria para facilitar digitar comandos no monitor serial.

## 4. Limites e formato

Comandos:

```text
ANG,999,0,90,0,0,0,50
ANG,0,0,90,0,0,0
MOV,0,0,0,0,0,0
```

Esperado:

```text
ERR_LIMIT
ERR_BAD_FORMAT
ERR_UNSUPPORTED_MOV
```

Aceite:

- Comando fora dos limites nao gera movimento.
- Comando incompleto nao trava o firmware.
- `MOV` continua bloqueado na v1, porque IK/FK ficam na IHM.

## 5. Movimento estimado sem motores

Comandos:

```text
START
POSOFF
ANG,10,0,90,0,0,0,50
POS?
HOME
POS?
```

Esperado:

```text
OK_MOVE_DONE
POS,10.00,0.00,90.00,0.00,0.00,0.00,50,0.0,0.0,0.0
OK_MOVE_DONE
POS,0.00,0.00,90.00,0.00,0.00,0.00,50,0.0,0.0,0.0
```

Aceite:

- O firmware atualiza a posicao estimada apos `ANG`.
- `HOME` retorna para a posicao home configurada.
- Como nao ha feedback, este teste valida software e contagem planejada, nao posicao fisica.

## 6. Sinais STEP/DIR sem motores

Pinos:

| Eixo | STEP | DIR |
|---|---:|---:|
| q1–q5 | RAMPS 1.4 | RAMPS 1.4 |
| q6 | GPIO14 | GPIO13 |

Comandos sugeridos:

```text
START
POSOFF
ANG,10,0,90,0,0,0,0
ANG,-10,0,90,0,0,0,0
HOME
```

Aceite:

- O STEP do eixo q1 pulsa durante movimento.
- O DIR muda quando o alvo muda de sentido.
- Eixos sem delta nao devem pulsar.

## 7. Garra servo MG996R

Primeiro sem servo:

- Medir GPIO4.
- Confirmar PWM em 50 Hz.
- `GRP,0` deve gerar pulso proximo de 1000 us.
- `GRP,50` deve gerar pulso proximo de 1500 us.
- `GRP,100` deve gerar pulso proximo de 2000 us.

Depois com servo:

- Usar fonte externa adequada para o MG996R.
- Ligar GND da fonte do servo ao GND do ESP32.
- Testar primeiro sem mecanismo acoplado, para evitar bater em fim mecanico.

Comandos:

```text
START
POSOFF
GRP,0
POS?
GRP,50
POS?
GRP,100
POS?
```

Aceite:

- `GRP` responde `OK_GRIPPER_DONE`.
- O campo `G` no `POS` acompanha 0, 50 e 100.
- O servo nao deve aquecer, vibrar forte ou bater mecanicamente.

## 8. Emergencia e parada

Comandos:

```text
START
ESTOP
ANG,10,0,90,0,0,0,0
GRP,50
```

Esperado:

```text
OK_START
OK_ESTOP
ERR_ESTOP
ERR_ESTOP
```

Aceite:

- Depois de `ESTOP`, novos movimentos ficam bloqueados.
- Para sair de `ESTOP` na v1, reiniciar o ESP32.

## 9. Observacoes para o relatorio

- O firmware v1 trabalha em malha aberta.
- `POS` e posicao estimada, baseada nos passos comandados e no home manual.
- Sem encoders, nao ha confirmacao fisica de que o eixo chegou no angulo.
- A validacao sem motores aprova protocolo, limites, estado, telemetria e geracao de sinais.
- A validacao fisica final depende de A4988, fonte, motores, mecanica e ajuste de corrente.
