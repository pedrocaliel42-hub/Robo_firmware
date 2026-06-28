# Plano: ESP32 Mestre + Arduino Mega RAMPS para ROBO_6DOF

## Resumo

A nova arquitetura divide o controle dos motores entre duas placas:

- A IHM continua falando somente com o ESP32.
- O ESP32 vira o mestre do sistema, mantendo o protocolo serial externo atual.
- O ESP32 controla diretamente apenas o eixo `q6` e a garra.
- O Arduino Mega com shield RAMPS 1.4 controla os motores `q1` a `q5`.
- A comunicação ESP32-Mega usa UART TTL a `115200 8N1`.

O objetivo é preservar a IHM existente e mover a maior parte da geração de pulsos STEP/DIR para o Mega, que fica fisicamente conectado ao shield de drivers.

## Mudanças no ESP32

O protocolo externo com a IHM permanece:

- `PING`
- `START`
- `STOP`
- `ESTOP`
- `HOME`
- `ANG,q1,q2,q3,q4,q5,q6,G`
- `GRP,Pos`
- `POS?`
- `POSON`
- `POSOFF`

O ESP32 passa a ter três responsabilidades principais:

- Validar comandos da IHM, limites e estado de segurança.
- Repassar movimentos de `q1` a `q5` ao Mega.
- Executar localmente `q6` e a garra.

Pinagem ESP32 adotada:

| Função | GPIO |
|---|---:|
| q6 STEP | GPIO32 |
| q6 DIR | GPIO33 |
| Servo garra | GPIO4 |
| UART2 RX do Mega | GPIO16 |
| UART2 TX para Mega | GPIO17 |
| BT_START | GPIO34 |
| BT_STOP / Emergência | GPIO35 |
| FIM_DE_CURSO | GPIO2 |

Os antigos pinos de `q1` a `q5` no ESP32 deixam de gerar pulsos.

## Protocolo Interno ESP32-Mega

Comunicação:

- UART TTL
- `115200 8N1`
- ASCII
- Terminador `\n`
- ESP32 como mestre
- Mega como executor dos eixos `q1` a `q5`

Comandos:

| Comando | Resposta esperada | Ação |
|---|---|---|
| `MPING` | `MOK_PONG` | Teste de comunicação |
| `MPREP,seq,q1,q2,q3,q4,q5` | `MREADY,seq` | Valida e prepara movimento |
| `MGO,seq` | `MDONE,seq` | Executa movimento preparado |
| `MSTOP,seq` | `MSTOPPED,seq` | Parada controlada |
| `MESTOP` | `MOK_ESTOP` | Emergência imediata |

Erros possíveis:

- `MERR_FORMAT`
- `MERR_LIMIT`
- `MERR_BUSY`
- `MERR_ESTOP`
- `MERR_FAULT`

Fluxo de `ANG`:

1. A IHM envia `ANG,q1,q2,q3,q4,q5,q6,G` ao ESP32.
2. O ESP32 valida os seis ângulos e a garra.
3. O ESP32 envia `MPREP,seq,q1,q2,q3,q4,q5` ao Mega.
4. Se o Mega responder `MREADY,seq`, o ESP32 envia `MGO,seq`.
5. O Mega executa `q1` a `q5`.
6. O ESP32 executa `q6` localmente.
7. O ESP32 responde `OK_MOVE_DONE` à IHM somente quando Mega e `q6` finalizam sem erro.

Se o Mega não responder dentro do timeout, o ESP32 retorna `ERR_FAULT` para a IHM e não atualiza a posição estimada.

## Arduino Mega / RAMPS 1.4

O firmware do Mega será um projeto Arduino IDE:

```text
arduino_mega_ramps/
└── robo_mega_ramps/
    └── robo_mega_ramps.ino
```

Mapeamento lógico:

| Junta | Slot RAMPS |
|---|---|
| q1 | X |
| q2 | Y |
| q3 | Z |
| q4 | E0 |
| q5 | E1 |

Pinagem base RAMPS 1.4:

| Slot | STEP | DIR | ENABLE |
|---|---:|---:|---:|
| X | 54 | 55 | 38 |
| Y | 60 | 61 | 56 |
| Z | 46 | 48 | 62 |
| E0 | 26 | 28 | 24 |
| E1 | 36 | 34 | 30 |

Antes de energizar motores, confirmar a pinagem no shield físico ou silk screen.

### Ajuste de reduções mecânicas

Nesta fase será usado o Plano A: redução fixa em macro de firmware.

No firmware do Mega, ajustar `q1` a `q5` em:

```text
arduino_mega_ramps/robo_mega_ramps/robo_mega_ramps.ino
```

Macros:

```cpp
#define ROBO_Q1_GEAR_RATIO 1.0F
#define ROBO_Q2_GEAR_RATIO 1.0F
#define ROBO_Q3_GEAR_RATIO 1.0F
#define ROBO_Q4_GEAR_RATIO 1.0F
#define ROBO_Q5_GEAR_RATIO 1.0F
```

No firmware do ESP32, ajustar `q6` em:

```text
components/board_config/board_config.cpp
```

Macro:

```cpp
#define ROBO_Q6_GEAR_RATIO 1.0F
```

A fórmula usada é:

```text
steps_per_degree = steps_per_rev * microstep * gear_ratio / 360
```

Exemplo: motor de 200 passos, microstep 16 e redução 10:1:

```text
steps_per_degree = 200 * 16 * 10 / 360 = 88.888 passos/grau
```

### Rampa de aceleração do Mega

Os movimentos absolutos (`MGO`) e relativos (`MJOG`) usam o mesmo perfil
coordenado de aceleração, cruzeiro e desaceleração. Os parâmetros ficam no
início de `robo_mega_ramps.ino`:

```cpp
constexpr uint16_t kStepIntervalStartUs = 800;
constexpr uint16_t kStepIntervalCruiseUs = 120;
constexpr unsigned long kAccelerationSteps = 1200UL;
```

- `kStepIntervalStartUs`: intervalo de partida e chegada; maior significa mais
  suave e lento.
- `kStepIntervalCruiseUs`: intervalo na velocidade máxima; menor significa
  mais rápido.
- `kAccelerationSteps`: duração de cada rampa em passos; maior significa uma
  transição mais gradual.

Movimentos longos possuem perfil trapezoidal. Em movimentos curtos, as rampas
se encontram antes da velocidade máxima e formam um perfil triangular. A
calibração deve começar com o robô sem carga, reduzindo o intervalo de cruzeiro
gradualmente e verificando perda de passos, ruído e aquecimento.

Ligação UART:

- Mega RX1 pin 19 recebe TX do ESP32.
- Mega TX1 pin 18 envia para RX do ESP32.
- GND comum obrigatório.
- Usar divisor de tensão ou level shifter no TX 5 V do Mega para o RX 3.3 V do ESP32.

## Testes

1. ESP32 sem Mega:
   - IHM conecta.
   - `PING` responde `OK_PONG`.
   - `ANG` retorna erro claro se o Mega não responder.

2. UART ESP32-Mega:
   - Validar `MPING`.
   - Validar timeout.
   - Validar erro de formato.
   - Validar reconexão após reset do Mega.

3. Mega sem motores:
   - Validar `MPREP`, `MGO`, `MDONE`.
   - Conferir pulsos STEP com LED ou analisador lógico.

4. Teste por eixo:
   - Energizar um driver por vez nos slots X/Y/Z/E0/E1.
   - Confirmar sentido, enable, contagem de passos e aquecimento.

5. Teste coordenado:
   - Enviar `ANG` com mudança em `q1..q6`.
   - Confirmar que `OK_MOVE_DONE` só sai após Mega e ESP32 finalizarem.

6. Segurança:
   - `STOP` deve enviar parada ao Mega e parar `q6`.
   - `ESTOP` deve interromper tudo e bloquear novos movimentos.
   - Timeout do Mega deve virar `ERR_FAULT` ou `ERR_ESTOP`.

## Premissas

- O eixo mantido no ESP32 é `q6 Garra Rot.`.
- O Mega controla exatamente `q1` a `q5`.
- O shield considerado é RAMPS 1.4.
- O firmware do Mega usa Arduino IDE `.ino`.
- A IHM não será alterada nesta fase.
- O arquivo está em UTF-8 limpo.
