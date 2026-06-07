# Plano Master de Firmware - ROBO_6DOF (ESP-IDF)

> Plano detalhado para desenvolvimento do firmware do ROBO_6DOF usando **ESP-IDF** no **ESP32-WROOM-32D**.  
> O firmware deve conversar com a IHM atual via USB/Serial, controlar motores de passo por drivers A4988 e controlar a garra por servo MG996R.  
> **V1:** controle em malha aberta, sem depender dos encoders AS5600 e sem multiplexador TCA9548A instalado.

---

## Premissas Importantes

1. **A IHM continua responsável por FK/IK.**  
   O firmware v1 não calcula cinemática inversa embarcada. A IHM envia comandos angulares por `ANG,q1,q2,q3,q4,q5,q6,G`.

2. **Comunicação principal:** USB/Serial, `115200 8N1`, ASCII, terminador `\n`.

3. **Motores de passo:** controle por drivers A4988 usando sinais `STEP` e `DIR`.

4. **V1 funciona sem sensores.**  
   Hoje existe apenas 1 encoder AS5600 disponível, então o firmware não pode depender de leitura absoluta para movimentar o robô.

5. **Expansão futura planejada:** multiplexador TCA9548A + múltiplos AS5600.  
   O código deve nascer preparado para trocar posição estimada por posição absoluta real quando os encoders forem instalados.

6. **Garra v1:** servo MG996R por PWM.  
   O firmware não usará eletroímã no v1. O comando `GRP,0-100` controla abertura/fechamento proporcional do servo.

7. **Home inicial manual no v1.**  
   Até existir homing confiável com sensores/fim de curso, o robô deve ser posicionado fisicamente em home antes de ligar ou resetar o ESP32.

8. **Segurança vem antes de precisão.**  
   `ESTOP` e botão físico de parada devem ter prioridade sobre qualquer movimento, comando serial ou sequência.

---

## Arquitetura Esperada do Firmware

> **Objetivo:** separar o firmware em módulos pequenos para facilitar teste, manutenção e expansão futura.

Estrutura futura sugerida:

```text
firmware/
└── robo_6dof_espidf/
    ├── CMakeLists.txt
    ├── sdkconfig.defaults
    ├── main/
    │   ├── CMakeLists.txt
    │   └── app_main.cpp
    └── components/
        ├── board_config/       # pinagem, limites, passos/grau, servo, features
        ├── serial_protocol/    # parser ASCII e respostas para IHM
        ├── robot_state/        # estados globais, safety, posição atual
        ├── motion_control/     # planner, conversão grau->passo, filas de movimento
        ├── stepper_a4988/      # geração STEP/DIR
        ├── gripper_servo/      # PWM LEDC para MG996R
        └── sensors/            # v1 estimado; futuro AS5600/TCA9548A
```

Regras de arquitetura:

- Nenhum pino deve ficar hardcoded dentro do controle de motor.
- Limites, relação mecânica, sentido e passos por grau ficam em configuração.
- O parser serial não deve acionar GPIO diretamente; ele chama serviços de estado/motion.
- A camada de sensores deve ter interface única para permitir trocar estimativa por leitura absoluta no futuro.

---

## Fase 1 - Preparação do Projeto ESP-IDF

> **Objetivo:** criar uma base compilável do firmware, ainda sem movimentar hardware.

### Passo 1.1 - Criar projeto base

Criar a pasta futura:

```text
firmware/robo_6dof_espidf/
```

Arquivos mínimos:

```text
CMakeLists.txt
sdkconfig.defaults
main/CMakeLists.txt
main/app_main.cpp
```

Métodos:

- Configurar target `esp32`.
- Criar `app_main()` com log inicial.
- Inicializar UART/console padrão do ESP-IDF.
- Enviar `OK_INIT_ROBOT` no boot para a IHM.

Teste:

- Rodar `idf.py set-target esp32`.
- Rodar `idf.py build`.
- Gravar no ESP32 com `idf.py flash monitor`.
- Ver no monitor serial a mensagem de boot e `OK_INIT_ROBOT`.

### Passo 1.2 - Criar componentes principais

Criar componentes vazios, mas compiláveis:

```text
board_config
serial_protocol
robot_state
motion_control
stepper_a4988
gripper_servo
sensors
```

Métodos:

- Cada componente deve ter `include/` e arquivo `.cpp`.
- Criar funções de inicialização simples.
- Evitar dependência circular entre módulos.

Teste:

- Compilar com todos os componentes linkados.
- Verificar que o boot não reinicia por watchdog.

---

## Fase 2 - Configuração de Hardware

> **Objetivo:** centralizar pinagem e parâmetros físicos em um único ponto do firmware.

### Passo 2.1 - Arquivo de pinagem

Criar configuração baseada no esquema elétrico do PDF.

Tabela inicial:

| Função | GPIO |
|---|---:|
| M1 STEP | GPIO13 |
| M1 DIR | GPIO14 |
| M2 STEP | GPIO16 |
| M2 DIR | GPIO17 |
| M3 STEP | GPIO18 |
| M3 DIR | GPIO19 |
| M4 STEP | GPIO23 |
| M4 DIR | GPIO25 |
| M5 STEP | GPIO26 |
| M5 DIR | GPIO27 |
| M6 STEP | GPIO32 |
| M6 DIR | GPIO33 |
| BT_START | GPIO34 |
| BT_STOP / Emergência | GPIO35 |
| FIM_DE_CURSO | GPIO2 |
| SERVO_GARRA | GPIO4 |

Observações:

- `GPIO34` e `GPIO35` são entradas somente; usar apenas como botões.
- `GPIO4` foi documentado no PDF como `ELETROIMA`, mas no v1 será reaproveitado para o servo da garra.
- A pinagem é referência inicial e pode mudar conforme a placa física evoluir.

Teste:

- Inicializar todos os pinos no boot.
- Logar a configuração carregada.
- Validar com LED/analisador lógico antes de conectar drivers e motores.

### Passo 2.2 - Configuração por junta

Criar tabela para cada junta:

| Campo | Descrição |
|---|---|
| `name` | Nome da junta: Base, Ombro, Cotovelo, Punho 1, Punho 2, Garra Rot. |
| `step_gpio` | GPIO STEP |
| `dir_gpio` | GPIO DIR |
| `min_deg` | Limite mínimo em graus |
| `max_deg` | Limite máximo em graus |
| `home_deg` | Ângulo considerado home |
| `steps_per_rev` | Passos por volta do motor |
| `microstep` | Microstepping configurado no A4988 |
| `gear_ratio` | Relação mecânica da junta |
| `invert_dir` | Inversão lógica do sentido |
| `max_speed_dps` | Velocidade máxima em graus/s |
| `max_accel_dps2` | Aceleração máxima em graus/s² |

Valores iniciais dos limites devem seguir a IHM:

| Junta | Home | Min | Max |
|---|---:|---:|---:|
| q1 Base | 0° | -180° | 180° |
| q2 Ombro | 0° | -90° | 90° |
| q3 Cotovelo | 90° | -180° | 180° |
| q4 Punho 1 | 0° | -180° | 180° |
| q5 Punho 2 | 0° | -180° | 180° |
| q6 Garra Rot. | 0° | -180° | 180° |

Teste:

- Criar função que valida se um alvo angular está dentro dos limites.
- Testar manualmente alvos válidos e inválidos.

---

## Fase 3 - Protocolo Serial Compatível com a IHM

> **Objetivo:** fazer o ESP32 responder exatamente ao protocolo que a IHM atual espera.

### Passo 3.1 - Parser de linhas ASCII

O firmware deve ler comandos terminados por `\n`.

Comandos v1:

| Comando | Formato | Ação |
|---|---|---|
| PING | `PING` | Handshake com a IHM |
| START | `START` | Arma o robô |
| STOP | `STOP` | Parada controlada |
| ESTOP | `ESTOP` | Parada imediata |
| HOME | `HOME` | Move para home configurado |
| ANG | `ANG,q1,q2,q3,q4,q5,q6,G` | Move juntas e atualiza garra |
| GRP | `GRP,Pos` | Move somente a garra |
| MOV | `MOV,X,Y,Z,Yaw,Pitch,Roll` | Não suportado no v1 |

Métodos:

- Remover `\r` e espaços no fim da linha.
- Separar tokens por vírgula.
- Validar quantidade de argumentos.
- Converter números com tratamento de erro.
- Nunca travar em comando malformado.

Teste:

- Enviar comandos via monitor serial.
- Validar resposta para comando correto, comando incompleto e comando desconhecido.

### Passo 3.2 - Respostas para a IHM

Respostas esperadas:

| Evento | Resposta |
|---|---|
| Boot | `OK_INIT_ROBOT` |
| PING | `OK_PONG` |
| START aceito | `OK_START` |
| STOP aceito | `OK_STOP` |
| ESTOP aceito | `OK_ESTOP` |
| Movimento concluído | `OK_MOVE_DONE` |
| Garra concluída | `OK_GRIPPER_DONE` |
| Formato inválido | `ERR_BAD_FORMAT` |
| Limite violado | `ERR_LIMIT` |
| Movimento sem START | `ERR_NOT_ARMED` |
| Robô em emergência | `ERR_ESTOP` |
| MOV no v1 | `ERR_UNSUPPORTED_MOV` |
| Falha genérica | `ERR_FAULT` |

Regras:

- A IHM considera qualquer `OK_*`, `ERR_*` ou `POS` como resposta válida ao handshake.
- `PING` deve responder rápido para evitar timeout de 2 segundos.
- `MOV` deve responder `ERR_UNSUPPORTED_MOV`, porque a IK fica na IHM.

Teste:

- Abrir a IHM em modo real.
- Conectar na porta COM do ESP32.
- Confirmar que a IHM não desconecta por falta de resposta.

### Passo 3.3 - Streaming de posição `POS`

Formato:

```text
POS,q1,q2,q3,q4,q5,q6,G,X,Y,Z
```

Frequência:

- V1: enviar a cada 100 ms.
- Equivale a 10 Hz, suficiente para a IHM atualizar painel e gêmeo digital.

Conteúdo no v1:

- `q1..q6`: posição estimada por passos desde o home manual.
- `G`: posição atual da garra em porcentagem.
- `X,Y,Z`: iniciar com `0.0,0.0,0.0` ou futuramente calcular FK embarcada simples.

Teste:

- Verificar logs RX da IHM recebendo `POS`.
- Confirmar que os ângulos atuais da IHM mudam após `ANG`.

---

## Fase 4 - Máquina de Estados e Segurança

> **Objetivo:** impedir movimentos indevidos e garantir resposta previsível em parada normal ou emergência.

### Passo 4.1 - Estados do robô

Estados:

| Estado | Significado |
|---|---|
| `BOOT` | Inicializando firmware |
| `IDLE` | Ligado, não armado |
| `ARMED` | Pronto para aceitar movimento |
| `MOVING` | Executando movimento |
| `STOPPED` | Parada controlada solicitada |
| `ESTOP` | Emergência ativa |
| `FAULT` | Falha crítica |

Transições principais:

- `BOOT -> IDLE` após inicialização.
- `IDLE -> ARMED` com `START`.
- `ARMED -> MOVING` com `ANG` ou `HOME`.
- `MOVING -> ARMED` quando movimento termina.
- Qualquer estado -> `ESTOP` com comando `ESTOP` ou botão de emergência.
- `MOVING -> STOPPED` com `STOP`.

Teste:

- Logar toda transição de estado.
- Verificar que `ANG` em `IDLE` retorna `ERR_NOT_ARMED`.

### Passo 4.2 - Parada controlada e emergência

`STOP`:

- Interrompe sequência/movimento de forma controlada.
- Pode desacelerar os motores se o planner já estiver rodando.
- Retorna `OK_STOP`.

`ESTOP`:

- Cancela geração de passos imediatamente.
- Bloqueia novos movimentos.
- Retorna `OK_ESTOP` e/ou envia `ESTOP`.
- Só sai por reset ou comando futuro de desbloqueio, ainda não definido no v1.

Botão físico:

- `BT_STOP` deve ter prioridade sobre serial.
- Entrada deve ter debounce.
- Ação recomendada no v1: entrar em `ESTOP`.

Teste:

- Iniciar movimento longo.
- Enviar `STOP` e validar parada.
- Iniciar novo movimento.
- Acionar `ESTOP` e validar bloqueio de novos comandos.

### Passo 4.3 - Fim de curso opcional

`FIM_DE_CURSO` existe na pinagem, mas o uso pode evoluir.

V1:

- Ler entrada e logar estado.
- Se habilitado em configuração, bloquear movimento na direção perigosa ou entrar em `FAULT`.

Futuro:

- Usar como parte do homing.
- Registrar posição de referência do eixo associado.

Teste:

- Simular fim de curso com jumper/botão.
- Confirmar que o firmware identifica a mudança.

---

## Fase 5 - Controle dos Motores de Passo com A4988

> **Objetivo:** converter ângulos recebidos da IHM em passos físicos nos motores.

### Passo 5.1 - Driver STEP/DIR básico

Implementar funções:

```text
stepper_init()
stepper_set_direction(axis, dir)
stepper_pulse(axis)
stepper_disable_all()
stepper_emergency_stop()
```

Métodos:

- Configurar `STEP` e `DIR` como saída.
- Garantir tempo mínimo de pulso compatível com A4988.
- Iniciar todos os `STEP` em nível baixo.
- Se existir pino `ENABLE` no hardware futuro, adicionar suporte global.

Teste:

- Sem motor conectado: medir pulsos no `STEP`.
- Confirmar `DIR` alternando conforme alvo maior/menor.

### Passo 5.2 - Conversão grau para passos

Fórmula base:

```text
motor_steps_per_joint_rev = steps_per_rev * microstep * gear_ratio
steps_per_degree = motor_steps_per_joint_rev / 360
target_steps = round(target_deg * steps_per_degree)
```

Regras:

- Cada junta pode ter `gear_ratio` diferente.
- Cada junta pode inverter direção.
- A posição estimada deve ser atualizada em passos e em graus.

Teste:

- Para cada junta, enviar alvo de `10°`.
- Verificar quantidade de passos esperada.
- Repetir para `-10°`.

### Passo 5.3 - Movimento multi-eixo

Implementar planner simples para mover até 6 eixos.

Requisitos:

- Todos os eixos devem iniciar e terminar juntos quando possível.
- O eixo com maior número de passos define a duração base.
- Eixos menores distribuem pulsos proporcionalmente.
- Velocidade inicial baixa para testes.

Método sugerido:

- V1 pode usar algoritmo tipo DDA/Bresenham para interpolar passos entre eixos.
- Começar com velocidade constante baixa.
- Depois adicionar aceleração trapezoidal.

Teste:

- Sem motores: verificar contagem de pulsos por eixo.
- Com 1 motor: validar sentido e suavidade.
- Com 6 motores: executar movimento curto e lento.

### Passo 5.4 - Limites de movimento

Antes de qualquer movimento:

- Validar `q1..q6` contra limites.
- Se qualquer junta violar limite, rejeitar o comando inteiro.
- Responder `ERR_LIMIT`.

Durante movimento:

- Se `STOP`, `ESTOP`, fim de curso ou falha ocorrer, interromper planner.

Teste:

- Enviar `ANG,999,0,90,0,0,0,0`.
- Esperado: `ERR_LIMIT` e nenhum pulso gerado.

### Passo 5.5 - Posição estimada no v1

Como o v1 não usa sensores:

- A posição atual do firmware é estimada pela contagem de passos.
- Ao boot, assumir que o robô está em `home_position`.
- `HOME` move para os ângulos home definidos em configuração.
- Se o motor perder passo, a estimativa fica incorreta até novo home manual.

Aviso técnico:

- Esta limitação deve ser documentada no relatório.
- A solução definitiva será usar AS5600 absoluto por junta no futuro.

Teste:

- Após boot, `POS` deve enviar home.
- Após `ANG`, `POS` deve enviar o alvo atingido.

---

## Fase 6 - Controle da Garra Servo MG996R

> **Objetivo:** controlar a garra mecânica articulada por PWM.

### Passo 6.1 - PWM LEDC 50 Hz

Configuração inicial:

| Parâmetro | Valor inicial |
|---|---:|
| Frequência | 50 Hz |
| Pulso fechado | 1000 us |
| Pulso aberto | 2000 us |
| Entrada lógica | 0 a 100% |
| GPIO | GPIO4 |

Métodos:

- Usar periférico LEDC do ESP32.
- Converter `0-100%` para largura de pulso.
- Saturar valores menores que 0 ou maiores que 100.

Teste:

- Com servo alimentado externamente e GND comum, enviar `GRP,0`, `GRP,50`, `GRP,100`.
- Confirmar movimento sem bater no limite mecânico.

### Passo 6.2 - Integração da garra com `ANG`

O último campo do comando `ANG` é `G`.

Exemplo:

```text
ANG,0,0,90,0,0,0,75
```

Comportamento:

- Mover juntas para o alvo.
- Atualizar garra para 75%.
- Registrar `G=75` no estado.
- Streaming `POS` deve refletir `G=75`.

Teste:

- Enviar `ANG` com `G=0` e depois `G=100`.
- Confirmar servo e valor no painel da IHM.

---

## Fase 7 - Integração com a IHM

> **Objetivo:** garantir que a IHM desenvolvida consiga controlar e monitorar o firmware real.

### Passo 7.1 - Handshake de conexão

Fluxo esperado:

1. Usuário desmarca `MODO TESTE`.
2. Usuário conecta na porta COM do ESP32.
3. IHM envia `PING`.
4. Firmware responde `OK_PONG` ou já envia `OK_INIT_ROBOT`.
5. IHM mantém conexão.

Teste:

- A IHM não deve mostrar "FALHA - SEM ESP32".
- Terminal deve mostrar RX/TX.

### Passo 7.2 - Botões principais

Validar:

| Botão IHM | Comando | Resposta |
|---|---|---|
| START | `START` | `OK_START` |
| STOP | `STOP` | `OK_STOP` |
| E-STOP | `ESTOP` | `OK_ESTOP` ou `ESTOP` |

Teste:

- Clicar cada botão e conferir estado/log do firmware.

### Passo 7.3 - Aba JUNTAS

Fluxo:

1. Usuário preenche q1..q6 e garra.
2. IHM envia `ANG`.
3. Firmware valida limites.
4. Firmware move motores.
5. Firmware envia `OK_MOVE_DONE`.
6. Firmware continua enviando `POS`.

Teste:

- Enviar pequenos ângulos com motores desconectados.
- Depois testar eixo por eixo com motor real.

### Passo 7.4 - Abas FK e IK

Regra:

- A IHM calcula FK/IK.
- O firmware não calcula `MOV`.
- Quando usuário envia resultado de IK, a IHM deve enviar `ANG`.

Teste:

- Usar aba IK para calcular uma pose.
- Clicar enviar.
- Verificar que o firmware recebe `ANG`, não precisa executar `MOV`.

### Passo 7.5 - Sequenciamento

Fluxo:

1. IHM carrega/adiciona pontos.
2. IHM envia uma sequência de comandos `ANG`.
3. Firmware executa um por vez.
4. IHM controla delay entre pontos.

Teste final:

- Criar sequência curta:
  - home;
  - mover q1 poucos graus;
  - abrir garra;
  - fechar garra;
  - retornar home.
- Executar em baixa velocidade.

---

## Fase 8 - Expansão Futura para TCA9548A e AS5600

> **Objetivo:** deixar registrado e planejado que o firmware deve evoluir para posição absoluta real por junta.

### Passo 8.1 - Situação atual dos sensores

Estado atual:

- Existe apenas 1 encoder AS5600 disponível.
- O multiplexador TCA9548A ainda não está disponível.
- Portanto, sensores não entram como requisito para o primeiro firmware funcional.

Decisão:

- V1 usa `EstimatedSensorProvider`.
- Futuro usa `As5600SensorProvider`.

Teste:

- Garantir que o firmware compila e roda com sensores desabilitados.

### Passo 8.2 - Interface `SensorProvider`

Criar no planejamento uma interface conceitual:

```text
SensorProvider
├── init()
├── update()
├── get_joint_angle_deg(axis)
├── is_available(axis)
└── get_fault(axis)
```

Implementações:

| Implementação | Uso |
|---|---|
| `EstimatedSensorProvider` | V1, baseado nos passos enviados |
| `As5600SensorProvider` | Futuro, leitura absoluta via TCA9548A |

Regra:

- O resto do firmware deve perguntar a posição atual pela interface, não diretamente pela contagem de passos.
- Isso evita reescrever motion/protocolo quando os encoders entrarem.

Teste futuro:

- Trocar provider por configuração e manter mesmo protocolo `POS`.

### Passo 8.3 - TCA9548A

Quando o multiplexador chegar:

- Inicializar I2C do ESP32.
- Inicializar TCA9548A no endereço padrão.
- Mapear canal do MUX para junta.
- Permitir canais desativados por configuração.

Exemplo de mapeamento futuro:

| Junta | Canal TCA9548A |
|---|---:|
| q1 | 0 |
| q2 | 1 |
| q3 | 2 |
| q4 | 3 |
| q5 | 4 |
| q6 | 5 |

Teste futuro:

- Ler um AS5600 por vez.
- Confirmar que trocar canal não trava o barramento.

### Passo 8.4 - AS5600 como posição absoluta

O AS5600 fornece posição angular absoluta.

Para cada junta será necessário salvar:

| Campo | Descrição |
|---|---|
| `zero_offset_raw` | Leitura bruta quando a junta está em home |
| `invert` | Se o sentido do encoder é invertido |
| `scale` | Conversão para graus da junta |
| `mechanical_offset_deg` | Ajuste fino mecânico |
| `valid_min_deg` | Limite mínimo confiável |
| `valid_max_deg` | Limite máximo confiável |

Regra importante:

- Quando AS5600 estiver ativo, `POS` deve enviar o ângulo real absoluto da junta, não apenas o alvo comandado.

Exemplo obrigatório:

```text
Se a junta andou 60 graus e o encoder absoluto confirma 60 graus,
o firmware deve enviar POS com q correspondente = 60.0.
```

Impacto na IHM:

- A IHM deve mostrar esse valor como posição atual real.
- A IHM poderá salvar esse valor em sequências.
- A IHM poderá usar esse valor como referência de calibração.

Teste futuro:

- Mover manualmente uma junta desligada.
- Verificar que `POS` muda conforme o AS5600, mesmo sem comando de motor.

### Passo 8.5 - Calibração em NVS

Salvar em NVS:

- offset de home;
- sentido do encoder;
- última posição conhecida;
- tolerância de erro;
- flags de sensor habilitado/desabilitado.

Comandos futuros possíveis:

| Comando futuro | Ação |
|---|---|
| `CAL,axis` | Salva posição atual como home do eixo |
| `ZERO,axis` | Zera offset do eixo |
| `SENS?` | Lista sensores disponíveis |
| `CFG?` | Retorna configuração resumida |

Esses comandos não precisam existir no v1, mas o plano deve reservar espaço para eles.

Teste futuro:

- Calibrar, reiniciar ESP32 e confirmar que offset foi preservado.

### Passo 8.6 - Comparação posição comandada vs posição real

Quando os encoders entrarem:

- `target_deg`: alvo enviado pela IHM.
- `estimated_deg`: posição por passos.
- `absolute_deg`: posição real do AS5600.
- `error_deg = absolute_deg - target_deg`.

Se `abs(error_deg)` passar da tolerância:

- Enviar `ERR_POSITION_ERROR`.
- Parar movimento ou entrar em `FAULT`, conforme configuração.

Teste futuro:

- Simular erro deslocando o eixo.
- Confirmar que firmware detecta diferença.

---

## Fase 9 - Checklist de Testes

> **Objetivo:** validar o firmware em camadas, sem ligar tudo de uma vez.

### 9.1 - Teste sem hardware de potência

- ESP32 ligado via USB.
- Sem A4988 energizado.
- Validar boot, parser, respostas e streaming.

Critério:

- IHM conecta sem timeout.
- `PING` responde.
- `POS` chega a 10 Hz.

### 9.2 - Teste com analisador lógico ou LED

- Validar `STEP` e `DIR` de cada eixo.
- Enviar movimentos pequenos.
- Confirmar quantidade aproximada de pulsos.

Critério:

- Nenhum pulso em comando inválido.
- `DIR` muda conforme sentido.

### 9.3 - Teste com um motor

- Conectar apenas um A4988 e um motor.
- Corrente do driver ajustada.
- Velocidade baixa.
- Movimento curto.

Critério:

- Motor gira no sentido esperado.
- Não perde passo em baixa velocidade.
- `STOP` e `ESTOP` interrompem movimento.

### 9.4 - Teste com seis motores

- Conectar todos os eixos.
- Executar movimentos pequenos.
- Validar aquecimento dos drivers.
- Validar fonte externa.

Critério:

- Movimento coordenado.
- Sem resets do ESP32.
- Sem travamento da serial.

### 9.5 - Teste com garra servo

- Servo alimentado por fonte adequada.
- GND comum com ESP32.
- Testar `GRP,0`, `GRP,50`, `GRP,100`.

Critério:

- Servo não bate mecanicamente.
- `POS` reflete porcentagem da garra.

### 9.6 - Teste final com IHM

- Conectar IHM real.
- Clicar `START`.
- Enviar ângulos pela aba JUNTAS.
- Calcular IK na IHM e enviar resultado.
- Executar sequência curta.
- Testar `STOP`.
- Testar `E-STOP`.

Critério final:

- Robô executa sequência curta em baixa velocidade.
- IHM recebe telemetria contínua.
- Emergência bloqueia novos movimentos.

---

## Critérios de Aceite do Firmware V1

- [ ] ESP32 inicializa e envia `OK_INIT_ROBOT`.
- [ ] IHM conecta sem timeout.
- [ ] `PING` responde `OK_PONG`.
- [ ] `START`, `STOP` e `ESTOP` funcionam.
- [ ] `ANG` valida limites e move por STEP/DIR.
- [ ] `GRP` controla servo MG996R.
- [ ] `HOME` retorna para home configurado.
- [ ] `MOV` responde `ERR_UNSUPPORTED_MOV`.
- [ ] `POS` é enviado continuamente a cada 100 ms.
- [ ] Firmware roda sem AS5600 e sem TCA9548A.
- [ ] Código está preparado para adicionar posição absoluta no futuro.

---

## Pontos que Não Podem Ser Perdidos

1. O firmware v1 é **open-loop**: usa posição estimada pelos passos.
2. O AS5600 será usado futuramente como **posição absoluta real**.
3. O TCA9548A ainda não existe no hardware atual, mas deve ser considerado na arquitetura.
4. A IHM deve futuramente receber e salvar ângulos reais vindos dos encoders.
5. Se uma junta andar 60 graus e o encoder absoluto confirmar isso, o `POS` deve mandar `60.0`.
6. O comando `MOV` não será operacional no v1; a IHM calcula IK e manda `ANG`.
7. A garra operacional do v1 é servo MG996R, não eletroímã.
8. Segurança física e `ESTOP` têm prioridade sobre qualquer função de movimento.

---

## Próxima Etapa Recomendada

Começar pela **Fase 1** criando o projeto ESP-IDF mínimo e validando comunicação serial com a IHM antes de ligar qualquer motor. Depois disso, implementar `START`, `STOP`, `ESTOP`, `PING` e `POS`; só então partir para pulsos STEP/DIR.
