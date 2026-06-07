# Plano de Teste IHM - Fase 7

> Objetivo: validar a integracao da IHM real com o firmware ESP-IDF na raiz do projeto.
> Estes testes podem ser feitos sem motores conectados.

## 1. Flash e conexao

- Fazer flash do projeto `C:\Users\pedro\Robo_firmware`.
- Abrir a IHM.
- Desmarcar `MODO TESTE`.
- Selecionar a porta COM do ESP32.
- Conectar.

Aceite:

- A IHM nao deve mostrar falha por falta de ESP32.
- `PING` deve receber `OK_PONG`.
- O boot ainda pode mostrar logs do ESP-IDF, mas os comandos do firmware devem responder com `OK_*`, `ERR_*` ou `POS`.

## 2. Botoes principais

Na IHM:

- Clicar `START`.
- Clicar `STOP`.
- Clicar `START` novamente.
- Clicar `E-STOP`.

Esperado na serial:

```text
OK_START
OK_STOP
OK_START
OK_ESTOP
```

Aceite:

- Depois de `E-STOP`, movimentos devem responder `ERR_ESTOP`.
- Para sair de `E-STOP` na v1, reiniciar o ESP32.

## 3. Aba Juntas

Com o robo armado:

- Enviar q1=0, q2=0, q3=90, q4=0, q5=0, q6=0, garra=50.
- Depois enviar q1=10 mantendo os outros valores.
- Depois enviar q1=999 para testar limite.

Esperado:

```text
OK_MOVE_DONE
POS,0.00,0.00,90.00,0.00,0.00,0.00,50,0.0,0.0,0.0
OK_MOVE_DONE
POS,10.00,0.00,90.00,0.00,0.00,0.00,50,0.0,0.0,0.0
ERR_LIMIT
```

Aceite:

- A IHM deve atualizar os angulos atuais a partir do `POS`.
- Movimento fora do limite deve ser rejeitado sem travar a conexao.

## 4. Garra

Na IHM, testar valores da garra:

- 0
- 50
- 100

Aceite:

- O firmware deve responder `OK_GRIPPER_DONE` quando usar comando `GRP`.
- O campo `G` do `POS` deve acompanhar o valor enviado.
- Se houver servo real, usar fonte externa e GND comum com o ESP32.

## 5. FK e IK

Regra da v1:

- A IHM calcula FK/IK.
- O firmware nao executa `MOV`.
- O envio final para o ESP32 deve ser `ANG,q1,q2,q3,q4,q5,q6,G`.

Teste:

- Calcular uma pose na aba IK.
- Enviar o resultado para o robo.
- Verificar se o ESP32 recebe `ANG` e responde `OK_MOVE_DONE`.

Se a IHM enviar `MOV`, o esperado e:

```text
ERR_UNSUPPORTED_MOV
```

## 6. Sequencia curta

Criar uma sequencia simples na IHM:

```text
ANG,0,0,90,0,0,0,50
ANG,10,0,90,0,0,0,50
ANG,10,0,90,0,0,0,100
ANG,0,0,90,0,0,0,0
```

Aceite:

- Cada ponto deve terminar com `OK_MOVE_DONE`.
- `POS` deve refletir a ultima posicao estimada.
- Sem motores, este teste valida contrato, sequenciamento e telemetria.
