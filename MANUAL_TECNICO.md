# Manual técnico — Stewart Ball-on-Plate

Referência interna do firmware: o que cada módulo faz, como manter, como estender
e quais divergências entre comentários e código existem hoje. Para operação
(comandos, telemetria, captura), ver `GUIA_DE_USO.md`.

Nota sobre o título: este arquivo cumpre o papel de manual de referência e
manutenção. Um `README.md` curto na raiz, apontando para este manual e para o
guia de uso, é opcional e complementar.

As descrições abaixo seguem o que o código faz nos arquivos fornecidos. Onde um
comentário do código diz uma coisa e o código faz outra, o manual segue o código
e registra a divergência na seção 8.

---

## 1. Estrutura de pastas

```
StewartBallOnPlate/
├── StewartBallOnPlate.ino     ponto de entrada (setup/loop)
├── config.h                   pinos, geometria, ganhos, temporização
├── types.h                    structs compartilhadas e enum de modos
├── kinematics.{h,cpp}         cinemática inversa (pose → 6 servos)
├── imu_sensor.{h,cpp}         BNO085 via SPI1 + recuperação
├── touch_sensor.{h,cpp}       touchscreen resistivo → posição/velocidade
├── servo_driver.{h,cpp}       saída PCA9685
├── attitude_ctrl.{h,cpp}      malha interna (atitude)
├── ball_ctrl.{h,cpp}          malha externa (posição da bola)
├── supervisor.{h,cpp}         orquestração, modos, telemetria, serial
├── tools/
│   ├── telemetry_logger.py    captura/comando/plot (roda no PC)
│   └── scripts/               sequências temporizadas (.txt)
└── logs/                      saída de captura (gerada em runtime)
```

O Arduino compila apenas a raiz do sketch e a subpasta `src/`. A pasta `tools/`
não é compilada e pode conviver com o sketch sem efeito sobre a build.

---

## 2. Arquitetura

Fluxo de dados em `BALL_BALANCE` (cascata completa):

```
Touch ─► BallCtrl ─► inclinação desejada (tilt_ref)
                          │
              + trim      ▼
        AttitudeCtrl(tilt_ref, IMU) ─► comando de velocidade (graus/s)
                          │
              integra (∫dt) ─► pose absoluta (roll, pitch)
                          │
                  Kinematics ─► 6 ângulos de servo ─► Servos (PCA9685)
```

Cada periférico tem um módulo único que o controla; nenhum módulo conhece o
hardware de outro. O supervisor é o único dono do estado: ele guarda as structs e
as passa por referência aos módulos.

Temporização (de `config.h`):

- Laço principal alvo: `LOOP_PERIOD_US` = 10000 µs (100 Hz). O `tick()`
  rate-limita por esse período.
- Malha externa (bola): roda uma vez a cada `BALL_LOOP_DIV` ticks. Com
  `BALL_LOOP_DIV = 3`, isso dá ~33 Hz. Os comentários do código dizem 20 Hz
  (divisor 5); ver seção 8.
- Telemetria: `SERIAL_PERIOD_MS` = 20 ms (até 50 Hz).
- Relatórios do BNO085: `BNO_REPORT_US` = 10000 µs (100 Hz).

---

## 3. Princípio de controle (forma em velocidade)

O controlador de atitude não emite um ângulo absoluto. Ele emite uma taxa de
variação da pose (graus/s), que o supervisor integra no tempo (`integratePose`)
até uma pose absoluta. Em regime, o equilíbrio exige derivada de pose nula, o que
só ocorre com erro nulo. Por isso o erro em regime tende a zero sem precisar de
termo integral grande.

A integração só ocorre quando o IMU está fresco; com IMU parado, a pose congela
no último valor válido (proteção contra windup sobre medida morta). A pose
integrada é limitada ao espaço de trabalho (`POSE_TILT_MAX`), não ao limite de
velocidade (`ATT_CMD_MAX`).

Estado dessa integração: `pose_cmd_roll` e `pose_cmd_pitch`, em `supervisor.cpp`.
São zerados em toda troca de modo.

---

## 4. Convenções do projeto

- Sem números mágicos fora de `config.h`. Pinos, geometria, ganhos, limites e
  temporização ficam todos lá. Se for ajustar algo, ajuste em `config.h`.
- Structs simples (POD) em `types.h`, passadas por referência. Nenhum módulo
  aloca em heap para isso.
- Um módulo por periférico. `servo_driver` é o único que fala com o PCA9685;
  `imu_sensor`, com o BNO085; `touch_sensor`, com o touchscreen.
- Namespaces por módulo (`Kinematics`, `IMU`, `Touch`, `Servos`, `AttitudeCtrl`,
  `BallCtrl`, `Supervisor`), cada um com `init()` e a função de trabalho.

---

## 5. Módulos

### 5.1 `config.h`
Fonte única de pinos, geometria, ganhos e tempos. Pontos a conhecer:

- Pinos efetivos: PCA9685 em I2C (SDA 18, SCL 19, endereço 0x40); BNO085 em SPI1
  (SCK 14, MOSI 15, MISO 12, CS 13, INT 11, RST 10); touch em D2–D7 (GP2–GP7) e
  ADC GP26/GP27.
- Geometria: `L1` (horn, 24.95 mm), `L2` (haste, 160 mm), `Z_HOME` (149.33 mm),
  vetores `BASE_PX/PY`, `PLAT_PX/PY`, `DIR_X/Y`, e os trims/sinais por servo.
- Limites: `SERVO_MIN_DEG`/`MAX_DEG`, `POSE_TILT_MAX`, `ATT_CMD_MAX`,
  `BALL_OUT_MAX`, `BALL_I_MAX`.
- Ganhos por eixo da malha interna (`ATT_KP/KI/KD_ROLL` e `_PITCH`) e da malha da
  bola (`BALL_KP/KI/KD_CLOSE` e `_FAR`, com `BALL_TRANSITION_DIST`).
- Há blocos `OLD` comentados (geometria e ganhos antigos) e várias alternativas
  comentadas em fim de linha. Não afetam a build; convém limpar após consolidar.

### 5.2 `types.h`
Define as structs trocadas entre módulos e o `enum class OpMode`:

- `Pose` (x, y, z, roll, pitch, yaw): pose de 6 DOF. Translação em mm, rotação em
  graus.
- `Attitude` (roll, pitch, yaw e suas taxas, `valid`): atitude do IMU, referida à
  gravidade; taxas vêm do giroscópio calibrado.
- `BallState` (x, y, vx, vy, `detected`, `timestamp_ms`): saída do touch.
- `ServoCmd` (`angle[6]`): comando dos seis servos, em graus.
- `TiltCmd` (roll, pitch): inclinação desejada; é a saída da malha da bola e a
  referência da malha de atitude.
- `OpMode`: os oito modos. A ordem define o inteiro reportado na telemetria.
  Acrescente novos modos no fim para não deslocar os inteiros existentes.

### 5.3 `kinematics.{h,cpp}`
Cinemática inversa: dada uma `Pose`, resolve os seis ângulos de servo. É o único
módulo que conhece a geometria.

- `solve(pose, cmd)`: para cada perna monta o vetor da base à junta da plataforma
  e resolve `A·cosθ + B·sinθ = C`. Se o discriminante for negativo (pose fora do
  espaço de trabalho daquela perna), retorna `false` e o servo mantém o valor
  anterior. O ângulo é convertido para graus de servo com `SERVO_SIGN` e
  `SERVO_TRIM` e limitado ao envelope.
- `makeTiltPose(roll, pitch)`: pose só de inclinação, na altura `Z_HOME`.
- `computeJoints(cmd, out)`: forward, calcula a ponta de cada horn (depuração).

### 5.4 `imu_sensor.{h,cpp}`
Lê o BNO085 por SPI1 e mantém a recuperação não bloqueante.

- Lê dois relatórios: o vetor de rotação (vira roll/pitch/yaw, com offset de
  montagem) e o giroscópio calibrado (taxas, sem diferenciação numérica). Usar o
  giroscópio direto evita o pico de derivada e a amplificação de ruído de
  quantização da diferenciação numérica.
- `update(att)` é não bloqueante: drena eventos pendentes, com limite por chamada
  (`IMU_DRAIN_MAX`), e só chama `getSensorEvent()` quando o pino INT indica dado
  pronto (evita o spin de espera por INT quando o FIFO está vazio).
- Rejeição de glitch: descarta saltos de um quadro acima de `ATT_JUMP_MAX`;
  descarta giroscópio acima de `GYRO_MAX_VALID`.
- Recuperação por máquina de estados (`HEALTHY → SW_REENABLE → HW_RESET_PULSE →
  HW_RESET_BOOT`), toda temporizada com `millis()`. Nenhum estágio bloqueia, então
  os modos de malha aberta continuam durante a recuperação.
- `isFresh()` indica se chegou vetor de rotação válido dentro de
  `IMU_TIMEOUT_MS`. As malhas usam isso para decidir se confiam na atitude.

Constantes atuais: `IMU_COUPLING_ANGLE = 0` (correção de desalinhamento Z
desligada) e `GYRO_ALPHA = 0` (sem filtro passa-baixa no giroscópio, passagem
direta).

### 5.5 `touch_sensor.{h,cpp}`
Lê o touchscreen resistivo e estima posição e velocidade.

- `readAxis()`: faz `TS_SAMPLES` leituras, descarta as duas menores e as duas
  maiores e verifica a faixa restante contra `TS_MAX_RANGE_ADC`; retorna a média
  aparada (`TS_SAMPLES` precisa ser ≥ 6, pois descarta 4).
- `update(ball)`: lê os dois eixos, detecta ausência de toque (deadzone),
  converte ADC → mm pela transformada afim (`TS_CAL_*`), faz verificação de faixa
  e rejeição de outlier por limiar (`TS_THRESHOLD_MM`), e estima velocidade por
  diferença finita.
- Saída bruta para calibração: há linhas comentadas no fim de `update()` que
  gravariam `raw_xp`/`raw_yp` em vez do valor em mm. Para calibrar a transformada
  afim, descomente esse trecho temporariamente para capturar o ADC bruto, levante
  os coeficientes e reverta.

### 5.6 `servo_driver.{h,cpp}`
Saída pelo PCA9685, canais 1 a 6.

- `write(cmd)`: limita cada ângulo ao envelope e mapeia 0–180° para
  `SERVO_PWM_MIN`–`SERVO_PWM_MAX`.
- `writeNeutral()`: põe todos em 90° + trim. É a pose segura de partida, chamada
  em `init()`.

### 5.7 `attitude_ctrl.{h,cpp}`
Malha interna (2 DOF). Estrutura em uso:

```
cmd = Kp·e + Ki·∫e·dt − Kd·ω_gyro
```

com `e = ref − medido` e a derivada vinda do giroscópio (não da medida
diferenciada), o que elimina o pico de derivada no degrau de referência. A saída
é limitada a `ATT_CMD_MAX` e interpretada como velocidade (graus/s); a integração
para pose absoluta é feita no supervisor.

Observações da configuração atual: `ATT_KI = 0` nos dois eixos, então a malha
opera como PD. O bloco `deadband_punch` está zerado (código inerte).
`ATT_DEADBAND = 0` (sem zona morta). Há um caminho de feedforward comentado, não
usado por não ser robusto a inclinação da base.

### 5.8 `ball_ctrl.{h,cpp}`
Malha externa (PID por eixo, com escalonamento de ganho).

- `update(ball, dt, out)`: calcula o erro até o alvo, aplica zona morta
  (`BALL_DEADBAND`), escolhe ganhos `CLOSE`/`FAR` conforme a distância
  (`BALL_TRANSITION_DIST`), roda PID por eixo (derivada sobre o erro bruto,
  anti-windup em `BALL_I_MAX`, saída em `BALL_OUT_MAX`) e mapeia erro da bola
  para inclinação no mundo (X → pitch, Y → roll, com sinal). Os sinais do
  mapeamento estão marcados no código e devem ser ajustados se os eixos físicos
  responderem invertidos.
- `setTarget(x, y)`: define o alvo (mm, referencial da placa).
- Configuração atual: `BALL_KI = 0`, então também opera como PD.

### 5.9 `supervisor.{h,cpp}`
Orquestra tudo. Responsabilidades:

- Guarda o estado global (atitude, bola, referências, pose, comando de servo,
  telemetria) e os offsets de trim e a pose integrada.
- `tick()`: poll do IMU (sempre), rate-limit do laço, divisão da malha externa, o
  `switch` por modo, e a telemetria. Cada modo decide se precisa de IMU fresco.
- `setMode(mode)`: reseta os controladores e o estado de cascata em qualquer
  troca de modo, e reemite o cabeçalho de telemetria.
- `handleSerial()` + `processCommand()`: parser de comandos não bloqueante,
  acumula caracteres e só interpreta no fim de linha.
- Telemetria: `sendTelemetry()` (dados) e `sendTelemetryHeader()` (cabeçalho).

A máquina de três fases de `BALL_BALANCE` (`CLOSED`/`OPEN`/`RELEVELING`) vive
aqui, assim como a integração de pose, a rampa de relevel (`slewToZero`) e o
critério de atitude nivelada (`attitudeSettledLevel`).

### 5.10 `StewartBallOnPlate.ino`
Ponto de entrada. `setup()` chama `Supervisor::init()`; `loop()` chama
`handleSerial()` e `tick()`. O cabeçalho de comentários desse arquivo lista pinos
desatualizados; ver seção 8.

### 5.11 `tools/telemetry_logger.py`
Roda no PC, não no Pico. Captura a telemetria, opcionalmente envia comandos
(interativo ou por script), salva CSV e gera gráficos e estatísticas. Detalhes de
uso no `GUIA_DE_USO.md`, seção 8. As colunas esperadas estão em
`EXPECTED_COLUMNS`; se a telemetria do firmware mudar, esse vetor precisa
acompanhar (ver seção 7.5).

---

## 6. Manutenção

### 6.1 Ajustar ganhos
Edite os valores em `config.h` e recompile. Não há ajuste em tempo de execução: o
comando `GAINS` não está implementado (seção 8). Roll e pitch têm ganhos
separados, porque a placa retangular tem inércias diferentes por eixo.

### 6.2 Ajustar geometria
`L1`, `L2`, `Z_HOME` e os vetores `BASE_*`, `PLAT_*`, `DIR_*` em `config.h`. Após
qualquer mudança, valide numericamente a cinemática (ex. verificar que no nível
todos os servos ficam próximos de 90°) antes de energizar.

### 6.3 Trim e sinal dos servos
`SERVO_TRIM[6]` zera o nível de cada horn; `SERVO_SIGN[6]` corrige o sentido de
rotação de cada servo. `SERVO_MIN_DEG`/`MAX_DEG` definem o envelope.

### 6.4 Calibrações
- IMU: `IMU_OFFSET_ROLL`/`PITCH` (procedimento no `GUIA_DE_USO.md`, 9.2).
- Touch: `TS_CAL_AX/BX/DX/AY/BY/DY` (procedimento em 9.3).

### 6.5 Temporização
`LOOP_PERIOD_US`, `BALL_LOOP_DIV`, `SERIAL_PERIOD_MS`, `BNO_REPORT_US`. Lembre que
`BALL_LOOP_DIV` é divisor de ticks, não frequência direta; a frequência da malha
externa é 100 Hz / `BALL_LOOP_DIV`.

---

## 7. Como estender

### 7.1 Adicionar um modo de operação
1. `types.h`: acrescente o valor no fim do `enum OpMode` (no fim, para não
   deslocar os inteiros já usados na telemetria).
2. `supervisor.cpp`, `processCommand()`: adicione um ramo sob `MODE`. Cuidado com
   a ordem: o parser usa `indexOf` por substring, então nomes mais específicos
   devem ser testados antes (ex. `BALL_OPEN` antes de `OPEN` e de `BALL`).
3. `supervisor.cpp`, `tick()`: adicione um `case` no `switch` com o pipeline do
   modo. Use `IMU::isFresh()` se o modo depender de atitude.
4. `setMode()` já faz o reset genérico; adicione reset específico se necessário.
5. Telemetria: o inteiro do modo já é transmitido. Se o modo expõe um estado
   novo, ver 7.5.

### 7.2 Adicionar um comando serial
Em `processCommand()`, adicione `else if (line.startsWith("X"))`, parse com
`indexOf`/`substring`/`toFloat`. Comandos não reconhecidos caem no ramo final
que imprime `ERR: unknown command`. Mantenha o padrão dos comandos existentes
(`MODE`/`POSE`/`TARGET`/`TRIM`).

### 7.3 Alterar uma struct de dados
Edite `types.h`, depois atualize o módulo que produz o campo e os que o consomem.
Se o campo for telemetrado, atualize os três pontos descritos em 7.5. Como as
structs são passadas por referência e têm inicializadores padrão, acrescentar um
campo não quebra os chamadores que não o usam.

### 7.4 Adicionar um sensor ou atuador
Crie um par `meu_modulo.{h,cpp}` com um namespace próprio, `init()` e a função de
trabalho. Ponha os pinos e constantes em `config.h`. Chame `init()` em
`Supervisor::init()` e use o módulo no `tick()`. Não acesse o hardware fora do
módulo dele.

### 7.5 Alterar a telemetria
Três pontos precisam ficar em sincronia, na mesma ordem de colunas:
1. `supervisor.cpp`, `sendTelemetry()`: os `Serial.print` que emitem os dados.
2. `supervisor.cpp`, `sendTelemetryHeader()`: a linha de nomes das colunas.
3. `tools/telemetry_logger.py`, `EXPECTED_COLUMNS`: a contagem e os nomes que o
   logger usa (o logger valida pela contagem e mapeia pela ordem).

Se a contagem mudar e `EXPECTED_COLUMNS` não acompanhar, o logger descarta as
linhas como malformadas.

---

## 8. Inconsistências conhecidas

Divergências reais entre comentários e código nos arquivos atuais. Não afetam
necessariamente a operação, mas confundem quem lê. Registradas para serem
reconciliadas.

1. **Pinos do cabeçalho do `.ino` divergem de `config.h`.** O comentário de
   `StewartBallOnPlate.ino` lista PCA em I2C0 (SDA GP8, SCL GP9) e BNO com SCK
   GP10, MOSI GP11, INT GP14, RST GP15. Os valores compilados em `config.h` são
   PCA em SDA 18/SCL 19 e BNO com SCK 14, MOSI 15, INT 11, RST 10. `config.h` é o
   que vale. Atualizar o cabeçalho do `.ino`.

2. **`BALL_LOOP_DIV` vs comentários.** `BALL_LOOP_DIV = 3` em `config.h`, o que
   dá malha externa a ~33 Hz. Vários comentários (em `config.h` e em
   `supervisor.cpp`) afirmam "100 Hz / 5 = 20 Hz". Valor e comentário divergem.
   Decidir qual é o pretendido e alinhar o número e o texto.

3. **Nome e unidade das colunas 14–15 da telemetria.** O firmware grava o nome
   `cmd_r`/`cmd_p` no cabeçalho; `telemetry_logger.py` chama `corr_r`/`corr_p`. É
   a mesma coluna. O dado é o comando do controlador de atitude na forma de
   velocidade (graus/s), mas o gráfico do logger rotula o eixo como "PID
   correction (deg)". A unidade do rótulo está incorreta (é graus/s). A captura
   funciona porque o logger usa a ordem das colunas, não os nomes.

4. **Comando `GAINS` documentado mas inexistente.** O comentário do topo de
   `supervisor.cpp` lista `GAINS ATT ...` e `GAINS BALL ...`, mas
   `processCommand()` não trata `GAINS`. Enviá-lo retorna `ERR: unknown command`.

5. **`IDLE` não nivela os servos.** O comentário no `case IDLE` diz "Servos stay
   at neutral", mas nada reescreve os servos ao entrar em `IDLE`; eles mantêm o
   último PWM. O nivelamento só ocorre no boot, via `writeNeutral()` em `init()`.

6. **Ganhos integrais em zero.** `ATT_KI_ROLL/PITCH = 0` e `BALL_KI_* = 0` na
   build atual: as duas malhas operam como PD, apesar de descritas como PID.
   Itens correlatos: `deadband_punch` zerado em `attitude_ctrl.cpp` (inerte) e
   `ATT_DEADBAND = 0`.

7. **Blocos antigos comentados em `config.h`.** Geometria e ganhos `OLD`, além de
   alternativas comentadas em fim de linha. Não afetam a build; remover após
   consolidar reduz a chance de editar o valor errado.

---

## 9. Referência rápida de constantes

Valores atuais em `config.h`, para conferência. Sempre tratar `config.h` como a
fonte; esta tabela é um resumo.

| Constante | Valor | Significado |
|---|---|---|
| `LOOP_PERIOD_US` | 10000 | Período alvo do laço (100 Hz) |
| `BALL_LOOP_DIV` | 3 | Divisor da malha externa (~33 Hz) |
| `SERIAL_PERIOD_MS` | 20 | Período da telemetria (50 Hz) |
| `BNO_REPORT_US` | 10000 | Período dos relatórios do BNO085 (100 Hz) |
| `L1`, `L2` | 24.95, 160.0 | Horn e haste (mm) |
| `Z_HOME` | 149.33 | Altura neutra (mm) |
| `SERVO_MIN_DEG`/`MAX_DEG` | 15 / 165 | Envelope dos servos (graus) |
| `POSE_TILT_MAX` | 18 | Limite da inclinação integrada (graus) |
| `ATT_CMD_MAX` | 600 | Limite do comando de velocidade (graus/s) |
| `BALL_OUT_MAX` | 18 | Limite da saída da malha da bola (graus) |
| `BALL_TRANSITION_DIST` | 20 | Limiar de escalonamento de ganho (mm) |
| `IMU_TIMEOUT_MS` | 50 | Janela de frescor do IMU |
| `ATT_KP_ROLL`/`PITCH` | 9.2 / 8.7 | Ganho proporcional da atitude |
| `ATT_KD_ROLL`/`PITCH` | 0.48 / 0.38 | Ganho derivativo da atitude |
| `BALL_KP_CLOSE`/`FAR` | 0.068 / 0.068 | Ganho proporcional da bola |
| `BALL_KD_CLOSE`/`FAR` | 0.031 / 0.031 | Ganho derivativo da bola |
| `BALL_DEADBAND` | 3 | Zona morta da posição da bola (mm) |
| `PLATE_X_MAX`/`Y_MAX` | 310 / 236 | Extensão da placa (mm) |
