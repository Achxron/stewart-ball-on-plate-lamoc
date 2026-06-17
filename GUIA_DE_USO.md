# Guia de uso — Stewart Ball-on-Plate

Como operar o firmware do laboratório: compilar, gravar, enviar comandos, ler a
telemetria e capturar dados para análise. Voltado a quem vai usar o sistema, não
necessariamente alterá-lo. Para a descrição interna dos módulos e como modificar
o código, ver `MANUAL_TECNICO.md`.

Todo o conteúdo deste guia descreve o comportamento real do código nos arquivos
fornecidos. Onde um comentário do código diverge do que o código faz, o guia
segue o código e aponta a divergência.

---

## 1. O que é o sistema

Plataforma de Stewart de 6 graus de liberdade, atuada por seis servos rotativos
(horn + haste de comprimento fixo), controlando uma bola sobre uma placa
resistiva. O microcontrolador é um Raspberry Pi Pico (RP2040). A arquitetura é em
cascata: uma malha externa (posição da bola) gera uma inclinação desejada da
placa, e uma malha interna (atitude, com IMU) aplica essa inclinação corrigindo
distúrbios. A cinemática inversa converte a pose em seis ângulos de servo.

---

## 2. Requisitos

### Hardware
- Raspberry Pi Pico (RP2040).
- Driver de servo PCA9685 (I2C), seis servos MG90D.
- IMU BNO085 (SPI1).
- Touchscreen resistivo via SN74HC4066N.
- Fonte 5 V capaz de suprir os seis servos.

Os pinos efetivos estão em `config.h` (que é a fonte única de verdade). O
cabeçalho de `StewartBallOnPlate.ino` lista pinos diferentes e está
desatualizado: ver `MANUAL_TECNICO.md`, seção "Inconsistências conhecidas".

### Firmware
- Núcleo RP2040 para Arduino. O código usa `MbedSPI` e o construtor de `TwoWire`
  com pinos, então o núcleo instalado precisa fornecer essas classes. Use o
  mesmo núcleo já configurado na máquina do laboratório; confirme que ele compila
  `MbedSPI`.
- Bibliotecas (instalar pelo gerenciador de bibliotecas):
  - Adafruit PWM Servo Driver Library
  - SparkFun BNO08x Arduino Library
- `SPI` e `Wire` acompanham o núcleo.

### Análise de dados (opcional, só para a ferramenta de captura)
- Python 3, `pyserial` (obrigatório para capturar).
- `matplotlib`, `numpy`, `pandas` (só para gráficos e estatísticas).

```
pip install pyserial matplotlib numpy pandas
```

---

## 3. Compilar e gravar

1. Abra `StewartBallOnPlate.ino` no Arduino IDE. Os arquivos `.h`/`.cpp` devem
   estar na mesma pasta do `.ino` (a estrutura plana exigida pelo Arduino).
2. Selecione a placa Raspberry Pi Pico / RP2040 no núcleo instalado.
3. Compile e grave pela USB.

A pasta `tools/` (com o `telemetry_logger.py` e os scripts) fica ao lado do
sketch e não é compilada: o Arduino compila apenas a raiz do sketch e a subpasta
`src/`. Manter `tools/` ali é seguro.

---

## 4. Inicialização

Ao energizar ou reconectar, o firmware abre a serial a 115200 baud e imprime o
diagnóstico de boot, na ordem:

```
=== Stewart Ball-on-Plate ===
[OK] Kinematics
[OK] Servos (neutral)
[OK] IMU (BNO085)        (ou [FAIL] IMU — check wiring)
[OK] Touchscreen
MODE → 0
time_ms,mode,bx,by,...   (linha de cabeçalho da telemetria)
Ready. Send MODE command to begin.
```

No boot os servos vão para o neutro (90° + trim de cada servo). O sistema entra
em `IDLE` e aguarda um comando `MODE`. Se aparecer `[FAIL] IMU`, verifique a
fiação SPI antes de usar qualquer modo que dependa de atitude (`ATT`, `CHICKEN`,
`BALL`).

---

## 5. Comandos seriais

Linha de texto terminada em `\n` (o firmware ignora `\r`, então finais de linha
do Windows funcionam). Todos os comandos são maiúsculos. Ângulos em graus,
posições da bola em milímetros.

| Comando | Efeito | Onde tem efeito |
|---|---|---|
| `MODE IDLE` | Para o controle; sensores seguem atualizando | sempre |
| `MODE OPEN` | Pose direta por serial, sem realimentação | malha aberta |
| `MODE ATT <r> <p>` | Segura a atitude `<r> <p>` no referencial do mundo | malha interna |
| `MODE CHICKEN` | Mantém a placa nivelada rejeitando distúrbios da base | malha interna |
| `MODE BALL` | Balanceamento completo da bola (cascata) | cascata |
| `MODE BALL_OPEN` | Bola → cinemática → servos, sem realimentação de IMU | cascata sem IMU |
| `MODE CAL_IMU` | Transmite telemetria para medir offset do IMU | calibração |
| `MODE CAL_TOUCH` | Transmite telemetria para calibrar o touch | calibração |
| `POSE <r> <p>` | Define inclinação desejada (roll, pitch) | `OPEN` e `ATT` |
| `TARGET <x> <y>` | Define o alvo da bola (mm, referencial da placa) | `BALL` e `BALL_OPEN` |
| `TRIM <r> <p>` | Bias de nível aplicado a roll/pitch | todos os modos de pose |

Notas de uso:

- Em `MODE ATT <r> <p>`, os dois números são opcionais; sem eles, mantém o último
  `POSE`/`ATT` recebido.
- `POSE` só altera a inclinação em `OPEN` e `ATT`. Em `CHICKEN` e `BALL` a
  referência da malha interna é definida internamente (nível, ou saída da malha
  da bola), então `POSE` é ignorado.
- `TARGET` usa o referencial da placa: X em 0 a 310 mm, Y em 0 a 236 mm, centro
  em (155, 118).
- `TRIM` soma um bias constante a roll e pitch. Serve para compensar um
  desnível mecânico residual sem mexer no código.
- O comando `GAINS` aparece no comentário do topo de `supervisor.cpp`, mas **não
  está implementado**. Enviá-lo retorna `ERR: unknown command`. Para mudar
  ganhos, edite `config.h` e recompile.

Comandos desconhecidos retornam `ERR: unknown command` seguido da lista de
comandos válidos.

---

## 6. Modos de operação

A coluna "mode" da telemetria carrega o inteiro do modo (definido pela ordem do
`enum OpMode` em `types.h`):

| nº | Modo | O que faz | IMU | Touch |
|---|---|---|---|---|
| 0 | `IDLE` | Sem controle. Servos mantêm o último PWM | lê | não lê |
| 1 | `OPEN_LOOP_POSE` | Pose = trim + `POSE` → cinemática → servos | lê | lê (só telemetria) |
| 2 | `ATTITUDE_HOLD` | Segura atitude = trim + `POSE` (malha interna) | exige fresco | lê (só telemetria) |
| 3 | `CHICKEN_HEAD` | Segura nível = trim (rejeita distúrbio da base) | exige fresco | não lê |
| 4 | `BALL_BALANCE` | Cascata completa, com tratamento de perda de IMU | exige fresco | lê |
| 5 | `CALIBRATE_IMU` | Só transmite telemetria, sem controle | lê | não lê |
| 6 | `CALIBRATE_TOUCH` | Atualiza e transmite o touch, sem controle | lê | lê |
| 7 | `BALL_OPEN` | Bola → cinemática → servos, sem realimentação de IMU | ignora | lê |

Comportamento na perda de IMU:

- `ATT` e `CHICKEN`: se o IMU não estiver fresco (sem dado novo dentro de
  `IMU_TIMEOUT_MS`), a plataforma congela na última pose válida e os integradores
  da atitude são zerados (evita windup sobre medida parada). Ao voltar o dado, o
  controle retoma.
- `BALL`: usa uma máquina de três fases. `CLOSED` é a cascata normal. Se o IMU
  cai, entra em `OPEN` (malha aberta, segura a última inclinação da bola). Quando
  o IMU volta, entra em `RELEVELING`: a malha da bola fica suspensa e a pose é
  rampada de volta ao nível a `RELEVEL_SLEW_DPS`, devolvendo o controle à bola só
  depois que a atitude estabiliza nivelada. Isso evita a varredura brusca que
  poderia derrubar o sensor de novo.

Atenção operacional: ao sair de um modo de controle para `IDLE`, os servos **não**
voltam ao neutro automaticamente; mantêm o último PWM comandado. O nivelamento só
acontece no boot. Para nivelar manualmente, use `MODE OPEN` e `POSE 0 0` (com o
`TRIM` adequado).

---

## 7. Telemetria

O firmware transmite CSV pela serial, no máximo a cada `SERIAL_PERIOD_MS` (20 ms,
ou seja, até 50 Hz). A linha de cabeçalho é reemitida a cada troca de modo. A
ferramenta de captura trata linhas de cabeçalho como informação e usa a ordem das
colunas, então a reemissão não atrapalha a gravação.

São 23 colunas, nesta ordem:

| # | Coluna | Significado | Unidade |
|---|---|---|---|
| 1 | `time_ms` | Tempo do firmware (`millis()`) | ms |
| 2 | `mode` | Inteiro do modo atual | — |
| 3 | `bx` | Posição X da bola | mm |
| 4 | `by` | Posição Y da bola | mm |
| 5 | `bvx` | Velocidade X estimada | mm/s |
| 6 | `bvy` | Velocidade Y estimada | mm/s |
| 7 | `bdet` | Bola detectada (0/1) | — |
| 8 | `imu_r` | Roll medido | graus |
| 9 | `imu_p` | Pitch medido | graus |
| 10 | `gyro_r` | Taxa de roll (giroscópio) | graus/s |
| 11 | `gyro_p` | Taxa de pitch (giroscópio) | graus/s |
| 12 | `ref_r` | Referência de roll da malha interna | graus |
| 13 | `ref_p` | Referência de pitch da malha interna | graus |
| 14 | `cmd_r` | Comando do controlador de atitude (roll) | graus/s |
| 15 | `cmd_p` | Comando do controlador de atitude (pitch) | graus/s |
| 16 | `pose_r` | Pose de roll enviada à cinemática | graus |
| 17 | `pose_p` | Pose de pitch enviada à cinemática | graus |
| 18–23 | `s1`..`s6` | Ângulos dos seis servos | graus |

Duas ressalvas importantes sobre as colunas 14 e 15:

- O firmware grava o nome `cmd_r`/`cmd_p` no cabeçalho; o `telemetry_logger.py`
  rotula a mesma coluna como `corr_r`/`corr_p`. É a mesma coluna, com o mesmo
  dado.
- O dado nessas colunas é o comando do controlador de atitude na forma de
  **velocidade** (graus/s, integrada depois em pose). O gráfico gerado pelo
  logger rotula o eixo como "PID correction (deg)", mas a unidade real é graus/s.
  Trate o eixo como graus/s ao interpretar.

A referência `ref_r`/`ref_p` é o que a malha interna persegue: em `ATT` é
trim + `POSE`; em `CHICKEN` é só trim (nível); em `BALL` é trim + saída da malha
da bola (varia no tempo).

---

## 8. Ferramenta de captura (`tools/telemetry_logger.py`)

Captura a telemetria pela USB, salva em CSV com data/hora no nome, opcionalmente
envia comandos e gera gráficos. O arquivo fica em `tools/`; os scripts de
sequência ficam em `tools/scripts/`. Os exemplos abaixo assumem execução a partir
da raiz do projeto.

### Três modos

1. Só capturar, por tempo fixo:

```
python3 tools/telemetry_logger.py --port /dev/ttyACM0 --name idle --duration 30
```

2. Interativo (digitar comandos enquanto grava):

```
python3 tools/telemetry_logger.py --port /dev/ttyACM0 --name tuning --interactive
```
Digite comandos (`MODE ATT`, `POSE 5 0`, `POSE 0 0`) e pressione Enter; Ctrl-C
encerra. Os comandos enviados são registrados num arquivo `.commands.txt` para
rastreabilidade.

3. Roteirizado (sequência temporizada, reproduzível):

```
python3 tools/telemetry_logger.py --port /dev/ttyACM0 --name step_roll5 \
    --script tools/scripts/step_roll5.txt
```

### Formato do script

Texto puro, uma linha por evento: `<segundos_desde_o_início>  <comando>`.
Linhas com `#` são comentário. `END` encerra a captura.

```
# tools/scripts/step_roll5.txt
0.0  MODE ATT
1.0  POSE 0 0
3.0  POSE 5 0
8.0  POSE 0 0
12.0 END
```

### Replotar uma captura existente

```
python3 tools/telemetry_logger.py --plot logs/step_roll5_2026-04-20_14-33.csv
```

### Saída

- `logs/<nome>_<data-hora>.csv`: a telemetria.
- `logs/<nome>_<data-hora>.commands.txt`: comandos enviados, com o tempo.
- `logs/<nome>_<data-hora>.png`: gráfico (roll, pitch, comando do controlador,
  ângulos dos servos), salvo após a captura, salvo `--no-plot`.
- Estatísticas rápidas no terminal, por eixo: média, desvio padrão, pico a pico e
  RMS do erro `ref - medido`.

Os logs vão para `./logs` por padrão (relativo à pasta de execução). Rode a partir
da raiz, ou passe `--log-dir <pasta>`.

### Opções principais

| Opção | Padrão | Função |
|---|---|---|
| `--port` | — | Porta serial (ex. `/dev/ttyACM0`, `COM4`). Obrigatória ao capturar |
| `--baud` | 115200 | Velocidade da serial |
| `--name` | `run` | Nome usado no arquivo de saída |
| `--duration` | sem limite | Captura por N segundos e para |
| `--log-dir` | `logs` | Pasta dos CSV |
| `--interactive`, `-i` | — | Permite digitar comandos durante a captura |
| `--script` | — | Roda um script temporizado |
| `--plot` | — | Só plota um CSV existente (não captura) |
| `--no-plot` | — | Não gera gráfico após a captura |

---

## 9. Procedimentos reproduzíveis

Os procedimentos abaixo declaram cada passo para que possam ser repetidos.

### 9.1 Resposta ao degrau da malha de atitude (roll)

Objetivo: medir tempo de subida, sobressinal, tempo de acomodação e erro em regime
da malha interna, num degrau de referência.

1. Crie `tools/scripts/step_roll5.txt` com o conteúdo do exemplo da seção 8.
2. Energize a plataforma e aguarde `Ready.` na serial.
3. Confirme que o IMU subiu (`[OK] IMU`). Sem IMU fresco, `ATT` não controla.
4. Rode:
   ```
   python3 tools/telemetry_logger.py --port /dev/ttyACM0 --name step_roll5 \
       --script tools/scripts/step_roll5.txt
   ```
5. O CSV terá o degrau de `ref_r` (de 0 para 5 graus em t ≈ 3 s) e a resposta em
   `imu_r`. As métricas são extraídas de `imu_r` contra `ref_r`.
6. Para pitch, troque `POSE 5 0` por `POSE 0 5` e renomeie a saída.

Observação: a captura roteirizada é determinística nos instantes de comando, mas a
amostragem da telemetria não é uniforme (até 50 Hz, sujeita à carga do laço). Use
os carimbos `time_ms` reais ao calcular métricas temporais, não o índice da
amostra.

### 9.2 Calibração do offset do IMU

Objetivo: medir o desvio de montagem do IMU no zero geométrico, para preencher
`IMU_OFFSET_ROLL` e `IMU_OFFSET_PITCH` em `config.h`.

1. Coloque a placa no nível mecânico de referência (zero geométrico real).
2. Garanta que `IMU_OFFSET_ROLL` e `IMU_OFFSET_PITCH` estejam em 0 na build atual,
   para medir o offset bruto. (Se não estiverem, some o offset atual à leitura.)
3. `MODE CAL_IMU`. Capture com tempo fixo:
   ```
   python3 tools/telemetry_logger.py --port /dev/ttyACM0 --name cal_imu --duration 30
   ```
4. Calcule a média de `imu_r` e `imu_p` no intervalo. O offset a aplicar é o
   negativo dessas médias.
5. Escreva os valores em `config.h`, recompile e repita para confirmar que
   `imu_r` e `imu_p` ficam próximos de zero no nível.

### 9.3 Calibração do touchscreen

Objetivo: levantar a transformada afim ADC → mm (coeficientes `TS_CAL_*` em
`config.h`).

1. `MODE CAL_TOUCH`.
2. Capture lendo `bx`/`by` enquanto posiciona a bola (ou um ponteiro) em
   posições conhecidas da placa, em milímetros medidos.
   ```
   python3 tools/telemetry_logger.py --port /dev/ttyACM0 --name cal_touch --interactive
   ```
3. Para obter o ADC bruto em vez do valor calibrado, é preciso transmitir o ADC.
   Hoje `touch_sensor.cpp` aplica a calibração antes de transmitir; o trecho que
   transmitiria o ADC bruto está comentado. Ver `MANUAL_TECNICO.md`, seção do
   módulo de touch, para habilitar a saída bruta durante a calibração.
4. Ajuste os seis coeficientes (`TS_CAL_AX`, `BX`, `DX`, `AY`, `BY`, `DY`) por
   mínimos quadrados sobre os pares (ADC, mm medido), escreva em `config.h` e
   recompile.

---

## 10. Segurança operacional

- Os servos têm envelope limitado a [`SERVO_MIN_DEG`, `SERVO_MAX_DEG`] (15° a
  165°) e a cinemática rejeita poses fora do espaço de trabalho. A inclinação
  integrada é limitada a `POSE_TILT_MAX` (18°).
- O comando de velocidade da malha interna é limitado a `ATT_CMD_MAX` (600
  graus/s). A saída da malha da bola é limitada a `BALL_OUT_MAX` (18°).
- Sob movimento violento o BNO085 pode reiniciar. A recuperação é não bloqueante
  (máquina de estados em `imu_sensor.cpp`); os modos de malha aberta continuam
  rodando durante a recuperação.
- Antes de habilitar `BALL`, verifique o nível com `TRIM` em `CHICKEN`, para que a
  placa fique de fato nivelada quando a malha da bola assumir.
