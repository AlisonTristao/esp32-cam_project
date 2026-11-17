# ESP32-CAM: imagem binaria no serial

A configuracao atual de camera.matrix, compressao RAW/RLE e threshold
adaptativo esta em [Compressao da camera](docs/camera-compression.md). As instrucoes de
saida ASCII abaixo descrevem a versao inicial do projeto.

Projeto PlatformIO com ESP-IDF para ESP32-CAM AI Thinker e OV2640.
A camera entrega grayscale em QQVGA (160 x 120), usando um framebuffer
na PSRAM. O programa amostra diretamente esse buffer e imprime `0` para
pixels escuros e `1` para pixels claros, sem alocar outra matriz.

## Configuracao

Altere as constantes no inicio de `src/main.c` e recompile:

```c
#define SAMPLE_STEP 4
#define BW_THRESHOLD 180
#define FRAME_DELAY_MS 1000
```

| SAMPLE_STEP | Colunas x linhas |
| --- | --- |
| 1 | 160 x 120 |
| 2 | 80 x 60 |
| 4 | 40 x 30 |
| 8 | 20 x 15 |

O passo deve ser inteiro e positivo. Se nao dividir a largura ou altura,
a quantidade de amostras naquele eixo e arredondada para cima.
Pixels com intensidade maior ou igual ao threshold viram `1`; os demais,
`0`. O threshold aceita valores inteiros de 0 a 255.

Cada frame tem o cabecalho `FRAME`, as dimensoes da matriz, as linhas sem
espacos e o separador `---`. Logs de inicializacao e de falhas podem aparecer
no monitor; eles nao fazem parte da matriz.

O intervalo configurado e aplicado depois de imprimir e devolver o frame.
Portanto, o periodo total inclui captura, transmissao serial e uma pausa
de um tick por linha para permitir a execucao das outras tarefas.
Com passo 1, a transmissao leva mais tempo por conter 19.200 digitos.

## Compilar e gravar

No terminal do PlatformIO, na pasta do projeto:

```powershell
pio run -e esp32cam
pio run -e esp32cam -t upload --upload-port COM5
pio device monitor -e esp32cam --port COM5
```

Substitua `COM5` pela porta da placa. O monitor usa 115200 baud.
Para gravacao com adaptador USB-serial, conecte GPIO0 ao GND e reinicie
a placa para entrar no bootloader. Depois de gravar, remova essa ligacao
e reinicie para executar a aplicacao; GPIO0 e usado como XCLK da camera.

A dependencia `espressif/esp32-camera` 2.1.7 e baixada pelo gerenciador de
componentes do ESP-IDF na primeira compilacao. A PSRAM esta habilitada
em `sdkconfig.esp32cam` e em `sdkconfig.defaults` para novas configuracoes.

## Validacao na placa

Se o monitor ficar vazio, confirme que o Upload terminou com sucesso,
selecione a porta da ESP32-CAM a 115200 baud, remova GPIO0 do GND e
pressione RESET com o monitor aberto. `BOOT: aplicacao ESP32-CAM iniciada`
indica que o firmware chegou a `app_main`. Em seguida aparece o progresso
da inicializacao da camera. Falhas de inicializacao sao repetidas a cada
2 segundos; nesse estado, corrija a causa e reinicie a placa.
O monitor foi configurado com DTR e RTS desativados.

Compilacao verificada com `pio run -e esp32cam`, PlatformIO Espressif32
7.0.1 e ESP-IDF 6.0.1. Firmware gerado em `.pio/build/esp32cam/firmware.bin`.
A captura fisica e a saida serial ainda precisam ser verificadas na placa:

1. Confira `40 x 30` no cabecalho com a configuracao inicial.
2. Aponte para regioes claras e escuras e observe a mudanca dos digitos.
3. Altere o threshold para ajustar a resposta a iluminacao.
4. Altere o passo para 8, recompile e confira `20 x 15`.

Nao ha deteccao de linha, centroide ou calculo de posicao nesta etapa.

## Referencias

- [Driver esp32-camera da Espressif](https://components.espressif.com/components/espressif/esp32-camera/versions/2.1.7/readme)
- [Pinagem AI Thinker no exemplo da Espressif](https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h)
