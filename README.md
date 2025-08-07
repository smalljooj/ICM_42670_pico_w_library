# Documentação do Driver ICM-42670 IMU

## Autor

Ismael Marinho Rocha

Este documento fornece uma visão geral abrangente do driver para a Unidade de Medição Inercial (IMU) de 6 eixos TDK ICM-42670, projetado para uso com o Raspberry Pi Pico através de comunicação SPI. O driver inclui funções para inicialização, configuração, leitura de sensores e um filtro de Kalman para fusão de sensores, permitindo o cálculo de ângulos de inclinação (pitch) e rotação (roll) estáveis.

## Índice
- [Visão Geral](#visão-geral)
- [Arquivos](#arquivos)
- [Inicialização e Configuração](#inicialização-e-configuração)
- [Leitura de Dados dos Sensores](#leitura-de-dados-dos-sensores)
- [Filtro de Kalman para Orientação](#filtro-de-kalman-para-orientação)
- [Recursos de Movimento APEX](#recursos-de-movimento-apex)
- [Acesso a Registradores de Baixo Nível](#acesso-a-registradores-de-baixo-nível)
- [Estruturas de Dados e Enums](#estruturas-de-dados-e-enums)
- [Configuração de Hardware](#configuração-de-hardware)
- [Exemplo de Uso](#exemplo-de-uso)

## Visão Geral
O driver ICM-42670 oferece uma API pública para interação com o TDK ICM-42670, uma IMU de 6 eixos que inclui um acelerômetro de 3 eixos, um giroscópio de 3 eixos, um sensor de temperatura e um motor de movimento avançado (APEX). O driver é implementado em C e utiliza a interface SPI do Raspberry Pi Pico. Ele suporta:

- Inicialização e configuração dos sensores e recursos da IMU.
- Leitura de dados brutos do acelerômetro, giroscópio e sensor de temperatura.
- Fusão de sensores usando um filtro de Kalman para estimar ângulos de inclinação e rotação.
- Acesso a recursos APEX, como contagem de passos e detecção de atividade.
- Operações de leitura/escrita de registradores de baixo nível para controle detalhado.

## Arquivos
O driver é composto pelos seguintes arquivos:
- **icm_42670.h**: Arquivo de cabeçalho da API pública com protótipos de funções para inicialização, configuração, leitura de sensores e operações do filtro de Kalman.
- **icm_42670_registers.h**: Define os endereços dos registradores para User Bank 0 e Regiões de Memória (MREG1, MREG2, MREG3).
- **icm_42670_types.h**: Define estruturas de dados e enums para configuração de sensores, armazenamento de dados e parâmetros do filtro de Kalman.
- **icm_42670.c**: Implementação das funções do driver, incluindo comunicação SPI, leitura de sensores e algoritmos do filtro de Kalman.

## Inicialização e Configuração
O driver fornece funções para inicializar e configurar o sensor ICM-42670 e sua interface SPI.

### Funções
- **init_spi()**: Inicializa o periférico SPI (400 kHz) e configura os pinos GPIO para MISO (16), MOSI (19), SCK (18) e CS (10, ativo em nível baixo).
- **init_icm_42670(icm_42670_t* init_struct)**: Inicializa a IMU com as configurações fornecidas na estrutura `icm_42670_t`. Realiza um reset suave, aguarda a prontidão do dispositivo e configura gerenciamento de energia, giroscópio, acelerômetro e filtros. Retorna 1 em caso de sucesso, 0 em caso de falha por timeout.
- **icm_42670_status()**: Imprime valores de registradores-chave (ex.: `MCLK_RDY`, `PWR_MGMT0`, `WHO_AM_I`) para depuração.
- **icm_42670_interrupt_config(icm_42670_int_t* config)**: Configura os pinos de interrupção INT1 e INT2 (modo, polaridade, circuito de acionamento).
- **icm_42670_fifo_config(icm_42670_fifo_t* config)**: Configura o buffer FIFO (modo, bypass).
- **icm_42670_apex_config(icm_42670_apex_t* config)**: Configura os recursos APEX (ex.: pedômetro, detecção de inclinação, configurações do DMP).

## Leitura de Dados dos Sensores
O driver suporta a leitura de dados brutos e processados do acelerômetro, giroscópio e sensor de temperatura.

### Funções
- **icm_42670_read_all_sensors(icm_42670_all_sensors_data_t* data)**: Lê dados brutos de todos os sensores (acelerômetro, giroscópio, temperatura) em uma estrutura `icm_42670_all_sensors_data_t`.
- **icm_42670_read_temperature_celsius()**: Lê a temperatura em Celsius usando a fórmula: `Temp_C = (RAW_TEMP / 132.48) + 25`.
- **icm_42670_read_temperature_kelvin()**: Converte Celsius para Kelvin (`Celsius + 273.15`).
- **icm_42670_read_temperature_fahrenheit()**: Converte Celsius para Fahrenheit (`Celsius * 1.8 + 32`).
- **icm_42670_read_gyro(icm_42670_gyro_data_t* data)**: Lê dados brutos do giroscópio (eixos x, y, z).
- **icm_42670_read_accel(icm_42670_accel_data_t* data)**: Lê dados brutos do acelerômetro (eixos x, y, z).

## Filtro de Kalman para Orientação
O driver inclui uma implementação de filtro de Kalman para fusão de dados do acelerômetro e giroscópio, gerando estimativas estáveis de ângulos de inclinação (pitch) e rotação (roll).

### Funções
- **icm_42670_kalman_init(icm_42670_kalman_t* kf, double Q_angle, double Q_bias, double R_measure)**: Inicializa um filtro de Kalman com parâmetros de ajuste para ruído do processo (`Q_angle`, `Q_bias`) e ruído de medição (`R_measure`).
- **icm_42670_kalman_init_struct()**: Inicializa os filtros globais de pitch e roll com valores de ajuste predefinidos (`Q_angle=0.001`, `Q_bias=0.003`, `R_measure=0.03`), se ainda não inicializados.
- **icm_42670_kalman_update()**: Executa um ciclo de atualização para os filtros de pitch e roll:
  - Lê dados brutos dos sensores.
  - Converte para unidades físicas (acelerômetro: g, giroscópio: deg/s, assumindo 16g e 2000 dps).
  - Calcula ângulos com base no acelerômetro usando `atan2`.
  - Atualiza os filtros com o delta de tempo (`dt`) e dados dos sensores.
- **icm_42670_kalman_get_angle(icm_42670_kalman_t* kf, double newAngle, double newRate, double dt)**: Executa um passo do filtro de Kalman, fundindo o ângulo do acelerômetro com a taxa do giroscópio para uma estimativa suavizada.
- **icm_42670_kalman_get_angles(icm_42670_angles_data_t* angles)**: Obtém os ângulos atuais de pitch e roll dos filtros globais.
- **icm_42670_kalman_get_angles_autoupdate(icm_42670_angles_data_t* angles)**: Executa um ciclo de atualização e obtém os novos ângulos.

### Notas sobre o Filtro de Kalman
- O filtro de Kalman usa instâncias globais (`pitch_filter`, `roll_filter`) para rastrear ângulos de pitch e roll.
- Fatores de conversão assumem configurações de escala total de 2000 dps (giroscópio, 16.4 LSB/dps) e 16g (acelerômetro, 2048 LSB/g).
- O delta de tempo (`dt`) é calculado usando `absolute_time_t` do Pico em segundos.

## Recursos de Movimento APEX
O motor de movimento APEX oferece funcionalidades como contagem de passos e detecção de atividade (caminhada, corrida).

### Funções
- **icm_42670_dmp_is_running()**: Retorna 1 se o Processador de Movimento Digital (DMP) está ativo, 0 caso contrário (verifica o bit DMP_IDLE em `APEX_DATA3`).
- **icm_42670_apex_step_count()**: Lê a contagem total de passos de `APEX_DATA0` e `APEX_DATA1` (16 bits).
- **icm_42670_apex_step_cadence()**: Lê a cadência de passos de `APEX_DATA2` (unidade: 2 passos/s, 0 indica <0.5 Hz).
- **icm_42670_apex_pedometer_activity()**: Lê o status da atividade de `APEX_DATA3` (0: Desconhecido, 1: Caminhada, 2: Corrida).

## Acesso a Registradores de Baixo Nível
O driver fornece funções para acesso direto aos registradores em User Bank 0 e Regiões de Memória (MREG1, MREG2, MREG3).

### Funções
- **icm_42670_read_bank0_register(uint8_t reg)**: Lê um byte de um registrador em User Bank 0.
- **icm_42670_write_bank0_register(uint8_t reg, uint8_t data)**: Escreve um byte em um registrador em User Bank 0.
- **icm_42670_read_bank0_register_16(uint8_t reg)**: Lê um valor de 16 bits de dois registradores consecutivos em User Bank 0.
- **icm_42670_read_mreg1_register(uint8_t reg)**: Lê um registrador de MREG1 usando `BLK_SEL_R` (0x00) e `MADDR_R`.
- **icm_42670_write_mreg1_register(uint8_t reg, uint8_t data)**: Escreve em um registrador em MREG1 usando `BLK_SEL_W` (0x00) e `MADDR_W`.
- **icm_42670_read_mreg2_register(uint8_t reg)**: Lê um registrador de MREG2 usando `BLK_SEL_R` (0x28).
- **icm_42670_write_mreg2_register(uint8_t reg, uint8_t data)**: Escreve em um registrador em MREG2 usando `BLK_SEL_W` (0x28).
- **icm_42670_read_mreg3_register(uint8_t reg)**: Lê um registrador de MREG3 usando `BLK_SEL_R` (0x50).
- **icm_42670_write_mreg3_register(uint8_t reg, uint8_t data)**: Escreve em um registrador em MREG3 usando `BLK_SEL_W` (0x50).

### Notas
- O acesso aos registradores usa SPI com o pino CS (10) alternado para cada transação.
- Um atraso de 10µs é adicionado após configurar endereços de regiões de memória para garantir operação confiável.

## Estruturas de Dados e Enums
O arquivo `icm_42670_types.h` define estruturas e enums para configuração e manipulação de dados.

### Estruturas de Dados
- **icm_42670_kalman_t**: Armazena o estado do filtro de Kalman (ângulo, viés, taxa, matriz de covariância) e parâmetros de ajuste (`Q_angle`, `Q_bias`, `R_measure`).
- **icm_42670_gyro_data_t**: Armazena dados brutos do giroscópio (`gx`, `gy`, `gz` em floats).
- **icm_42670_accel_data_t**: Armazena dados brutos do acelerômetro (`ax`, `ay`, `az` em floats).
- **icm_42670_angles_data_t**: Armazena ângulos filtrados de pitch e roll (`pitch`, `roll` em floats).
- **icm_42670_all_sensors_data_t**: Armazena dados brutos de todos os sensores (`ax`, `ay`, `az`, `gx`, `gy`, `gz`, `temperature`).
- **icm_42670_t**: Agrupa configurações principais dos sensores (modos de giroscópio/acelerômetro, escalas totais, ODRs, larguras de banda de filtros, etc.).
- **icm_42670_int_t**: Agrupa configurações dos pinos de interrupção (modo, circuito de acionamento, polaridade para INT1/INT2).
- **icm_42670_apex_t**: Agrupa configurações APEX (DMP, pedômetro, inclinação, queda livre, etc.).
- **icm_42670_fifo_t**: Agrupa configurações do FIFO (modo, bypass).

### Enums
- **GYRO_MODE**: `GYRO_OFF`, `GYRO_STANDBY`, `GYRO_LN_MODE`.
- **ACCEL_MODE**: `ACCEL_OFF`, `ACCEL_LP_MODE`, `ACCEL_LN_MODE`.
- **IDLE**: `IDLE_OFF`, `IDLE_ON`.
- **ACCEL_LP_CLK_SEL**: `WAKE_UP_OSCILLATOR`, `RC_OSCILLATOR`.
- **GYRO_UI_FS_SEL**: Escalas totais (2000, 1000, 500, 250 dps).
- **GYRO_ODR**: Taxas de saída de dados (1600 Hz a 12.5 Hz).
- **ACCEL_UI_FS_SEL**: Escalas totais (16g, 8g, 4g, 2g).
- **ACCEL_ODR**: Taxas de saída de dados (1600 Hz a 1.5625 Hz).
- **TEMP_FILT_BW**: Larguras de banda do filtro de temperatura (bypass a 4 Hz).
- **GYRO_UI_FILT_BW**: Larguras de banda do filtro do giroscópio (bypass a 16 Hz).
- **ACCEL_UI_AVG**: Configurações de média do acelerômetro (2x a 64x).
- **ACCEL_UI_FILT_BW**: Larguras de banda do filtro do acelerômetro (bypass a 16 Hz).
- **INT_MODE**: `INT_MODE_PULSED`, `INT_MODE_LATCHED`.
- **INT_DRIVE_CIRCUIT**: `OPEN_DRAIN`, `PUSH_PULL`.
- **INT_POLARITY**: `ACTIVE_LOW`, `ACTIVE_HIGH`.
- **FIFO_MODE**: `FIFO_MODE_STREAM`, `FIFO_MODE_STOP_FULL`.
- **FIFO_BYPASS**: `FIFO_NOT_BYPASSED`, `FIFO_BYPASSED`.
- **Enums APEX**: `DMP_POWER_SAVE`, `DMP_INIT`, `DMP_MEM_RESET`, `SMD_ENABLE`, `FF_ENABLE`, `TILT_ENABLE`, `PED_ENABLE`, `DMP_ODR`, `DMP_IDLE_STATE`, `PEDOMETER_ACTIVITY`.

## Configuração de Hardware
O driver está configurado para o Raspberry Pi Pico com as seguintes atribuições de pinos SPI:
- **Porta SPI**: `spi0` (400 kHz).
- **MISO**: Pino 16.
- **MOSI**: Pino 19.
- **SCK**: Pino 18.
- **CS**: Pino 10 (ativo em nível baixo, I/O padrão).

## Exemplo de Uso
Abaixo está um exemplo de como usar o driver para inicializar a IMU, configurar sensores e ler ângulos de orientação filtrados.

```c
#include "icm_42670.h"

int main() {
    stdio_init_all(); // Inicializa I/O padrão
    init_spi();       // Inicializa SPI

    // Configura as definições da IMU
    icm_42670_t config = {
        .gyro_mode = GYRO_LN_MODE,
        .accel_mode = ACCEL_LN_MODE,
        .idle = IDLE_OFF,
        .accel_lp_clk_sel = WAKE_UP_OSCILLATOR,
        .gyro_ui_fs_sel = GYRO_UI_2000_DPS,
        .gyro_odr = GYRO_ODR_200HZ,
        .accel_ui_fs_sel = ACCEL_UI_16_G,
        .accel_odr = ACCEL_ODR_200HZ,
        .temp_filt_bw = TEMP_FILT_4_HZ,
        .gyro_ui_filt_bw = GYRO_FILT_25_HZ,
        .accel_ui_avg = ACCEL_AVG_8X,
        .accel_ui_filt_bw = ACCEL_FILT_25_HZ
    };

    // Inicializa a IMU
    if (!init_icm_42670(&config)) {
        printf("Falha na inicialização da IMU!\n");
        return 1;
    }

    // Imprime status para depuração
    icm_42670_status();

    // Loop principal: lê e imprime ângulos de pitch e roll
    icm_42670_angles_data_t angles;
    while (true) {
        icm_42670_kalman_get_angles_autoupdate(&angles);
        printf("Pitch: %.2f, Roll: %.2f\n", angles.pitch, angles.roll);
        sleep_ms(100); // Atualiza a cada 100ms
    }

    return 0;
}