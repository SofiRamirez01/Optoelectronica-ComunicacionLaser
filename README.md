# OPTOELECTRONICA

Proyecto de comunicación óptica por láser/FSO entre dos ESP32. El sistema incluye un transmisor (`main_tx.cpp`) que genera telemetría simulada en formato binario Manchester y un receptor (`main_rx.cpp`) que decodifica la señal recibida con un sensor LDR y ofrece un dashboard web.

## Descripción general

- `main_tx.cpp`: Nodo transmisor.
  - Usa `PIN_TX2 = 17` para modular la señal Manchester a 20 bps.
  - Genera un paquete binario fijo con:
    - `seq`, `ts`
    - lecturas simuladas de cuatro sensores `F0`..`F3`
    - errores de azimut/elevación `errAz`, `errEl`
    - activación de motores `motAz`, `motEl`
    - tensión `vdc`, corriente `idc` y potencia `pw`
  - Envía la trama con un preámbulo de 4 bytes `0x55`, byte de sincronización `0x7E`, longitud de payload y CRC32.
  - Incluye un botón en `PIN_BUTTON = 4` para cambiar entre:
    - modo normal: transmisión periódica cada 20 segundos
    - modo apuntado: láser fijo y transmisión pausada.

- `main_rx.cpp`: Nodo receptor.
  - Lee el sensor LDR en `PIN_ADC_LDR = 34` para detectar el haz láser.
  - Decodifica Manchester de bajo nivel a partir de los flancos del ADC.
  - Reconstruye tramas con preámbulo `0x55`, sync `0x7E`, longitud y payload + CRC32.
  - Valida CRC y muestra métricas de enlace:
    - tramas correctas, errores CRC, errores de trama, saltos de secuencia.
  - Usa un botón en `PIN_BUTTON = 14` para alternar entre:
    - modo normal: decodificación activa y estado de enlace
    - modo direccionamiento: LED azul indica si el haz incide en el LDR.
  - Ofrece un servidor web en modo AP con SSID `FSO-RX` y contraseña `fso12345`.
  - El dashboard muestra en tiempo real:
    - potencia, tensión, corriente, estado de tracking
    - valores `F0`..`F3`
    - calidad del enlace y contadores de errores

## Estructura de la trama

- Preambulo: 4 bytes `0x55`
- Sincronización: 1 byte `0x7E`
- Longitud: 2 bytes (alto, bajo)
- Payload: datos de telemetría de `sizeof(TelemetryData)` bytes
- CRC32: 4 bytes calculados sobre el payload

## Hardware recomendado

- ESP32 DevKit para TX y RX
- Transmisor óptico (láser/infrarrojo) conectado al pin GPIO 17 del TX
- Detector LDR conectado al pin ADC 34 del RX
- LEDs de estado en el RX en GPIO 25, 26 y 27
- Botón en el TX en GPIO 4 y en el RX en GPIO 14

## Cómo compilar y cargar

Este proyecto usa PlatformIO. En `platformio.ini` están definidos los entornos:

- `env:tx` para compilar y cargar `main_tx.cpp`
- `env:rx` para compilar y cargar `main_rx.cpp`

Ejemplo de comando:

```ini
platformio run -e tx
platformio run -e rx
```

Para cargar:

```ini
platformio run -e tx -t upload
platformio run -e rx -t upload
```

> Asegúrate de seleccionar el entorno correcto según el ESP32 que uses en cada nodo.
