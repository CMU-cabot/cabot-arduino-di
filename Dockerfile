FROM jpconstantineau/arduino-cli:0.33.0

RUN arduino-cli config init
COPY arduino-cli.yaml arduino-cli.yaml
RUN arduino-cli core update-index --config-file arduino-cli.yaml
RUN arduino-cli core install esp32:esp32@1.0.6 --additional-urls https://github.com/espressif/arduino-esp32/releases/download/1.0.6/package_esp32_index.json
RUN arduino-cli lib install "Rosserial Arduino Library"@0.9.1
RUN arduino-cli lib install "Adafruit BNO055"@1.6.1
RUN arduino-cli lib install "Adafruit BME280 Library"@2.2.2
RUN arduino-cli lib install "Adafruit MPR121"@1.1.1
RUN arduino-cli lib install "arduino-timer"@3.0.0
RUN arduino-cli lib install "ESP32 AnalogWrite"@0.1
RUN pip3 install pyserial
