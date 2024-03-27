FROM jpconstantineau/arduino-cli:0.33.0

RUN arduino-cli config init
COPY arduino-cli.yaml arduino-cli.yaml
RUN arduino-cli core update-index --config-file arduino-cli.yaml
RUN arduino-cli core install esp32:esp32@2.0.14 --additional-urls https://github.com/espressif/arduino-esp32/releases/download/2.0.14/package_esp32_index.json
RUN arduino-cli lib install "Adafruit BNO055"@1.6.1
RUN arduino-cli lib install "Adafruit BME280 Library"@2.2.2
RUN arduino-cli lib install "Adafruit MPR121"@1.1.1
RUN arduino-cli lib install "arduino-timer"@3.0.0
ENV ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true
COPY ICS_Library_for_Arduino_V2_1/IcsClass_V210.zip .
RUN arduino-cli lib install --zip-path IcsClass_V210.zip
RUN arduino-cli lib install --git-url https://github.com/KenichiroSameshima/SparkFun_Qwiic_Haptic_Driver_DA7280_Arduino_Library
RUN pip3 install pyserial
