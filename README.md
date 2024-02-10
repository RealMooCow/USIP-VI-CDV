# USIP-VI-CDV
**Conductivity payload code for USIP VI/NEBP 2023-2024**


# PURPOSE
This is the software project for UH's conductivity payload, based on what was created by Andy Nguyencuu for USIP IV's conductivity payload. It contains mostly Arduino files to run on an Arduino Mega along with some Python to gather and process data collected on payload flights.

# SENSORS
_There are three sensors which will be used on the payload. Their datasheets have been linked below._

**[Adafruit LPS25]([url](https://www.st.com/resource/en/datasheet/lps25hb.pdf)https://www.st.com/resource/en/datasheet/lps25hb.pdf)**
    - Barometer

**[Bosch BME-680]([url](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme680-ds001.pdf)https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme680-ds001.pdf)**
    - Gas sensor
    - Measures relative humidity, VOC, temperature, and pressure

**[STMicro LSM303AGR]([url](https://www.st.com/resource/en/datasheet/lsm303agr.pdf)https://www.st.com/resource/en/datasheet/lsm303agr.pdf)**
    - Accelerometer and magnetometer
