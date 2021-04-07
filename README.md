# tuya_mcu
esp-open-rtos implementation of tuya_mcu serial port protocol details of which can be found here:- 

https://developer.tuya.com/en/docs/iot/device-development/embedded-software-development/mcu-development-access/wifi-mcu-sdk-solution/tuya-cloud-universal-serial-port-access-protocol



tuya_mcu contains the basic functions for communication with the tuya_mcu
  - timeAvailable is a global boolean that determines where or not your controller can provide the current time 

tuya_thermostat contains functions for handling a thermostat based on the tuya_mcu 
  - you need to provide a function to handle how tuya_thermostat sends changes to your controller 
  - function prototype: void tuya_thermostat_emitChange(TUYA_Thermostat_change_type_t cmd)
  
