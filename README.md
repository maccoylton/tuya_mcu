# tuya_mcu
esp-open-rtos implementation of tuya_mcu serial port protocol details of which can be found here:- 

https://developer.tuya.com/en/docs/iot/device-development/embedded-software-development/mcu-development-access/wifi-mcu-sdk-solution/tuya-cloud-universal-serial-port-access-protocol



tuya_mcu contains the basoc functions for communication with the tuya_mcu
  - timeAvailable is a global boolean that determines where or not your controller can provide the current time 

tuya_thermostat contains fucntions for handling a theomstat besd on the tuya_mcu 
  - yuo need to provicde a funttion to handle how tuya_thermostat send change to your controller 
  - fucntion prototype if void tuya_thermostat_emitChange(TUYA_Thermostat_change_type_t cmd)
  
