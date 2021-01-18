# Component makefile for tuya_mcu


INC_DIRS += $(tuya_mcu_ROOT)


tuya_mcu_INC_DIR = $(tuya_mcu_ROOT)
tuya_mcu_SRC_DIR = $(tuya_mcu_ROOT)

$(eval $(call component_compile_rules,tuya_mcu))
