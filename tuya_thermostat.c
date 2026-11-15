/*
 *
 * Copyright 2019-2020  David B Brown  (@maccoylton)
 *
 */

#include <espressif/esp_wifi.h>
#include <sys/time.h>
#include <espressif/esp_sta.h>
#include <tuya_thermostat.h>
#include <task.h>
#include <string.h>

#define LOG_ERR    1
#define LOG_ACTION 2
#define LOG_EVENT  3
#define LOG_WIFI   4
#define LOG_MEM    5
#define LOG_FLOW   6
#define LOG_SNTP   7

extern int log_level;
#define LOG(level, ...) do { if (level <= log_level) { printf(__VA_ARGS__); } } while(0)

/* external variables */
bool powerOn = false;
float setPointTemp = 20.0f;
float internalTemp = 20.0f;
float externalTemp = 20.0f;
thermostat_mode_t mode = 0;
bool economyOn = false;
bool locked = false;
bool externalTempSensor= false;
uint8_t schedule[54] = {0};
bool    timeAvailable = false;

/* call for heat override state */
bool  callForHeat = false;
float callForHeatTargetTemp = 22.0f;
static bool     callForHeatTargetPower = false;
static uint32_t callForHeatClearedTick = 0;
static bool     callForHeatEchoGuard = false;

/* deferred re-assert of the override - queued from the rx handler, transmitted
 * once the receive buffer has been drained (see tuya_mcu_processRx) */
static bool     callForHeatPendingSetpoint = false;
static bool     callForHeatPendingPower = false;
static uint32_t callForHeatLastAttemptTick = 0;
static uint8_t  callForHeatAttempts = 0;
static bool     callForHeatGaveUp = false;
static bool     callForHeatFlushRegistered = false;

/* window after the override is cleared during which MCU reports that match the
 * override value are treated as stale echoes of our own command */
#define CALL_FOR_HEAT_ECHO_GUARD_MS 5000
/* minimum spacing between override re-asserts */
#define CALL_FOR_HEAT_REASSERT_MS 5000
/* give up rather than fight the MCU forever */
#define CALL_FOR_HEAT_MAX_ATTEMPTS 3
/* sanity limits - a bad value must never reach the MCU.
 * note the ceiling is the protocol range (HomeKit declares 10-38C for Target
 * Temperature, and the MCU accepts DP setpoints above its own 35C panel button
 * limit - verified at 37.0), NOT the call for heat pin value, because
 * send_setpoint_cmd() is also used to restore the user's real target. */
#define SETPOINT_MIN 5.0f
#define SETPOINT_MAX 38.0f

/* the compiled-in defaults for the temperature variables are guesses, not
 * readings (internalTemp/externalTemp/setPointTemp all start at 20.0f). if the
 * first real report happens to equal the default, the "did it change?" test
 * suppresses it and the value is never published - e.g. a room at 20.0C left
 * HomeKit showing 0C (32F) until the temperature moved. these flags force the
 * first report through. */
static bool internalTempValid = false;
static bool externalTempValid = false;
static bool setPointTempValid = false;


/*variables used here */
bool haveSchedule = false;
int scheduleCurrentDay = -1;
int scheduleCurrentPeriod = -1;




int tuya_thermostat_getScheduleDay()
{
    return scheduleCurrentDay;
}


int tuya_thermostat_getScheduleCurrentPeriod()
{
    return scheduleCurrentPeriod;
}


void tuya_thermostat_getSchedulePeriod(int day, int period, SchedulePeriod_t* p)
{
    if (day < 0 || day > 6 || period < 0 || period > 5)
        return;
    
    int i = 0;
    int j = period;
    if (day > 4)
        i = day - 4;
    
    p->minute = schedule[i * 18 + j * 3 + 0];
    p->hour = schedule[i * 18 + j * 3 + 1];
    p->temperature = schedule[i * 18 + j * 3 + 2] / 2.f;
}


float tuya_thermostat_getScheduleSetPointTemperature(int day, int period)
{
    SchedulePeriod_t p;
    tuya_thermostat_getSchedulePeriod(day, period, &p);
    return p.temperature;
}


float tuya_thermostat_getScheduleCurrentPeriodSetPointTemp()
{
    return tuya_thermostat_getScheduleSetPointTemperature(scheduleCurrentDay, scheduleCurrentPeriod );
}


/* unconditional senders - used where the value must reach the MCU even though
 * the cached state already matches it (override re-asserts) */
static void send_power_cmd(bool on)
{
    static uint8_t setPowerCmd[5] = {0x01, 0x01, 0x00, 0x01, 0x00};
    
    powerOn = on;
    setPowerCmd[4] = (uint8_t)(on ? 1 : 0);
    tuya_mcu_send_message(MSG_CMD_DP_CMD, setPowerCmd, 5);
}


static void send_setpoint_cmd(float temp)
{
    static uint8_t setSetpointCmd[8] = {0x02, 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00};
    
    /* never let a bad value (NaN, flash garbage, out of range) reach the MCU */
    if (!(temp >= SETPOINT_MIN))
        temp = SETPOINT_MIN;
    if (temp > SETPOINT_MAX)
        temp = SETPOINT_MAX;
    
    setPointTemp = temp;
    setSetpointCmd[7] = (uint8_t)(temp*2 + 0.5);
    tuya_mcu_send_message(MSG_CMD_DP_CMD, setSetpointCmd, 8);
}


/* invoked after the MCU receive buffer has been drained - the only place the
 * override is re-asserted, so the rx stream is never starved mid-frame */
static void call_for_heat_flush(void)
{
    if (!callForHeatPendingSetpoint && !callForHeatPendingPower)
        return;
    
    if (callForHeatAttempts >= CALL_FOR_HEAT_MAX_ATTEMPTS)
    {
        if (!callForHeatGaveUp)
        {
            callForHeatGaveUp = true;
            LOG(LOG_ERR, "%s: MCU did not accept the call for heat override after %d attempts - giving up\n", __func__, CALL_FOR_HEAT_MAX_ATTEMPTS);
        }
        callForHeatPendingSetpoint = false;
        callForHeatPendingPower = false;
        return;
    }
    
    uint32_t now = xTaskGetTickCount();
    if ((now - callForHeatLastAttemptTick) < pdMS_TO_TICKS(CALL_FOR_HEAT_REASSERT_MS))
        return;
    
    callForHeatLastAttemptTick = now;
    callForHeatAttempts++;
    
    if (callForHeatPendingPower)
    {
        callForHeatPendingPower = false;
        send_power_cmd(true);
    }
    
    if (callForHeatPendingSetpoint)
    {
        callForHeatPendingSetpoint = false;
        send_setpoint_cmd(TUYA_THERMOSTAT_CALL_FOR_HEAT_SETPOINT);
    }
    
    LOG(LOG_ACTION, "%s: re-asserting call for heat override (attempt %d of %d)\n", __func__, callForHeatAttempts, CALL_FOR_HEAT_MAX_ATTEMPTS);
}


static bool call_for_heat_echo_guard_active()
{
    if (!callForHeatEchoGuard)
        return false;
    
    if ((xTaskGetTickCount() - callForHeatClearedTick) < pdMS_TO_TICKS(CALL_FOR_HEAT_ECHO_GUARD_MS))
        return true;
    
    callForHeatEchoGuard = false;
    return false;
}


void tuya_thermostat_setCallForHeat(bool on, float real_target_temp, bool real_target_power)
{
    LOG(LOG_ACTION, "%s: %s\n", __func__, on ? "true" : "false");
    
    if (!callForHeatFlushRegistered)
    {
        /* transmit override re-asserts after the rx drain, never during it */
        tuya_mcu_set_rx_complete_callback(call_for_heat_flush);
        callForHeatFlushRegistered = true;
    }
    
    if (on == callForHeat)
        return;
    
    if (on)
    {
        /* force heating on regardless of the real target: power up the MCU and pin
         * the setpoint at the maximum so the MCU control loop keeps the relay closed */
        callForHeatTargetTemp  = real_target_temp;
        callForHeatTargetPower = real_target_power;
        callForHeat = true;
        callForHeatEchoGuard = false;
        callForHeatGaveUp = false;
        callForHeatPendingSetpoint = false;
        callForHeatPendingPower = false;
        callForHeatAttempts = 1;
        callForHeatLastAttemptTick = xTaskGetTickCount();
        send_power_cmd(true);
        send_setpoint_cmd(TUYA_THERMOSTAT_CALL_FOR_HEAT_SETPOINT);
        LOG(LOG_ACTION, "%s: call for heat on, setpoint override requested\n", __func__);
    }
    else
    {
        /* restore the real setpoint and power state; ignore the MCU's echo of the
         * override for a short window so it cannot clobber the real target */
        callForHeat = false;
        callForHeatTargetTemp  = real_target_temp;
        callForHeatTargetPower = real_target_power;
        callForHeatClearedTick = xTaskGetTickCount();
        callForHeatEchoGuard = true;
        callForHeatGaveUp = false;
        callForHeatPendingSetpoint = false;
        callForHeatPendingPower = false;
        callForHeatAttempts = 0;
        send_setpoint_cmd(callForHeatTargetTemp);
        send_power_cmd(callForHeatTargetPower);
        LOG(LOG_ACTION, "%s: call for heat off, real target and power restored\n", __func__);
    }
}


void tuya_thermostat_setPower( bool on, bool updateMCU)
{
    static uint8_t setPowerCmd[5] = {0x01, 0x01, 0x00, 0x01, 0x00};
    
    LOG(LOG_EVENT, "%s: Current State: %d, New State: %d, update MCU: %d\n", __func__, powerOn, on, updateMCU);
    
    if (on != powerOn)
    {
        powerOn = on;
        if (updateMCU)
        {
            setPowerCmd[4] = (uint8_t)(powerOn ? 1 : 0);
            tuya_mcu_send_message(MSG_CMD_DP_CMD, setPowerCmd, 5);
        } else {
            tuya_thermostat_emitChange(CHANGE_TYPE_POWER);
        }
    }
    
}


void tuya_thermostat_setSetPointTemp( float temp, bool updateMCU)
{
    static uint8_t setSetpointCmd[8] = {0x02, 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00};
    
    LOG(LOG_EVENT, "%s: current target: %2.1f, new target: %2.1f, update MCU: %s\n", __func__, setPointTemp, temp, updateMCU ? "True" : "False");
    
    /* always publish the first reading - see setPointTempValid */
    if (!setPointTempValid || temp != setPointTemp)
    {
        setPointTemp = temp;
        setPointTempValid = true;
        if (updateMCU)
        {
            setSetpointCmd[7] = (uint8_t)(setPointTemp*2 + 0.5);
            tuya_mcu_send_message(MSG_CMD_DP_CMD, setSetpointCmd, 8);
        } else {
            tuya_thermostat_emitChange(CHANGE_TYPE_SETPOINT_TEMP);
        }
    }
    
}

void tuya_thermostat_setInternalTemp( float temp)
{
    LOG(LOG_EVENT, "%s: temp: %2.1f, last internal temp: %2.1f\n", __func__, temp, internalTemp);
    
    /* always publish the first reading - see internalTempValid */
    if (!internalTempValid || temp != internalTemp)
    {
        internalTemp = temp;
        internalTempValid = true;
        tuya_thermostat_emitChange(CHANGE_TYPE_INTERNAL_TEMP);
    }
    
}


void tuya_thermostat_setExternalTemp( float temp)
{
    LOG(LOG_EVENT, "%s: temp: %f, last external temp: %f\n", __func__, temp, externalTemp);
    
    /* always publish the first reading - see internalTempValid */
    if (!externalTempValid || temp != externalTemp)
    {
        externalTemp = temp;
        externalTempValid = true;
        tuya_thermostat_emitChange(CHANGE_TYPE_EXTERNAL_TEMP);
    }
    
}


void tuya_thermostat_setMode(thermostat_mode_t m, bool updateMCU)
{
    static uint8_t setModeCmd[5] = {0x04, 0x04, 0x00, 0x01, 00};
    LOG(LOG_EVENT, "%s: current mode %d, new mode %d, update MCU %s\n", __func__, mode, m, updateMCU ? "True" : "False");
    if (m != mode)
    {
        mode = m;
        if (updateMCU)
        {
            setModeCmd[4] = mode;
            tuya_mcu_send_message(MSG_CMD_DP_CMD, setModeCmd, 5);
            
        } else {
            tuya_thermostat_emitChange(CHANGE_TYPE_MODE);
        }
    }
    
}


void tuya_thermostat_setEconomy( bool econ, bool updateMCU)
{
    static uint8_t setEconomyCmd[5] = {0x05, 0x01, 0x00, 0x01,0};
    
    LOG(LOG_EVENT, "%s: current %d, new %d, update MCU %s\n", __func__, economyOn, econ, updateMCU ? "True" : "False");
    
    if (econ != economyOn)
    {
        economyOn = econ;
        if(updateMCU)
        {
            setEconomyCmd[4] = (uint8_t)( economyOn ? 1 : 0);
            tuya_mcu_send_message(MSG_CMD_DP_CMD, setEconomyCmd, 5);
        } else
        {
            tuya_thermostat_emitChange(CHANGE_TYPE_ECONOMY);
        }
    }
    
}


void tuya_thermostat_setLock(bool lock, bool updateMCU)
{
    static uint8_t setLockCmd[5] = {0x06, 0x01, 0x00, 0x01, 0x00};
    LOG(LOG_EVENT, "%s: lock %d, updateMCU %s\n", __func__, lock, updateMCU ? "True" : "False");
    
    if (lock != locked)
    {
        locked = lock;
        if(updateMCU)
        {
            setLockCmd[4] = (uint8_t)(locked ? 1 : 0);
            tuya_mcu_send_message(MSG_CMD_DP_CMD, setLockCmd, 5);
        } else {
            tuya_thermostat_emitChange(CHANGE_TYPE_LOCK);
        }
    }
    
}


void tuya_thermostat_setSchedule(const uint8_t* s, uint8_t length, bool updateMCU)
{
    static  uint8_t setScheduleCmd[58]= {0x65, 0x00, 0x00, 0x36, 0};
    
    LOG(LOG_EVENT, "%s:\n", __func__);
    
    if (54 != length)
        return;
    
    bool changed = false;
    
    for (int i = 0; i < length; ++i)
    {
        if (s[i] != schedule[i])
        {
            schedule[i] = s[i];
            changed = true;
        }
    }
    if (changed)
    {
        haveSchedule = true;
        if (updateMCU)
        {
            uint8_t* ptr = setScheduleCmd + 4;
            for (int i = 0; i < 54; ++i)
            {
                ptr[i] = schedule[i];
            }
            tuya_mcu_send_message(MSG_CMD_DP_CMD, setScheduleCmd, 58);
        }
    }
    
}


void tuya_device_handleDPStatusMsg(uint8_t msg[])
{
    uint8_t payload_length = tuya_mcu_get_payload(msg, payload);
    
    switch(payload[0])
    {
        case CHANGE_TYPE_POWER:
        {
            LOG(LOG_EVENT, "%s: Type: Power\n", __func__);
            
            if (5 == payload_length)
            {
                bool reported_power = (1 == payload[4]);
                
                if (callForHeat)
                {
                    if (reported_power)
                    {
                        /* confirmation of the forced power on */
                        callForHeatPendingPower = false;
                        callForHeatAttempts = 0;
                    }
                    else
                    {
                        /* someone at the panel turned it off - the panel wins.
                         * abandon the override rather than fighting a person
                         * standing next to a heater. */
                        callForHeat = false;
                        callForHeatPendingSetpoint = false;
                        callForHeatPendingPower = false;
                        callForHeatEchoGuard = false;
                        callForHeatGaveUp = false;
                        callForHeatAttempts = 0;
                        LOG(LOG_ACTION, "%s: panel power off during call for heat - override abandoned\n", __func__);
                        tuya_thermostat_setPower(false, false);
                        tuya_thermostat_emitChange(CHANGE_TYPE_CALL_FOR_HEAT);
                    }
                }
                else if (reported_power && (powerOn != reported_power) && call_for_heat_echo_guard_active())
                {
                    /* stale echo of the forced power on, arriving after the override cleared */
                    LOG(LOG_ACTION, "%s: stale power on echo ignored\n", __func__);
                    send_power_cmd(callForHeatTargetPower);
                }
                else
                {
                    tuya_thermostat_setPower(reported_power, false);
                }
            }
        }
            break;
        case CHANGE_TYPE_SETPOINT_TEMP:
        {
            LOG(LOG_EVENT, "%s: Type: SETPOINT_TEMP\n", __func__);
            
            if (8 == payload_length)
            {
                float reported_temp = payload[7]/2.0f;
                
                if (callForHeat)
                {
                    if (reported_temp == TUYA_THERMOSTAT_CALL_FOR_HEAT_SETPOINT)
                    {
                        /* MCU accepted the override - nothing to do, never emit */
                        callForHeatPendingSetpoint = false;
                        callForHeatAttempts = 0;
                        callForHeatGaveUp = false;
                    }
                    else if (!callForHeatGaveUp && !callForHeatPendingSetpoint)
                    {
                        /* panel knob turned away from the override (or the MCU has
                         * not applied it yet) - queue a re-assert, do not emit */
                        callForHeatPendingSetpoint = true;
                        LOG(LOG_ACTION, "%s: setpoint differs from override - re-assert queued\n", __func__);
                    }
                }
                else if ((reported_temp == TUYA_THERMOSTAT_CALL_FOR_HEAT_SETPOINT) && (setPointTemp != reported_temp) && call_for_heat_echo_guard_active())
                {
                    /* stale echo of the override, arriving after the override cleared */
                    LOG(LOG_ACTION, "%s: stale setpoint echo ignored\n", __func__);
                    send_setpoint_cmd(callForHeatTargetTemp);
                }
                else
                {
                    tuya_thermostat_setSetPointTemp(reported_temp, false);
                }
            }
            
        }
            break;
        case CHANGE_TYPE_INTERNAL_TEMP:
        {
            LOG(LOG_EVENT, "%s: Type: Internal Temp\n", __func__);
            if (externalTempSensor)
            {
                LOG(LOG_EVENT, "%s: Ignoring internal temp, using external temp\n", __func__);
            }
            else {
                if (8 == payload_length)
                {
                    tuya_thermostat_setInternalTemp(payload[7]/2.0f);
                }
                
            }
        }
            break;
        case CHANGE_TYPE_MODE:
        {
            LOG(LOG_EVENT, "%s: Type: Mode\n", __func__);
            if (5 == payload_length)
            {
                tuya_thermostat_setMode(payload[4] ? MODE_MANUAL : MODE_SCHEDULE, false);
                
            }
            
        }
            break;
        case CHANGE_TYPE_ECONOMY:
        {
            LOG(LOG_EVENT, "%s: Type: Economy\n", __func__);
            if (5 == payload_length)
            {
                tuya_thermostat_setEconomy(1 == payload[4], false);
            }
        }
            break;
        case CHANGE_TYPE_LOCK:
        {
            LOG(LOG_EVENT, "%s: Type: Lock\n", __func__);
            
            if (5 == payload_length)
            {
                tuya_thermostat_setLock(1 == payload[4], false);
            }
        }
            break;
        case 0x65:
        {
            LOG(LOG_EVENT, "%s: Type: Schedule\n", __func__);
            
            if (58  == payload_length)
            {
                tuya_thermostat_setSchedule(payload + 4, tuya_mcu_get_payload_length(msg)-4, false);
            }
        }
            break;
        case CHANGE_TYPE_EXTERNAL_TEMP:
        {
            if (externalTempSensor) {
                LOG(LOG_EVENT, "%s: Type: External Temp\n", __func__);
                if (8 == payload_length)
                {
                    tuya_thermostat_setExternalTemp(payload[7]/2.0f);
                }
            } else {
                LOG(LOG_EVENT, "%s: Ignoring external temp, using internal temp\n", __func__);
            }
        }
            break;
        case 0x68:
        {
            LOG(LOG_WIFI, "%s: Type: Unknown (0x68)\n", __func__);
        }
            break;
        default:
            LOG(LOG_ERR, "%s: Type: Default Unknown (0x%02X)\n", __func__, payload[0]);
    }
    
}



void tuya_device_loop (void *args){
    
    static uint32_t timeLastScheduleUpdate = 0;

    if (timeAvailable)
    {
        uint32_t timeNow = tuya_mcu_get_millis();
        
        if (timeNow - timeLastScheduleUpdate > 30000)
        {
            timeLastScheduleUpdate = timeNow;
            int day = 0;
            int hour = 0;
            int mins = 0;
            if (tuya_mcu_getTime(&day, &hour, &mins))
            {
                // make monday first day
                if (day == 0)
                    day = 7;
                --day;
                
                int period = -1;
                SchedulePeriod_t p;
                for (int i = 0 ; i < 6; ++i)
                {
                    tuya_thermostat_getSchedulePeriod(day, i, &p);
                    
                    if (hour < p.hour)
                    {
                        break;
                    }
                    else if (hour == p.hour && mins < p.minute)
                    {
                        break;
                    }
                    else
                    {
                        period = i;
                    }
                }
                if (-1 == period) // still on previous day schedule
                {
                    if (day == 0)
                        day = 7;
                    --day;
                    period = 5;
                }
                if (day != scheduleCurrentDay || period != scheduleCurrentPeriod)
                {
                    scheduleCurrentDay = day;
                    scheduleCurrentPeriod = period;
                    tuya_thermostat_emitChange(CHANGE_TYPE_CURRENT_SCHEDULE_PERIOD);
                }
            }
        }
    }

    
}
    /*
     String toString()
     {
     String str;
     str += "HEATING: ";
     if (getIsHeating())
     str  += "ON\n";
     else
     str += "OFF\n";
     str +=
     String("SetTemp: ") + String(getSetPointTemp()) +
     String(", internal temp: ") + String(getInternalTemp()) +
     String(", external temp: ") + String(getExternalTemp()) +
     String(", lock: ") + String(getLock()) +
     String(", mode: ") + String(getMode()) +
     String(", power: ") + String(getPower()) +
     String(", economy: ") + String(getEconomy()) +
     String("\n\n") +
     String("Schedule:");
     for (int i = 0; i < 3; ++i)
     {
     if (0 == i)
     str += String("\nM-F: ");
     else if (1 == i)
     str += String("\nSat: ");
     else if (2 == i)
     str += String("\nSun: ");
     for(int j = 0; j < 6; ++j)
     {
     uint8_t m = schedule[i * 18 + j * 3 + 0];
     uint8_t h = schedule[i * 18 + j * 3 + 1];
     float t = schedule[i * 18 + j * 3 + 2] / 2.f;
     if (0 != j)
     str += ", ";
     str += String("time") + String(j+1) + String(": ");
     str += String(h) + String(":") + String(m);
     str += String(" temp") + String(j+1) + String(": ");
     str += String(t);
     }
     }
     
     str += "\n";
     str += "curr day:";
     str += String(scheduleCurrentDay);
     str += ", curr period:";
     str += String(scheduleCurrentPeriod);
     str += ", curr temp:";
     str += String(getScheduleCurrentPeriodSetPointTemp());
     return str;
     }
     */
    
