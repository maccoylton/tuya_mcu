/*
 *
 * Copyright 2019-2020  David B Brown  (@maccoylton)
 *
 */

#include <espressif/esp_wifi.h>
#include <sys/time.h>
#include <espressif/esp_sta.h>
#include <tuya_thermostat.h>
#include <string.h>

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
    if (i > 4)
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


void tuya_thermostat_setPower( bool on, bool updateMCU)
{
    static uint8_t setPowerCmd[5] = {0x01, 0x01, 0x00, 0x01, 0x00};
    
    printf ("%s: Curent State: %d:  New State %d: update MCU %d ", __func__, powerOn, on, updateMCU);
    
    if (on != powerOn)
    {
        powerOn = on;
        if (updateMCU)
        {
            // send change to mcu
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
    
    printf ("%s: current target: %2.1f, new target: %2.1f: update MCU: %s ", __func__, setPointTemp, temp, updateMCU ? "True" : "False");
    
    if (temp != setPointTemp)
    {
        setPointTemp = temp;
        if (updateMCU)
        {
            // send change to mcu
            //RX: 55 aa 01 07 00 08 02 02 00 04 00 00 00 24 3c
            setSetpointCmd[7] = (uint8_t)(setPointTemp*2 + 0.5);
            tuya_mcu_send_message(MSG_CMD_DP_CMD, setSetpointCmd, 8);
        } else {
            tuya_thermostat_emitChange(CHANGE_TYPE_SETPOINT_TEMP);
        }
    }
    printf (" End\n");
    
}

void tuya_thermostat_setInternalTemp( float temp)
{
    printf (" temp: %2.1f, last inernal temp:%2.1f: ", temp, internalTemp);
    
    if (temp != internalTemp)
    {
        internalTemp = temp;
        tuya_thermostat_emitChange(CHANGE_TYPE_INTERNAL_TEMP);
    }
    //printf (" End\n");
    
}


void tuya_thermostat_setExternalTemp( float temp)
{
    printf ("%s: temp:%f2.2, last external temp:%f2.2\n", __func__, temp, externalTemp);
    
    if (temp != externalTemp)
    {
        externalTemp = temp;
        tuya_thermostat_emitChange(CHANGE_TYPE_EXTERNAL_TEMP);
    }
    printf ("%s: End\n", __func__);
    
}


void tuya_thermostat_setMode(thermostat_mode_t m, bool updateMCU)
{
    static uint8_t setModeCmd[5] = {0x04, 0x04, 0x00, 0x01, 00};
    printf ("%s: current mode %d; new mode %d: update MCU %s\n", __func__, mode, m, updateMCU ? "True" : "False");
    if (m != mode)
    {
        mode = m;
        if (updateMCU)
        {
            // send change to mcu
            setModeCmd[4] = mode;
            tuya_mcu_send_message(MSG_CMD_DP_CMD, setModeCmd, 5);
            
        } else {
            tuya_thermostat_emitChange(CHANGE_TYPE_MODE);
        }
    }
    printf ("%s: End\n", __func__);
    
}


void tuya_thermostat_setEconomy( bool econ, bool updateMCU)
{
    static uint8_t setEconomyCmd[5] = {0x05, 0x01, 0x00, 0x01,0};
    
    printf ("%s: current %d, new %d, update MCU %s", __func__, economyOn, econ, updateMCU ? "True" : "False");
    
    if (econ != economyOn)
    {
        economyOn = econ;
        if(updateMCU)
        {
            // send change to mcu
            setEconomyCmd[4] = (uint8_t)( economyOn ? 1 : 0);
            tuya_mcu_send_message(MSG_CMD_DP_CMD, setEconomyCmd, 5);
        } else
        {
            tuya_thermostat_emitChange(CHANGE_TYPE_ECONOMY);
        }
    }
    printf ("%s: End\n", __func__);
    
}


void tuya_thermostat_setLock(bool lock, bool updateMCU)
{
    static uint8_t setLockCmd[5] = {0x06, 0x01, 0x00, 0x01, 0x00};
    printf ("%s: Start\n", __func__);
    
    if (lock != locked)
    {
        locked = lock;
        if(updateMCU)
        {
            // send change to mcu
            setLockCmd[4] = (uint8_t)(locked ? 1 : 0);
            tuya_mcu_send_message(MSG_CMD_DP_CMD, setLockCmd, 5);
        } else {
            tuya_thermostat_emitChange(CHANGE_TYPE_LOCK);
        }
    }
    printf ("%s: End\n", __func__);
}


void tuya_thermostat_setSchedule(const uint8_t* s, uint8_t length, bool updateMCU)
{
    static  uint8_t setScheduleCmd[58]= {0x65, 0x00, 0x00, 0x36, 0};
    
    printf ("%s: Start\n", __func__);
    
    if (54 != length)
        return;
    
    bool changed = false;
    
    //setSchedule weekday, sat, sun x 6 times
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
            // send change to mcu
            //payload = {0x65, 0x00, 0x00, 0x36, 0};
            uint8_t* ptr = setScheduleCmd + 4;
            for (int i = 0; i < 54; ++i)
            {
                ptr[i] = schedule[i];
            }
            tuya_mcu_send_message(MSG_CMD_DP_CMD, setScheduleCmd, 58);
        }
    }
    printf ("%s: End\n", __func__);
}


void tuya_device_handleDPStatusMsg(uint8_t msg[])
{
    //printf (" %s: ", __func__);
    
    uint8_t payload_length = tuya_mcu_get_payload(msg, payload);
    
    
    //printf (" Payload, length %d: ", payload_length);
    //for (uint8_t i = 0; i < payload_length ; i++)
    //{
    //    printf(" 0x%02X", payload[i]);
    //}
    switch(payload[0])
    {
        case CHANGE_TYPE_POWER: // power on/off (byte 4)
        {
            printf ("Type: Power\n");
            
            //RX: 55 aa 01 07 00 05 01 01 00 01 01 10
            if (5 == payload_length    )
            {
                tuya_thermostat_setPower(1 == payload[4], false);
            }
        }
            break;
        case CHANGE_TYPE_SETPOINT_TEMP: // set point temp
        {
            //RX: 55 aa 01 07 00 08 02 02 00 04 00 00 00 2e 45
            printf ("Type: SETPOINT_TEMP\n");
            
            if (8 == payload_length)
            {
                tuya_thermostat_setSetPointTemp(payload[7]/2.0f, false);
            }
            
        }
            break;
        case CHANGE_TYPE_INTERNAL_TEMP: // temperature
        {
            //RX: 55 aa 01 07 00 08 03 02 00 04 00 00 00 24 3c
            printf ("Type: Internal Temp\n");
            if (externalTempSensor)
            {
                printf ("Type: Inernal Temp Ignoring as using external temp\n");
            }
            else {
                /* no external temp sensor so use internal  */
                if (8 == payload_length)
                {
                    tuya_thermostat_setInternalTemp(payload[7]/2.0f);
                }
                
            }
        }
            break;
        case CHANGE_TYPE_MODE: // mode (schedule = 0 / manual = 1)
        {
            //RX: 55 aa 01 07 00 05 04 04 00 01 00 15
            printf ("Type: Mode\n");
            if (5 == payload_length)
            {
                tuya_thermostat_setMode(payload[4] ? MODE_MANUAL : MODE_SCHEDULE, false);
                
            }
            
        }
            break;
        case CHANGE_TYPE_ECONOMY: // economy
        {
            //RX: 55 aa 01 07 00 05 05 01 00 01 00 13
            printf ("Type: Economy\n");
            if (5 == payload_length)
            {
                tuya_thermostat_setEconomy(1 == payload[4], false);
            }
        }
            break;
        case CHANGE_TYPE_LOCK: // lock
        {
            //RX: 55 aa 01 07 00 05 06 01 00 01 00 14
            //    TX: 55 AA 00 06 00 05 06 01 00 01 01 00
            
            printf ("Type: Lock\n");
            
            if (5 == payload_length)
            {
                tuya_thermostat_setLock(1 == payload[4], false);
            }
        }
            break;
        case 0x65: // schedule
        {
            // RX: 55 aa 01 07 00 3a 65
            // 00 00 36 00
            // 06 28 00 08 1e 1e b 1e 1e d 1e 00 11 2c 00 16 1e 00
            // 06 28 00 08 28 1e b 28 1e d 28 00 11 28 00 16 1e 00
            // 06 28 00 08 28 1e b 28 1e d 28 00 11 28 00 16 1e f
            printf ("Type: Schedule\n");
            
            if (58  == payload_length)
            {
                tuya_thermostat_setSchedule(payload + 4, tuya_mcu_get_payload_length(msg)-4, false);
            }
        }
            break;
        case CHANGE_TYPE_EXTERNAL_TEMP: // floor temp
        {
            // RX: 55 aa 01 07 00 08 66 02 00 04 00 00 00 00 7b
            if (externalTempSensor) {
                /*  external temp sensor is present */
                printf ("Type: External Temp\n");
                if (8 == payload_length)
                {
                    tuya_thermostat_setExternalTemp(payload[7]/2.0f);
                }
            } else {
                printf ("Type: External Temp Ignoring as using internal temp\n");
            }
        }
            break;
        case 0x68: // ??
        {
            // RX: 55 aa 01 07 00 05 68 01 00 01 01 77
            printf ("Type: Unknown\n");
            if (5 == payload_length)
            {
            }
            
        }
            break;
        default:
            printf ("Type: Default Unknown\n");
    }
    
}



void tuya_device_loop (void *args){
    

    // update scheduleCurrentDay  and scheduleCurrentPeriod
    if (timeAvailable)
    {
        uint32_t timeNow = tuya_mcu_get_millis();
        uint32_t timeLastScheduleUpdate = timeNow+1;
        
        if (timeNow - timeLastScheduleUpdate > 30000)
        {
            timeLastScheduleUpdate = timeNow;
            int day = 0;
            int hour = 0;
            int mins = 0;
            if (tuya_mcu_getTime(day, hour, mins))
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
    
