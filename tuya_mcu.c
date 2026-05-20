/*
 *
 * Copyright 2019-2020  David B Brown  (@maccoylton) 
 * 
 */

#include <tuya_mcu.h>

#define LOG_ERR    1
#define LOG_ACTION 2
#define LOG_EVENT  3
#define LOG_WIFI   4
#define LOG_MEM    5
#define LOG_FLOW   6
#define LOG_SNTP   7

extern int log_level;
#define LOG(level, ...) do { if (level <= log_level) { printf(__VA_ARGS__); } } while(0)

bool    gotHeartbeat = false;
bool    gotProdKey = false;
bool    gotWifiMode = false;
WifiState_t wifiState;
bool    sendWifiStateMsg = false;
bool    resetBuffer = false;
uint32_t currentByte = 0;
uint8_t dataLength = 0;
uint32_t lastMS = 0;
uint8_t mcu_init_stage = 0;
uint8_t mcuProtocolVersion = 0;
bool    canQuery=false;
WifiMode_t wifiMode;
uint32_t HeartbeatDelay = 3000;



/* declare these as global to avoid fragmentation */

uint8_t msg[MAX_RECEIVE_BUFFER_LENGTH];
uint8_t payload[MAX_RECEIVE_BUFFER_LENGTH-TUYA_MCU_HEADER_SIZE];
uint8_t messageToSend[MAX_SEND_BUFFER_LENGTH];
SemaphoreHandle_t write_semaphore = NULL;



void tuya_mcu_init() {
    write_semaphore = xSemaphoreCreateMutex();
    
    xTaskCreate(tuya_mcu_loop, "tuya_mcu_loop", 512 , NULL, tskIDLE_PRIORITY+1, NULL);
    
}



bool tuya_mcu_getTime(int *dayOfWeek, int *hour, int *minutes)
{
    LOG(LOG_FLOW, "%s:\n", __func__);
    bool gotTime = false;
    struct tm* new_time = NULL;
    time_t tnow = (time_t)-1;
    if (timeAvailable == true)
    {
        struct timezone tz = {1*60, 0};
        struct timeval tv = {0};
        
        gettimeofday(&tv, &tz);
        tnow = time(NULL);
        
        new_time = localtime (&tnow);
        // sunday = 0, sat = 6
        *dayOfWeek = new_time->tm_wday;
        *hour = new_time->tm_hour;
        *minutes = new_time->tm_min;
        
        gotTime = true;
    }
    LOG(LOG_EVENT, "%s: day: %d, hour: %d, min: %d (epoch %ld)\n", __func__, *dayOfWeek, *hour, *minutes, (long int)tnow);
    
    return gotTime;
}


long long tuya_mcu_get_millis() {
    //printf ("%s: ", __func__);
    /*    struct timeval te;
     gettimeofday(&te, NULL);
     printf("Time in microseconds: %0.3f microseconds: ",
     (float)(te.tv_sec));
     //printf ("End: ", __func__);
     return te.tv_sec * 1000LL + te.tv_usec / 1000;*/
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}


void tuya_mcu_sendTime(bool timeAvailable)
{
    static uint8_t sendTimeCmd[8]= {01,00,00,00,00,00,00,00};
    LOG(LOG_EVENT, "%s: timeAvailbale: %s, can query: %s\n", __func__, timeAvailable ? "True" : "False", canQuery ? "True" : "False");
    struct tm* new_time = NULL;
    if (timeAvailable ==true)
    {
        struct timezone tz;
        struct timeval tv;
        
        gettimeofday(&tv, &tz);
        
        time_t tnow = time(NULL);
        
        static time_t nexttv = 0;
        if (nexttv < tv.tv_sec && canQuery)
        {
            LOG(LOG_FLOW, "%s: getting local time\n", __func__);
            nexttv = tv.tv_sec + 3600;
            new_time = localtime(&tnow);
            if (new_time != NULL)
            {
                LOG(LOG_FLOW, "%s: got new_time\n", __func__);
                sendTimeCmd[7] = (new_time->tm_wday == 0 ? 7 : new_time->tm_wday);
                sendTimeCmd[6] = new_time->tm_sec;
                sendTimeCmd[5] = new_time->tm_min;
                sendTimeCmd[4] = new_time->tm_hour;
                sendTimeCmd[3] = new_time->tm_mday;
                sendTimeCmd[2] = new_time->tm_mon  + 1;
                sendTimeCmd[1] = new_time->tm_year % 100;
                tuya_mcu_send_message(MSG_CMD_OBTAIN_LOCAL_TIME, sendTimeCmd, 8);
            } else {
                LOG(LOG_ERR, "%s: NOT got new_time\n", __func__);
            }
        }
        LOG(LOG_FLOW, "%s: time since epoch %ld\n", __func__, (long int)tnow );
    }
    
    LOG(LOG_FLOW, "%s: End\n", __func__);
}


bool serial_available(){
    //printf ("%s: ", __func__);
    
    if (uart_rxfifo_wait(uart_port,0) == 0){
        //printf ("nothing to read. End: ");
        return false;
    } else {
        //printf ("something to read. End: ");
        return true;
    }
}


int serial_read (){
    int byte;
    byte = uart_getc_nowait(uart_port);
    return(byte);
}


void reset()
{
    //printf ("%s: ", __func__);
    resetBuffer = true;
    //printf ("End: ");
}


void checkReset()
{
    if (resetBuffer == true )
    {
        LOG(LOG_ACTION, "%s: Resetting\n", __func__);
        currentByte = 0;
        dataLength = 0;
        resetBuffer = false;
    }
}


int serial_write (const uint8_t* ptr, uint8_t len){
    
    if ( write_semaphore != NULL){
        LOG(LOG_FLOW, "%s:\n", __func__);
       
        if( xSemaphoreTake( write_semaphore, ( TickType_t ) 100 ) == pdTRUE )
        {
            
            for(uint8_t i = 0; i < len; i++) {
                if(((char *)ptr)[i] == '\r')
                    continue;
                if(((char *)ptr)[i] == '\n')
                    uart_putc(uart_port, '\r');
                LOG(LOG_FLOW, "0x%02X ", ptr[i]);
                uart_putc(uart_port, ((char *)ptr)[i]);
            }
            LOG(LOG_FLOW, "%s: Sent %d bytes\n", __func__, len);
                        
            xSemaphoreGive( write_semaphore );
            
            return len;
        }
        else
        {
            LOG(LOG_ERR, "%s: unable to obtain semaphore\n", __func__);
            return -1;
        }
    } else {
        LOG(LOG_ERR, "%s: semaphore NULL\n", __func__);
        return -1;
    }
}


/* bool tuya_mcu_msg_buffer_addbyte(uint8_t byte, uint8_t msg[])
{
    //printf ("%s: Start, byte:0x%02X\n", __func__, byte);
    
    uint32_t newLastMS = tuya_mcu_get_millis();
    
    if (newLastMS - lastMS > 5){
        //printf ("%s: newLastMS:%d, lastMS %d, difference %d ", __func__, newLastMS, lastMS, newLastMS - lastMS );
        reset();
    }
    
    checkReset();
    
    lastMS = newLastMS;
    
    if (E_MAGIC1 == currentByte)
    {
        if (0x55 == byte)
        {
            //printf ("%s: First Magic Bit\n", __func__);
            msg[currentByte] = byte;
            ++currentByte;
        }
        
    }
    else if (E_MAGIC2 == currentByte)
    {
        if (0xAA == byte)
        {
            //printf ("%s: Second Magic Bit\n", __func__);
            msg[currentByte] = byte;
            ++currentByte;
        }
        else
        {
            printf (": not magic byte so reesetting: " );
            reset();
        }
    }
    else
    {
        msg[currentByte] = byte;
        
        if (E_PAYLOAD_LENGTH_LO == currentByte){
            dataLength = msg[E_PAYLOAD_LENGTH_HI] * 0x100 + msg[E_PAYLOAD_LENGTH_LO];
            //printf (" : Got length bytes, length %d ", dataLength);
        }
        if (currentByte >= 6u && dataLength+6u == currentByte)
        {
            //printf (": Got last byte: ");
            reset();
            return true;
        }
        else
            ++currentByte;
    }
    //printf ("%s: End\n", __func__);
    
    return false;
}
 */


uint16_t tuya_mcu_get_msg_length(uint8_t msg[])
{
    return (tuya_mcu_get_payload_length(msg) + TUYA_MCU_HEADER_SIZE);
}


uint8_t tuya_mcu_calc_checksum(uint8_t msg[])
{
    uint16_t message_length = tuya_mcu_get_msg_length (msg);
    uint8_t chksum = 0;
    for (uint8_t i = 0 ; i < message_length -1; ++i)
        chksum += msg[i];
    
    return chksum;
}


void tuya_mcu_set_checksum(uint8_t msg[])
{
    //printf ("%s: Start\n", __func__);
    msg[tuya_mcu_get_msg_length(msg) - 1] = tuya_mcu_calc_checksum(msg);
    //printf ("%s: End\n", __func__);
}

void tuya_mcu_print_message (uint8_t msg[], bool valid){
    
    uint16_t message_length = MAX_SEND_BUFFER_LENGTH;
    if (valid == true) {
        message_length = tuya_mcu_get_msg_length (msg);
    }
    LOG(LOG_EVENT, "%s: Message:", __func__);
    
    for (uint8_t i=0 ;  i < message_length ; i++)
    {
        switch (i) {
            case E_MAGIC1:
                LOG(LOG_EVENT, " MB 1:");
                break;
            case E_MAGIC2:
                LOG(LOG_EVENT, " MB 2:");
                break;
            case E_VERSION:
                LOG(LOG_EVENT, " Version:");
                break;
            case E_CMD:
                LOG(LOG_EVENT, " CMD:");
                break;
            case E_PAYLOAD_LENGTH_HI:
                LOG(LOG_EVENT, " Length HI:");
                break;
            case E_PAYLOAD_LENGTH_LO:
                LOG(LOG_EVENT, " Length Low:");
                break;
            case E_PAYLOAD:
                LOG(LOG_EVENT, "\nPayload:");
            default:
                break;
        }
        if (i == message_length-1)
            LOG(LOG_EVENT, " Checksum:");
        LOG(LOG_EVENT, " 0x%02X", msg[i]);
    }
}



bool tuya_mcu_message_is_valid(uint8_t msg[])
{
    uint16_t message_length = tuya_mcu_get_msg_length (msg);
    uint8_t checksum = tuya_mcu_calc_checksum(msg);
    
    if (msg[E_MAGIC1] == 0x55 && msg[E_MAGIC2] == 0xaa && message_length >= TUYA_MCU_HEADER_SIZE && checksum == msg[message_length - 1] ){
        LOG(LOG_EVENT, "%s: Valid\n", __func__);
        return true;
    } else {
        if ( msg[E_MAGIC1] != 0x55 || msg[E_MAGIC2] != 0xaa)
            LOG(LOG_ERR, "%s: invalid magic bits 0x%02X 0x%02X\n", __func__, msg[E_MAGIC1], msg[E_MAGIC2]);
        if (message_length < TUYA_MCU_HEADER_SIZE)
            LOG(LOG_ERR, "%s: invalid message length %d\n", __func__, message_length);
        if ( checksum != msg[message_length - 1] )
            LOG(LOG_ERR, "%s: invalid checksum, calculated %d, received %d, length: %d\n", __func__, checksum, (uint8_t) msg[message_length - 1], message_length );
        return false;
    }
}


uint8_t tuya_mcu_get_command(uint8_t msg[])
{
    //printf ("%s: Start\n", __func__);
    return msg[E_CMD];
}


uint8_t tuya_mcu_get_version(uint8_t msg[])
{
    //printf ("%s: Start\n", __func__);
    return msg[E_VERSION];
}


uint16_t tuya_mcu_get_payload_length(uint8_t msg[])
{
    return (msg[E_PAYLOAD_LENGTH_HI] * 0x100 + msg[E_PAYLOAD_LENGTH_LO]);
}


uint16_t tuya_mcu_get_payload(uint8_t msg[], uint8_t payload[])
{
    uint16_t payload_length = tuya_mcu_get_payload_length(msg);
    memcpy(payload, &msg[E_PAYLOAD], payload_length);
    LOG(LOG_FLOW, "%s: length %d\n", __func__, payload_length);
    return (payload_length);
}


void tuya_mcu_set_payload_length(uint8_t msg[], uint8_t payload_length)
{
    if (payload_length <= (MAX_SEND_BUFFER_LENGTH - TUYA_MCU_HEADER_SIZE))
    {
        msg[E_PAYLOAD_LENGTH_HI] = payload_length >> 8;
        msg[E_PAYLOAD_LENGTH_LO] = payload_length & 0xff;
    }
    else
    {
        LOG(LOG_ERR, "%s: Payload too long: %d\n", __func__, payload_length);
    }
}


void tuya_mcu_set_payload(uint8_t msg[], uint8_t* payload, uint8_t payload_length)
{
    if (payload_length <= (MAX_SEND_BUFFER_LENGTH - TUYA_MCU_HEADER_SIZE))
    {
        tuya_mcu_set_payload_length ( msg, payload_length);
        memcpy(&msg[E_PAYLOAD], payload, payload_length);
    }
    else
    {
        LOG(LOG_ERR, "%s: Payload too long: %d\n", __func__, payload_length);
    }
}


void tuya_mcu_send_cmd(uint8_t cmd)
{
    LOG(LOG_EVENT, "%s: 0x%02X\n", __func__, cmd);
    
    messageToSend[E_MAGIC1] = 0x55;
    messageToSend[E_MAGIC2] = 0xaa;
    messageToSend[E_VERSION] = 0;
    messageToSend[E_CMD] = cmd;
    tuya_mcu_set_payload_length(messageToSend,0);
    tuya_mcu_set_checksum (messageToSend);
    
    if (tuya_mcu_message_is_valid(messageToSend) == true)
    {
        serial_write(messageToSend, tuya_mcu_get_msg_length (messageToSend));
    } else {
        tuya_mcu_print_message (messageToSend, false);
    }
    LOG(LOG_FLOW, "%s: End\n", __func__);
}


void tuya_mcu_send_message(uint8_t cmd, uint8_t payload[], uint8_t payload_length)
{
    LOG(LOG_EVENT, "%s: cmd 0x%02X, len %d\n", __func__, cmd, payload_length);
    
    uint8_t msg_buf[MAX_SEND_BUFFER_LENGTH];
    
    msg_buf[E_MAGIC1] = 0x55;
    msg_buf[E_MAGIC2] = 0xaa;
    msg_buf[E_VERSION] = 0x00;
    msg_buf[E_CMD] = cmd;
    tuya_mcu_set_payload_length(msg_buf, payload_length);
    tuya_mcu_set_payload(msg_buf, payload, payload_length);
    tuya_mcu_set_checksum(msg_buf);
    
    if (tuya_mcu_message_is_valid(msg_buf) == true)
    {
        serial_write(msg_buf, tuya_mcu_get_msg_length(msg_buf));
    }
    LOG(LOG_FLOW, "%s: End\n", __func__);
    
}

void tuya_mcu_updateWifiState()
{
    LOG(LOG_FLOW, "%s:\n", __func__);
    
    static uint32_t timeLastSend = 0;
    uint32_t timeNow = tuya_mcu_get_millis();
    
    if ((timeNow - timeLastSend) > 1000)
    {
        timeLastSend = timeNow;
        
        WifiState_t newState = WIFI_STATE_CONNECT_FAILED;
        
        uint8_t status = sdk_wifi_station_get_connect_status();
        
        switch (status)
        {
                
            case STATION_NO_AP_FOUND:
            case STATION_CONNECT_FAIL:
                newState = WIFI_STATE_CONNECT_FAILED;
                break;
            case STATION_GOT_IP:
                newState = WIFI_STATE_CONNECTED_WITH_INTERNET;
                break;
            default:
                newState = WIFI_STATE_CONNECT_FAILED;
                break;
                
        }
        
        tuya_mcu_setWifiState(newState);
        LOG(LOG_EVENT, "%s: wifi state %d\n", __func__, newState);
    }
}



void tuya_mcu_sendHeartbeat()
{
    LOG(LOG_FLOW, "%s:\n", __func__);
    
    tuya_mcu_send_cmd(MSG_CMD_HEARTBEAT);
}



void tuya_mcu_setWifiState(WifiState_t newState)
{
    LOG(LOG_WIFI, "%s: current state: %d, new state: %d\n", __func__, wifiState, newState);
    
    if (wifiState != newState || sendWifiStateMsg == true)
    {
        wifiState = newState;
        
        sendWifiStateMsg = true;
        
        if (gotWifiMode == true)
        {
            payload[0] = (uint8_t)wifiState;
            tuya_mcu_send_message (MSG_CMD_REPORT_WIFI_STATUS, payload, 1);
            sendWifiStateMsg = false;
        } else
        {
            LOG(LOG_WIFI, "%s: Not gotWifiMode\n", __func__);
        }
        
    }
    
}


void tuya_mcu_process_message(uint8_t msg[])
{
    LOG(LOG_EVENT, "\n\n%s:\n", __func__);
    
    if (!tuya_mcu_message_is_valid(msg))
    {
        tuya_mcu_print_message (msg, false);
        return;
    }
    
    uint8_t cmd = tuya_mcu_get_command(msg);
    LOG(LOG_EVENT, "%s: CMD: 0x%02X\n", __func__, cmd);
    
    mcuProtocolVersion = tuya_mcu_get_version(msg);
    uint16_t payload_length = tuya_mcu_get_payload ( msg, payload);
    
    switch(cmd)
    {
        case MSG_CMD_HEARTBEAT:
        {
            LOG(LOG_EVENT, "%s: Heartbeat\n", __func__);
            if (1 == payload_length)
            {
                if (payload[0] == 1){
                    
                    LOG(LOG_EVENT, "%s: MCU Heartbeat True\n", __func__);
                    
                    if (!gotHeartbeat)
                    {
                        gotHeartbeat = true;
                        HeartbeatDelay = 10000;
                        mcu_init_stage = 2;
                        
                        if (!gotProdKey)
                        {
                            tuya_mcu_send_cmd(MSG_CMD_QUERY_PROD_INFO);
                        } else {
                            LOG(LOG_ERR, "%s: Weird, got heartbeat but already have prod key\n", __func__);
                        }
                    }
                } else {
                    if (gotHeartbeat == true){
                        gotHeartbeat = false;
                        HeartbeatDelay = 3000;
                        gotProdKey = false;
                        mcu_init_stage = 1;
                        LOG(LOG_ACTION, "%s: MCU HeartBeat 0, resetting\n", __func__);
                    } else {
                        gotHeartbeat = true;
                        mcu_init_stage = 2;
                        if (!gotProdKey)
                        {
                            tuya_mcu_send_cmd(MSG_CMD_QUERY_PROD_INFO);
                        } else {
                            LOG(LOG_ERR, "%s: Weird, got heartbeat but already have prod key\n", __func__);
                        }
                    }
                }
            } else {
                LOG(LOG_ERR, "%s: INVALID heartbeat length: %d\n", __func__, payload_length);
            }
            break;
        }
        case MSG_CMD_QUERY_PROD_INFO:
        {
            LOG(LOG_EVENT, "%s: Received Query Prod Info\n", __func__);
            tuya_mcu_print_message ( msg, true);
            
            gotProdKey = true;
            
            if (!gotWifiMode) {
                if (mcu_init_stage == 2) mcu_init_stage = 3;
                tuya_mcu_send_cmd (MSG_CMD_QUERY_WIFI_MODE);
            } else {
                LOG(LOG_ERR, "%s: Weird, got prod key but already have wifi mode\n", __func__);
            }
            break;
        }
        case MSG_CMD_QUERY_WIFI_MODE:
        {
            LOG(LOG_EVENT, "%s: Received Query Wifi mode\n", __func__);
            
            gotWifiMode = true;
            
            if ( payload_length== 2)
            {
                uint8_t wifi_indicator_pin = payload[0];
                uint8_t reset_pin = payload[1];
                wifiMode = WIFI_MODE_WIFI_PROCESSING;
                LOG(LOG_EVENT, "%s: MCU wifi pin: %d, reset pin: %d\n", __func__, wifi_indicator_pin , reset_pin );
                canQuery = true;
                tuya_mcu_send_cmd (MSG_CMD_QUERY_DEVICE_STATUS);
                if (mcu_init_stage == 3) mcu_init_stage=5;
            }
            else
            {
                LOG(LOG_EVENT, "%s: Cooperative mode\n", __func__);
                wifiMode = WIFI_MODE_COOPERATIVE_PROCESSING;
                sendWifiStateMsg = true;
                tuya_mcu_updateWifiState();
                if (mcu_init_stage == 3) mcu_init_stage=4;
            }
            
            break;
        }
        case MSG_CMD_REPORT_WIFI_STATUS:
        {
            LOG(LOG_EVENT, "%s: Received Report WiFi status\n", __func__);
            canQuery = true;
            tuya_mcu_send_cmd (MSG_CMD_QUERY_DEVICE_STATUS);
            if (mcu_init_stage == 4) mcu_init_stage=5;
            break;
        }
        case MSG_CMD_RESET_WIFI_SWITCH_NET_CFG:
        {
            LOG(LOG_ACTION, "%s: Reset Wifi\n", __func__);
            tuya_mcu_send_cmd (MSG_CMD_RESET_WIFI_SWITCH_NET_CFG);
            break;
        }
        case MSG_CMD_DP_STATUS:
        {
            LOG(LOG_EVENT, "%s: DP Status\n", __func__);
            if (tuya_mcu_get_payload_length(msg) > 0)
            {
                tuya_device_handleDPStatusMsg(msg);
            }
            if (mcu_init_stage==5)
            {
                mcu_init_stage = 6;
                LOG(LOG_ACTION, "%s: Initialization Phase complete\n", __func__);
            }
            break;
        }
        case MSG_CMD_OBTAIN_LOCAL_TIME:
        {
            LOG(LOG_EVENT, "%s: Obtain Local Time\n", __func__);
            tuya_mcu_sendTime(timeAvailable);
            break;
        }
        default:
            LOG(LOG_ERR, "%s: unknown command: 0x%02X\n", __func__, cmd);
            break;
    }
}


/*
 void tuya_mcu_processRx()
 {
 printf ("%s: \n", __func__);
 bool hasMsg = false;
 if (serial_available())
 {
 while (serial_available())
 {
 hasMsg = tuya_mcu_msg_buffer_addbyte(serial_read(), msg);
 if (hasMsg)
 {
 tuya_mcu_process_message(msg);
 printf ( " MCU Init Stage: %d ", mcu_init_stage );
 }
 }
 }
 printf (":End\n");
 }
 */


void tuya_mcu_processRx()
{
    static uint8_t received_bytes = 0;
    uint8_t offset = 0;
    
    LOG(LOG_EVENT, "\n\n%s:\n", __func__);
    
    while (serial_available() > 0 && received_bytes < MAX_RECEIVE_BUFFER_LENGTH )
    {
        msg[received_bytes] = serial_read();
        received_bytes++;
    }
    LOG(LOG_EVENT, "%s: Received %d bytes\n", __func__, received_bytes);

    while  ((received_bytes - offset) >= TUYA_MCU_HEADER_SIZE ){
        
        if (msg[offset+E_MAGIC1] != 0x55) {
            LOG(LOG_ERR, "%s: Dropping 0x%02X\n", __func__, msg[offset+E_MAGIC1]);
            offset++;
            continue;
        }
        if (msg[offset+E_MAGIC2] != 0xAA) {
            LOG(LOG_ERR, "%s: Dropping 0x%02X\n", __func__, msg[offset+E_MAGIC2]);
            offset++;
            continue;
        }
        
        
        uint16_t message_length = tuya_mcu_get_msg_length (&msg[offset]);

        
        if (message_length > MAX_RECEIVE_BUFFER_LENGTH) {
            LOG(LOG_ERR, "%s: message length %d exceeds buffer, scanning forward\n", __func__, message_length);
            offset++;
            continue;
        }
        if ((offset + message_length) > received_bytes) {
            LOG(LOG_FLOW, "%s: waiting for %d more bytes\n", __func__, offset + message_length - received_bytes);
            break;
        }
        
        
        uint8_t checksum = tuya_mcu_calc_checksum(&msg[offset]);
        if (checksum != msg[offset+message_length-1]){
            LOG(LOG_ERR, "%s: invalid checksum, calculated %d, received %d, length: %d\n", __func__, checksum, (uint8_t) msg[offset+message_length-1], message_length );
            LOG(LOG_ERR, "%s: skipping message\n", __func__);
            offset += message_length;
            continue;
        }
        
        LOG(LOG_EVENT, "%s: Message length: %d, offset: %d\n", __func__, message_length, offset);
        tuya_mcu_process_message(&msg[offset]);
        offset += message_length;
        LOG(LOG_EVENT, "%s: MCU Init Stage: %d\n", __func__, mcu_init_stage);

    }
    
    received_bytes -= offset ;
    
    if (received_bytes > 0){
        LOG(LOG_FLOW, "%s: Bytes left in buffer: %d, Bytes processed: %d\n", __func__, received_bytes, offset);
        memmove(msg, msg + offset, received_bytes);
        memset(msg + received_bytes, 0, offset);
    }
    LOG(LOG_FLOW, "%s: End\n\n", __func__);
    
}



void tuya_mcu_loop(void *args)
{
    
    //tuya_mcu_setWifiState(4);
    //tuya_mcu_send_cmd(MSG_CMD_QUERY_PROD_INFO);
    
    while (1) {
        tuya_mcu_sendHeartbeat();
        tuya_mcu_updateWifiState();
        tuya_mcu_sendTime(timeAvailable);
        tuya_mcu_processRx();
        vTaskDelay(HeartbeatDelay/ portTICK_PERIOD_MS);
    }
}

