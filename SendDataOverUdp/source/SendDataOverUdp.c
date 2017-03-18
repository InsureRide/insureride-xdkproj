/**
* Licensee agrees that the example code provided to Licensee has been developed and released by Bosch solely as an example to be used as a potential reference for Licensee�s application development. 
* Fitness and suitability of the example code for any use within Licensee�s applications need to be verified by Licensee on its own authority by taking appropriate state of the art actions and measures (e.g. by means of quality assurance measures).
* Licensee shall be responsible for conducting the development of its applications as well as integration of parts of the example code into such applications, taking into account the state of the art of technology and any statutory regulations and provisions applicable for such applications. Compliance with the functional system requirements and testing there of (including validation of information/data security aspects and functional safety) and release shall be solely incumbent upon Licensee. 
* For the avoidance of doubt, Licensee shall be responsible and fully liable for the applications and any distribution of such applications into the market.
* 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are 
* met:
* 
*     (1) Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer. 
* 
*     (2) Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.  
*     
*     (3)The name of the author may not be used to
*     endorse or promote products derived from this software without
*     specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR 
*  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
*  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
*  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
*  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
*  POSSIBILITY OF SUCH DAMAGE.
*/
//lint -esym(956,*) /* Suppressing "Non const, non volatile static or external variable" lint warning*/

/* module includes ********************************************************** */

/* system header files */
#include <stdio.h>
#include <time.h>

/* additional interface header files */
#include "simplelink.h"
#include "BCDS_Basics.h"
#include "BCDS_Assert.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "BCDS_WlanConnect.h"
#include "BCDS_NetworkConfig.h"
#include <Serval_Types.h>
#include <Serval_Basics.h>
#include <Serval_Ip.h>

//#include "BCDS_Accelerometer.h"
#include "BCDS_Retcode.h"
#include "XdkSensorHandle.h"
#include "XdkBoardHandle.h"
#include "XdkUsbResetUtility.h"
#include "led.h"

/* own header files */
#include "SendDataOverUdp.h"

#define CALIBRATION_DONE UINT8_C(1) /**<Represents calibration done status */
#define CALIBRATION_NOT_DONE UINT8_C(0) /**<Represents calibration not done status */

/* constant definitions ***************************************************** */

/* local variables ********************************************************** */

/**
 * This buffer holds the data to be sent to server via UDP
 * */
static int32_t bsdBuffer_mau[BUFFER_SIZE] = { (int32_t) ZERO };

static int32_t ride_id = 0;
static int32_t timestamp_delta = 0;

static LED_handle_tp ledHandle = (LED_handle_tp) NULL;

/**
 * Timer handle for connecting to wifi and obtaining the IP address
 */
xTimerHandle wifiConnectTimerHandle_gdt = NULL;
/**
 * Timer handle for periodically sending data over wifi
 */
xTimerHandle wifiSendTimerHandle = NULL;

/* global variables ********************************************************* */

/* inline functions ********************************************************* */

static int32_t x_offset = 0;
static int32_t y_offset = 0;
static int32_t z_offset = 0;

static int MS_SEND_INTERVAL = 1000;

/* local functions ********************************************************** */
/**
 *  @brief
 *      Function to initialize the wifi network send application. Create timer task
 *      to start WiFi Connect and get IP function after one second. After that another timer
 *      to send data periodically.
 */
void init(void) {
	uint32_t Ticks = MS_SEND_INTERVAL;
    if (Ticks != UINT32_MAX) /* Validated for portMAX_DELAY to assist the task to wait Infinitely (without timing out) */ {
        Ticks /= portTICK_RATE_MS;
    }
    if (UINT32_C(0) == Ticks) /* ticks cannot be 0 in FreeRTOS timer. So ticks is assigned to 1 */ {
        Ticks = UINT32_C(1);
    }

    ledHandle = LED_create(gpioYellowLed_Handle, GPIO_STATE_OFF);
	/* assertion Reason : "LED creation failed" */
	assert(ledHandle != NULL);
	assert(LED_setState(ledHandle, LED_SET_ON) == LED_ERROR_OK);

    /* Return value for Accel Sensor */
	Retcode_T accelReturnValue = (Retcode_T) RETCODE_FAILURE;

    /*Initialize accel Sensor */
	accelReturnValue = Accelerometer_init(xdkAccelerometers_BMA280_Handle);

	if (RETCODE_OK == accelReturnValue) {
		printf("accel initialised\r\n");
	} else {
		printf("accel init failed...\r\n");
		assert(false);
	}

	/* create timer task*/
	wifiConnectTimerHandle_gdt = xTimerCreate((char * const ) "wifiConnect", Ticks, TIMER_AUTORELOAD_OFF, NULL, wifiConnectGetIP);
	wifiSendTimerHandle = xTimerCreate((char * const ) "wifiSend", Ticks, TIMER_AUTORELOAD_ON, NULL, wifiSend);

	if ((wifiConnectTimerHandle_gdt != NULL) && (wifiSendTimerHandle != NULL)) {
		/*start the wifi connect timer*/
		if ( xTimerStart(wifiConnectTimerHandle_gdt, TIMERBLOCKTIME) != pdTRUE) {
			assert(false);
		}
	} else {
		/* Assertion Reason: "Failed to create timer task during initialization"   */
		assert(false);
	}

	int32_t calib_x_sum = 0;
	int32_t calib_y_sum = 0;
	int32_t calib_z_sum = 0;
	int32_t calib_cnt = 10;

	for (int i = 0; i < calib_cnt; i++) {
		vTaskDelay(100);

		if (i % 10 == 0) {
			printf("calibrating...\r\n");
		}

		Accelerometer_XyzData_T getaccelData = { INT32_C(0), INT32_C(0), INT32_C(0) };
		assert(Accelerometer_readXyzGValue(xdkAccelerometers_BMA280_Handle, &getaccelData) == RETCODE_OK);

		calib_x_sum += getaccelData.xAxisData;
		calib_y_sum += getaccelData.yAxisData;
		calib_z_sum += getaccelData.zAxisData;
	}

	if (ride_id == 0) {
		ride_id = (calib_x_sum + calib_y_sum + calib_z_sum) / calib_cnt;
	}

	x_offset = (int32_t) (calib_x_sum * 1.0f / calib_cnt);
	y_offset = (int32_t) (calib_y_sum * 1.0f / calib_cnt);
	z_offset = (int32_t) (calib_z_sum * 1.0f / calib_cnt);

	assert(LED_setState(ledHandle, LED_SET_OFF) == LED_ERROR_OK);
	printf("calibration done; xo=%ld yo=%ld zo=%ld\r\n", x_offset, y_offset, z_offset);
}

/**
 * @brief This is a template function where the user can write his custom application.
 */
void appInitSystem(xTimerHandle xTimer) {
    BCDS_UNUSED(xTimer);
    init();
}

bool connected = false;
SlSockAddrIn_t Addr;
uint16_t AddrSize = (uint16_t) ZERO;
int16_t SockID = (int16_t) ZERO;
int16_t Status = (int16_t) ZERO;

/**
 *  @brief
 *      Function to periodically send data over WiFi as UDP packets. This is run as an Auto-reloading timer.
 *
 *  @param [in ] xTimer - necessary parameter for timer prototype
 */
static void wifiSend(xTimerHandle xTimer)
{
	//printf("wifiSend\r\n");
    BCDS_UNUSED(xTimer);

    if (!connected) {
    	printf("not connected, connecting...\r\n");
    	Addr.sin_family = SL_AF_INET;
		Addr.sin_port = sl_Htons((uint16_t) SERVER_PORT);
		Addr.sin_addr.s_addr = sl_Htonl(SERVER_IP);
		AddrSize = sizeof(SlSockAddrIn_t);

		SockID = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, (uint32_t) ZERO); /**<The return value is a positive number if successful; other wise negative*/
		if (SockID < (int16_t) ZERO)
		{
			/* error case*/
			printf("err sockid<zero\r\n");
			assert(false);
		}

		connected = true;
		printf("  connected\r\n");
    } else {
    	//printf("already connected\r\n");
    }

    static int32_t counter = INT32_C(0);
    static int32_t ride_counter = INT32_C(0);

    /* Reading calibrated accelerometer m/s2 value */
    Accelerometer_XyzData_T getaccelData = { INT32_C(0), INT32_C(0), INT32_C(0) };
    Retcode_T advancedApiRetValue = Accelerometer_readXyzGValue(xdkAccelerometers_BMA280_Handle, &getaccelData);
	if (RETCODE_OK == advancedApiRetValue)
	{
		int32_t x = getaccelData.xAxisData - x_offset,
				y = getaccelData.yAxisData - y_offset,
				z = getaccelData.zAxisData - z_offset;
		printf("got accel data x=%ldmg y=%ldmg z=%ldmg\r\n", x, y, z);
		bsdBuffer_mau[2] = x;
		bsdBuffer_mau[3] = y;
		bsdBuffer_mau[4] = z;
	} else {
		printf("accel data errr\r\n");
		assert(false);
	}

	bsdBuffer_mau[0] = timestamp_delta;
	bsdBuffer_mau[1] = counter;
	bsdBuffer_mau[5] = ride_counter;
	bsdBuffer_mau[6] = ride_id;

	/**<The return value is a number of characters sent;negative if not successful*/
	Status = sl_SendTo(SockID, bsdBuffer_mau, BUFFER_SIZE * sizeof(int32_t), (uint32_t) ZERO, (SlSockAddr_t *) &Addr, AddrSize);

	/*Check if 0 transmitted bytes sent or error condition*/
	if (Status < (int16_t) ZERO) {
		printf("errr status<zero statuscode=%d \r\n", Status);
		assert(false);
	} else {
		printf("sent %d bytes\r\n", Status);
	}

	//Status = sl_Close(SockID);
	//if (Status < 0)
	//{
	//	printf("errrr status<0\r\n");
	//	assert(false);
	//}

	counter++;
	printf("ts=%ldms counter=%ld ride_counter=%ld ride_id=%ld\r\n \r\n", timestamp_delta, counter, ride_counter, ride_id);

	timestamp_delta += MS_SEND_INTERVAL;
}

/**
 *  @brief
 *      Function to connect to wifi network and obtain IP address
 *
 *  @param [in ] xTimer
 */
static void wifiConnectGetIP(xTimerHandle xTimer)
{
    BCDS_UNUSED(xTimer);

    NetworkConfig_IpSettings_T myIpSettings;
    memset(&myIpSettings, (uint32_t) 0, sizeof(myIpSettings));
    char ipAddress[PAL_IP_ADDRESS_SIZE] = { 0 };
    Ip_Address_T* IpaddressHex = Ip_getMyIpAddr();
    WlanConnect_SSID_T connectSSID;
    WlanConnect_PassPhrase_T connectPassPhrase;
    Retcode_T ReturnValue = (Retcode_T)RETCODE_FAILURE;
    int32_t Result = INT32_C(-1);

    if (RETCODE_OK != WlanConnect_Init())
    {
        printf("Error occurred initializing WLAN \r\n ");
        return;
    }

    printf("Connecting to %s \r\n ", WLAN_CONNECT_WPA_SSID);

    connectSSID = (WlanConnect_SSID_T) WLAN_CONNECT_WPA_SSID;
    connectPassPhrase = (WlanConnect_PassPhrase_T) WLAN_CONNECT_WPA_PASS;
    ReturnValue = NetworkConfig_SetIpDhcp(NULL);
    if (ReturnValue)
    {
        printf("Error in setting IP to DHCP\n\r");
        return;
    }

    if (RETCODE_OK == WlanConnect_WPA(connectSSID, connectPassPhrase, NULL))
    {
        ReturnValue = NetworkConfig_GetIpSettings(&myIpSettings);
        if (RETCODE_OK == ReturnValue)
        {
            *IpaddressHex = Basics_htonl(myIpSettings.ipV4);
            Result = Ip_convertAddrToString(IpaddressHex, ipAddress);
            if (Result < 0)
            {
                printf("Couldn't convert the IP address to string format \r\n ");
                return;
            }
            printf("Connected to WPA network successfully. \r\n ");
            printf(" Ip address of the device: %s \r\n", ipAddress);
        }
        else
        {
            printf("Error in getting IP settings\n\r");
            return;
        }
    }
    else
    {
        printf("Error occurred connecting %s \r\n ", WLAN_CONNECT_WPA_SSID);
        return;
    }

    printf("ok now set timer wifisend\r\n");

    /* After connection start the wifi sending timer*/
    if (xTimerStart(wifiSendTimerHandle, TIMERBLOCKTIME) != pdTRUE) {
    	printf("  nooooo\r\n");
        assert(false);
    } else {
    	printf("  timer set ok\r\n");
    }
}

/* global functions ********************************************************* */

/** ************************************************************************* */
