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

/* header definition ******************************************************** */
#ifndef XDK110_SENDDATAOVERUDP_H_
#define XDK110_SENDDATAOVERUDP_H_

/* local interface declaration ********************************************** */

/* local type and macro definitions */
typedef enum returnTypes_e
{
    STATUS_NOT_OK,
    STATUS_OK,
    SOCKET_ERROR,
    SEND_ERROR
} returnTypes_t;

#define TIMERBLOCKTIME UINT32_MAX             /**< Macro used to define block time of a timer*/
#define ZERO    UINT8_C(0)  			            /**< Macro to define value zero*/
#define TIMER_AUTORELOAD_ON             UINT32_C(1)             /**< Auto reload of timer is enabled*/
#define TIMER_AUTORELOAD_OFF            UINT32_C(0)             /**< Auto reload of timer is disabled*/

/** Network configurations */
#define WLAN_CONNECT_WPA_SSID                "LGLTE_6551"         /**< Macros to define WPA/WPA2 network settings */
#define WLAN_CONNECT_WPA_PASS                "starthack"      /**< Macros to define WPA/WPA2 network settings */
#define BUFFER_SIZE        UINT8_C(7)
/** IP addressed of server side socket.Should be in long format, E.g: 0xc0a82177 == 192.168.33.119 */
#define SERVER_IP         UINT32_C(0xC0A82BC6)
#define SERVER_PORT        UINT16_C(6666)           /**< Port number on which server will listen */

#define ACCEL_RECEIVELENGTH             UINT8_C(30)                      /**< Receive length for BLE */

/* local function prototype declarations */
/**
 *  @brief
 *      Function to connect to wifi network and obtain IP address
 *
 *  @param [in ] xTimer
 */
static void wifiConnectGetIP(xTimerHandle xTimer);
/**
 *  @brief
 *      Function to periodically send data over WiFi as UDP packets. This is run by an Auto-reloading timer.
 *
 *  @param [in ] xTimer
 */
static void wifiSend(xTimerHandle xTimer);

/* local module global variable declarations */

/* local inline function definitions */

#endif /* XDK110_SENDDATAOVERUDP_H_ */

/** ************************************************************************* */
