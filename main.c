
/*****************************************************************************************
*                                INCLUDES - Start
*****************************************************************************************/

//Standard includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

// free-rtos/TI-rtos include
#include "osi.h"

// XDC module Headers
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

// BIOS module Headers
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

//driverlib includes
#include "hw_ints.h"
#include "hw_types.h"
#include "interrupt.h"
#include "hw_memmap.h"
#include "timer.h"
#include "uart.h"
#include "prcm.h"
#include "utils.h"
#include "hw_uart.h"
#include "hw_gpio.h"
#include "pin.h"

//#include "hw_apps_rcm.h"
#include "udma.h"
#include "gpio.h"
#include "rom.h"
#include "rom_map.h"

//Simplelink includes
#include "simplelink.h"

//middleware includes
#include "uart_drv.h"

//common interface includes
#include "timer_if.h"
#include "gpio_if.h"
#include "button_if.h"
#include "udma_if.h"
#include "uart_if.h"
#include "common.h"
#include "pinmux.h"

/*****************************************************************************************
*                                  INCLUDES - End
*****************************************************************************************/



/*****************************************************************************************
*                                  DEFINES - Start
*****************************************************************************************/

#define APP_NAME                    "N.L. Thesis. Telemetry for Quadcopter/Drone project"
#define APPLICATION_VERSION         "1.0"
#define OSI_TASK_STACK_SIZE         4096

// Network Configuration values
#define UDP_PORT                	14550
#define TCP_PORT			    	5760

//Values for below macros set the ping properties
#define PING_INTERVAL               1000    // In msecs
#define PING_TIMEOUT                3000    // In ms
#define PING_PKT_SIZE               20      // In Bytes(B)
#define NO_OF_ATTEMPTS              5
#define PING_FLAG                   0

// Size of Transfer Buffers, UDMA_SIZE_8
// UDMA_SIZE_# # should be increased as buffer sizes increase,
// Greater UDMA_SIZE_#'s yield higher throughput especiallly for ping-pong type transfers
// The MAVLINK msg len is 17B
#define UDMA_RXCH_BUF_SIZE    	    17      // In Bytes
#define UDMA_TXCH_BUF_SIZE		    17	    // In Bytes

// APIs parameters
#define SL_STOP_TIMEOUT             200     // in msecs
#define UARTA1_BAUD_RATE		    57600

// RTOS
#define SL_SPAWN_TASK_PRIORITY     	9
#define SEND_TASK_PRIORITY          4
#define RECEIVE_TASK_PRIORITY       2
#define GREEN_LIGHT_PRIORITY        1
#define INACTIVE_STATE_PRIORITY     -1

// System
#define SYSTEM_CLOCK                80000000
#define UARTA1_BAUD_RATE			57600

// MISC
#define GOOD_COUNT				    200
#define NULL						0
#define SUCCESS                     0
#define FALSE						0
#define false						0
#define TRUE						1
#define true						1
#define FAILURE                     -1
#ifndef NOTERM
#define UART_PRINT   				Report
//#define ERR_PRINT(x) Report("Error [%d] at line [%d] in function [%s]  \n\r",x,__LINE__,__FUNCTION__)
#define LOOP_FOREVER() \
            {\
                while(1); \
            }

//Application status/error codes
typedef enum{
	LAN_CONNECTION_FAILED = - 0x7D0,
	SOCKET_CREATE_ERROR = LAN_CONNECTION_FAILED - 1,
	CLIENT_CONNECTION_FAILED = SOCKET_CREATE_ERROR - 1,
	DEVICE_NOT_IN_STATION_MODE = CLIENT_CONNECTION_FAILED - 1,
	RECV_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    SEND_ERROR = RECV_ERROR - 1,
	BIND_ERROR = SEND_ERROR - 1,
    LISTEN_ERROR = BIND_ERROR - 1,
    SOCKET_OPT_ERROR = LISTEN_ERROR - 1,
    CONNECT_ERROR = SOCKET_OPT_ERROR - 1,
    ACCEPT_ERROR = CONNECT_ERROR - 1,
	INTERRUPT_ERROR = ACCEPT_ERROR - 1,
	STATUS_CODE_MAX = - 0xBBB
}e_AppStatusCodes;
#endif

// Interrupt vector table entry, depending on the IDE in use
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

/*****************************************************************************************
*                                 DEFINES - End
*****************************************************************************************/



/*****************************************************************************************
*                              GLOBAL VARIABLES - Start
*****************************************************************************************/

//SimpleLink Status - response on various async events -
volatile unsigned long  g_ulStatus = 0;

// Network/Communication global variables
unsigned long  		    g_ulDestIp 		    = 0;  // Connected client's IP address
unsigned long  		    g_ulPingPacketsRecv = 0;
volatile static _Bool   g_bBufTurn 		    = TRUE;
_Bool 				    g_bUdpOrTcp 		= TRUE;
int 				    g_iUdpSockHandle    = 0; 	// UDP socket descriptor (handle)
int 				    g_iTcpSockHandle    = 0; 	// TCP socket handle
#ifndef TCP_CLIENT
int             		g_iNewTcpSockID     = SL_EAGAIN;
#endif
SlSockAddrIn_t  	    g_slSocketParam; // Contains the network parameters of the connected host
int 				    g_iSockBlockSize 	= sizeof(SlSockAddrIn_t);

// The three DMA buffers used for delivering the data-packets
char 				    g_cUart1RxBufA[UDMA_RXCH_BUF_SIZE] = "";
char 				    g_cUart1RxBufB[UDMA_RXCH_BUF_SIZE] = "";
char 				    g_cUart1TxBuf[UDMA_TXCH_BUF_SIZE]  = "";

// Counts number of buffer refills and transmissions, receptions
unsigned long 		    g_ulRxABufFillCount = 0;
unsigned long 	  		g_ulRxBBufFillCount = 0;
unsigned long 	  		g_ulTxBufFillCount  = 0;
unsigned long 			g_ulTxCount = 0;
unsigned long 			g_ulRxCount = 0;

// RTOS
OsiTaskHandle   UdpHandle, TcpHandle, GreenLightHandle;
OsiSyncObj_t    g_sendSignal; 	// Semaphore variable

// Misc
//char g_cErrorBuffer[100];
#ifndef COMPUTER
_Bool g_bButtonPress    = FALSE;
#endif

/*****************************************************************************************
*                           GLOBAL VARIABLES - End
*****************************************************************************************/



/*****************************************************************************************
*                      LOCAL FUNCTION PROTOTYPES - Start
*****************************************************************************************/

// Event Handlers/Callbacks/Hook function declarations
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
								  SlHttpServerResponse_t *pHttpResponse);
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent);
void SimpleLinkWlanEventHandler (SlWlanEvent_t *pWlanEvent);
void SimpleLinkNetAppEventHandler (SlNetAppEvent_t *pNetAppEvent);
void SimpleLinkPingReport(SlPingReport_t *pPingReport);
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock);

//initialization/configuration function declarations
static void BoardInit(void);
void PinMuxConfig(void);
static void DisplayBanner(char * AppName);
void InitialSetup( void * pvParameters);
static long ConfigureSimpleLinkToDefaultState();
static int SetAPMode(int iMode);
static int PingTest(unsigned long ulIpAddr);
int UdpCLientServerConfig(unsigned short usPort);
int TcpCLientServerConfig(unsigned short usPort);
int Uart1ConfigDmaTransfer(void);

#ifndef COMPUTER
void DecisionMaking(void);
int TimerTimeoutConfig( unsigned long ulTimerBase, unsigned long ulTimerPeriph,
	unsigned long ulTimerMode, unsigned long ulPrescaleVal, unsigned short usIntPriority,
	void (*TimerInterruptHandler) (void), unsigned long ulTimeoutVal, unsigned long ulTimer );
static unsigned char GetTimerInterruptNum(unsigned long ulTimerBase,
		unsigned long ulTimer);
int ButtonSetup ( void (*SW3ButtonInterruptHandler) (void),
		void (*SW2ButtonInterruptHandler) (void) );
#endif

//RTOS TASKS
void TransmitReceiveUdpPackets(void * pvParameters);
void SendRecvTcpStream(void * pvParameters);
void GreenLight(void * pvParameters);

//Interrupts
void Uart1IntHandler(void);
#ifndef COMPUTER
void Sw3InterruptHandler(void);
void Sw2InterruptHandler(void);
void TimerA0InterruptHandler(void);
#endif

#ifndef NOTERM
// Check the error, handle and display it through UART
/*#define ASSERT_ON_ERROR(error_code)\
            {\
                 if(error_code < 0) \
                   {\
                        sprintf(g_cErrorBuffer,"Error [%d] at line [%d] in "\
"function [%s]", error_code,__LINE__,__FUNCTION__);\
                        UART_PRINT(g_cErrorBuffer);\
                        UART_PRINT("\n\r");\
                        return error_code;\
                 }\
            }*/
#endif

/*****************************************************************************************
*                         LOCAL FUNCTION PROTOTYPES - End
*****************************************************************************************/


/*****************************************************************************************
*        FreeRTOS User Hook/Callback Functions enabled in FreeRTOSConfig.h
*****************************************************************************************/

#ifdef USE_FREERTOS
/*****************************************************************************************
  *brief      Application defined hook (or callback) function - assert
  *			  Handle Asserts here

  *param[in]  pcFile - Pointer to the File Name
  *param[in]  ulLine - Line Number
  *return     none
*****************************************************************************************/
void vAssertCalled( const char *pcFile, unsigned long ulLine )
{
    while(1)
    {
    }
}

/*****************************************************************************************
  *brief      Application defined idle task hook
  *			  Handle Idle Hook for Profiling, Power Management etc

  *param      none
  *return     none
*****************************************************************************************/
void vApplicationIdleHook( void)
{
}

/*****************************************************************************************
  *brief      Application defined malloc failed hook
  *			  Handle Memory Allocation Errors

  *param      none
  *return     none
*****************************************************************************************/
void vApplicationMallocFailedHook()
{
    while(1)
    {
    }
}

/*****************************************************************************************
  *brief      Application defined stack overflow hook
  *			  Handle FreeRTOS Stack Overflow

  *param      none
  *return     none
*****************************************************************************************/
void vApplicationStackOverflowHook(OsiTaskHandle *pxTask,
                                   signed char *pcTaskName)
{
    while(1)
    {
    }
}
#endif //USE_FREERTOS





/*****************************************************************************************
******************************************************************************************
*                    EVENT HANDLING FUNCTIONS - Start
******************************************************************************************
*****************************************************************************************/



/*****************************************************************************************
  *brief        This function handles HTTP server events

  *param[in]    pServerEvent - Contains the relevant event information
  *param[in]    pServerResponse - Should be filled by the user with the
  *                                      relevant response information
  *return       None

  *warning      This function must be included or else the compiler goes nuts
*****************************************************************************************/

void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    // Unused in this app
}



/*****************************************************************************************
  *brief      This function handles General Events

  *param[in]  pDevEvent - Pointer to General Event Info
  *return     None
*****************************************************************************************/

// application port to SDK1.2.0 +
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    // Most of the general errors are not FATAL and are to be handled
    // appropriately by the application
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\r",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


/*****************************************************************************************
  *brief   On Successful completion of Wlan Connect, this function triggers
  *        Connection status to be set.

  *param   pSlWlanEvent pointer indicating Event type
  *return  None

  *note	   On connection events either to WLAN, or in AP mode, the status bit
  *		   g_ulStatus is set, and on disconnection event it is cleared
*****************************************************************************************/

void SimpleLinkWlanEventHandler(SlWlanEvent_t *pSlWlanEvent)
{
    switch(pSlWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {//we just need to monitor the g_ulStatus variable to check the status of the device
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pSlWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                UART_PRINT("Device disconnected from the AP on application's "
                            "request \n\r");
            }
            else
            {
                UART_PRINT("WARNING: Device disconnected from the AP on an ERROR..!!\n\r");
            }

        }
        break;

        // The following event is expected to be triggered
        case SL_WLAN_STA_CONNECTED_EVENT:
        {
            // when device is in AP mode and any client connects to device cc3xxx
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);/* g_ulStatus remains 2 */
        }
        break;

        // the client disconnecting from the device isn't meant to happen and
        // shouldn't happen in our application
        case SL_WLAN_STA_DISCONNECTED_EVENT:
        {
            // when client disconnects from device (AP)
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event\n\r");
        }
        break;
    }
}



/*****************************************************************************************
  *brief      This function handles network events such as IP acquisition, IP
  *           leased, IP released etc.

  *param[in]  pNetAppEvent - Pointer to NetApp Event Info
  *return     None
*****************************************************************************************/

void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        case SL_NETAPP_IPV6_IPACQUIRED_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);
        }/*g_ulStatus was 0, after IP acquired g_ulStatus=2, or 10 (in binary) */
        break;

        case SL_NETAPP_IP_LEASED_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);
            /* g_ulStatus = 2 again */
            // Assigns IP from the DHCP server
            g_ulDestIp = (pNetAppEvent)->EventData.ipLeased.ip_address;

            UART_PRINT("[NETAPP EVENT] IP Leased to Client: IP=%d.%d.%d.%d\n\r, ",
                        SL_IPV4_BYTE(g_ulDestIp,3), SL_IPV4_BYTE(g_ulDestIp,2),
                        SL_IPV4_BYTE(g_ulDestIp,1), SL_IPV4_BYTE(g_ulDestIp,0));
        }
        break;

        // Should not occur in our application
        case SL_NETAPP_IP_RELEASED_EVENT:
        {
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            UART_PRINT("[NETAPP EVENT] IP Released for Client: IP=%d.%d.%d.%d\n\r, ",
                        SL_IPV4_BYTE(g_ulDestIp,3), SL_IPV4_BYTE(g_ulDestIp,2),
                        SL_IPV4_BYTE(g_ulDestIp,1), SL_IPV4_BYTE(g_ulDestIp,0));
        }
        break;

        default:
        {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x]\n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}



/*****************************************************************************************
  *brief	  This function handles ping report events

  *param[in]  pPingReport - Ping report statistics
  *return     None
*****************************************************************************************/

void SimpleLinkPingReport(SlPingReport_t *pPingReport)
{
    SET_STATUS_BIT(g_ulStatus, STATUS_BIT_PING_DONE);
    /* After pinging g_ulStatus = 2, as before */
    g_ulPingPacketsRecv = pPingReport->PacketsReceived;
}



/*****************************************************************************************
  *brief      This function handles socket events indication
  *		      Used for TCP socket events

  *param[in]  pSock - Pointer to Socket Event Info
  *return     None
*****************************************************************************************/

void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    switch( pSock->Event )
    {
    	case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status)
            {
            	case SL_ECLOSE:
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n",
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default:
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

	default:
		UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n", pSock->Event);
		break;
	}
}

/*****************************************************************************************
*                      EVENT HANDLING FUNCTIONS - End
*****************************************************************************************/





/*****************************************************************************************
******************************************************************************************
*                   INITIALIZATION/SETUP FUNCTIONS - Start
******************************************************************************************
*****************************************************************************************/



/*****************************************************************************************
  *brief     This function puts the device in its default state. It cleans all the
  *          persistent settings stored in NVMEM (viz. connection profiles & policies,
  *          power policy etc). More specifically the function:
  *          - Sets the mode to STATION
  *          - Configures connection policy to Auto and AutoSmartConfig
  *          - Deletes all the stored profiles
  *          - Enables DHCP
  *          - Disables Scan policy
  *          - Sets Tx power to maximum
  *          - Sets power policy to normal
  *          - Unregister mDNS services
  *          - Remove all filters

  *param     none
  *return    On success, zero is returned. On error, negative is returned

  *CAUTION!  sl_Start must be called before any other simplelink API is used,
  *    	  	 or after sl_Stop is called for restarting the device, and if OS environment
  *    	  	 always after the corresponding RTOS scheduler has started.
*****************************************************************************************/

static long ConfigureSimpleLinkToDefaultState()
{
    long 								 lRetVal 		= -1;
    long 								 lMode   		= -1;
    unsigned char 						 ucVal 			= 1;
    unsigned char 						 ucConfigOpt 	= 0;
    unsigned char 						 ucConfigLen 	= 0;
    unsigned char 						 ucPower 		= 0;
    SlVersionFull   					 ver 			= {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    // sl_Start  initialize the communication interface, sets the enable pin of the device
    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode); // 0 for ROLE_STA = default mode

    // If the device is not in station-mode, try configuring it in station-mode
    if (ROLE_STA != lMode)
    {
        if (ROLE_AP == lMode)
        {
            // If the device is in AP mode, we need to wait for it to acquire an IP
            // address before doing anything else
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
            }
        }

        // Switch to STA role and restart
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        // Restart device to apply changes
        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again
        if (ROLE_STA != lRetVal)
        {
            // We don't want to proceed if the device is not coming up in STA-mode
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }

    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt,
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Deletes all stored profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    // Device in station-mode. Disconnect previous connection if any. The function returns
    // 0 if 'Disconnected done', negative number if already disconnected. Ignore other reurn-codes
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal)
    {
        // Wait
        while(IS_CONNECTED(g_ulStatus)) /* first encounter of g_ulStatus = 0
        	 there shouldn't be access to the loop */
        {
        // When using single threaded host-driver the following function
        // must be called within the loop
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
        }
    }

    // Enable DHCP client. Setting IP address by DHCP to FileSystem using WLAN sta mode
    // This is the system's default mode for acquiring an IP address after WLAN connection
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    g_ulStatus = 0; // staying safe

    return lRetVal; // Success
}



/*****************************************************************************************
  *brief     This function sets the WLAN mode as AP mode

  *param     iMode is the current mode of the device
  *return    sl_start return value(int).
*****************************************************************************************/

static int SetAPMode(int iMode)
{
	long lRetVal 	  = -1;
	char pcSsidName[] = "CC3200Thesis_N.L.";

    UART_PRINT("SSID: CC3200Thesis_N.L.\n\r");

	lRetVal = sl_WlanSetMode(ROLE_AP); // Sets AP mode
	ASSERT_ON_ERROR(lRetVal);

	lRetVal = sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SSID,
			strlen(pcSsidName), (unsigned char*) pcSsidName);
	ASSERT_ON_ERROR(lRetVal);

	UART_PRINT("Device is configured in AP mode\n\r");

	// Restart Network processor - Device has to be reset to apply the previous changes
	lRetVal = sl_Stop(SL_STOP_TIMEOUT);

	// reset status bits
	CLR_STATUS_BIT_ALL(g_ulStatus);

	return sl_Start(NULL, NULL, NULL);
}



/*****************************************************************************************
  *brief    CC3200 will try to ping the machine that has just connected to it

  *param    ulIpAddr is the ip address of the station which has connected to
  *         the device
  *return   0 if ping is successful, -1 for error
*****************************************************************************************/

static int PingTest(unsigned long ulIpAddr)
{
    signed long			 lRetVal = -1;
    SlPingStartCommand_t PingParams;
    SlPingReport_t 		 PingReport; // for statistics

    PingParams.PingIntervalTime = PING_INTERVAL;
    PingParams.PingSize = PING_PKT_SIZE;
    PingParams.PingRequestTimeout = PING_TIMEOUT;
    PingParams.TotalNumberOfAttempts = NO_OF_ATTEMPTS;
    PingParams.Flags = PING_FLAG;
    PingParams.Ip = ulIpAddr; // Client's ip address

    UART_PRINT("Running Ping Test...\n\r");
    // Pings the network host, by sending ICMP ECHO_REQUEST
    lRetVal = sl_NetAppPingStart((SlPingStartCommand_t*)&PingParams, SL_AF_INET,
                            (SlPingReport_t*)&PingReport, NULL);
    ASSERT_ON_ERROR(lRetVal);

    g_ulPingPacketsRecv = PingReport.PacketsReceived;

    if (g_ulPingPacketsRecv > 0 && g_ulPingPacketsRecv <= NO_OF_ATTEMPTS)
    {
      // LAN connection is successful
      UART_PRINT("Ping Test successful\n\r");
    }
    else // Problem with LAN connection
        ASSERT_ON_ERROR(LAN_CONNECTION_FAILED);

    return SUCCESS;
}



/*****************************************************************************************
  *brief    Start simplelink to default state. Assert AP WiFi mode, wait for the station
  *		    to connect to the device, run the ping test. Configure uDMA controller and
  *		    set socket parameters

  *param    pvparameters is the pointer to the list of parameters that can be
  *         passed to the task while creating it. Not in use here.
  *return   None

  *note     1) AP mode must use static IP settings.
  *		    2) All set functions require system restart for changes to take effect
  *		    3) CC3200 AP mode supports only one client/server connected at a time
  *		    4) This Task should not be disturbed from other tasks until it is complete
*****************************************************************************************/

void InitialSetup( void *pvParameters)
{
    int 			iPingResult 	= 0;
    unsigned char   ucDHCP;
    long 			lRetVal 	 	= -1;
    unsigned int 	uiUdpPortNum 	= UDP_PORT;
    unsigned int 	uiTcpPortNum 	= TCP_PORT;
	OsiReturnVal_e  xSemaphoreResult;


	// Suspend other tasks until this one is done
	Task_setPri(UdpHandle, INACTIVE_STATE_PRIORITY);
	Task_setPri(TcpHandle, INACTIVE_STATE_PRIORITY);
    Task_setPri(GreenLightHandle, INACTIVE_STATE_PRIORITY);
#ifndef COMPUTER
	// Lets the user make decision on the communication protocol based on button pressed
	DecisionMaking();
#endif


    // We could skip the following function if we are certain the device will start in
    // it's default state. - I'm not.
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0)
    {
        if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
            UART_PRINT("WARNING: Failed to configure the device in its default state.\n\r");
            sl_Stop(SL_STOP_TIMEOUT);
    }

    UART_PRINT("Device is configured in default state \n\r");

    lRetVal = sl_Start(NULL,NULL,NULL);
    if (lRetVal < 0)
    {
        UART_PRINT("WARNING: Failed to start the device \n\r");
        sl_Stop(SL_STOP_TIMEOUT);
    }

    // Configures the networking mode and ssid name for AP mode
    if(lRetVal != ROLE_AP)
    {
        if(SetAPMode(lRetVal) != ROLE_AP)
        {
            UART_PRINT("WARNING: Unable to set AP mode, exiting Application...\n\r");
            sl_Stop(SL_STOP_TIMEOUT);
        }
    }

    /* g_ulStatus should be 0 here at first. But then SimpleLinkNetAppEventHandler
       rises and sets the bit to decimal 2, which makes IS_IP_ACQUIRED(g_ulStatus)=1*/
    while(!IS_IP_ACQUIRED(g_ulStatus))
    {
      //looping till ip is acquired
    }

    unsigned char len = sizeof(SlNetCfgIpV4Args_t);
    SlNetCfgIpV4Args_t ipV4 = {0};

    // Get network configuration. Assigns static IP to itself = 192.168.1.1
    lRetVal = sl_NetCfgGet(SL_IPV4_AP_P2P_GO_GET_INFO,&ucDHCP,&len,
                            (unsigned char *)&ipV4);
    if (lRetVal < 0)
    {
        UART_PRINT("WARNING: Failed to get network configuration \n\r");
        sl_Stop(SL_STOP_TIMEOUT);
    }

    // Display CC3200 network status
    UART_PRINT("CC3200 Networks status:\n\r\
DHCP: ON  IP: %d.%d.%d.%d  MASK: %d.%d.%d.%d  GW %d.%d.%d.%d  DNS: %d.%d.%d.%d\n\n\r",
    SL_IPV4_BYTE(ipV4.ipV4,3),SL_IPV4_BYTE(ipV4.ipV4,2),SL_IPV4_BYTE(ipV4.ipV4,1),
	SL_IPV4_BYTE(ipV4.ipV4,0),SL_IPV4_BYTE(ipV4.ipV4Mask,3),SL_IPV4_BYTE(ipV4.ipV4Mask,2),
	SL_IPV4_BYTE(ipV4.ipV4Mask,1),SL_IPV4_BYTE(ipV4.ipV4Mask,0),
	SL_IPV4_BYTE(ipV4.ipV4Gateway,3),SL_IPV4_BYTE(ipV4.ipV4Gateway,2),
	SL_IPV4_BYTE(ipV4.ipV4Gateway,1),SL_IPV4_BYTE(ipV4.ipV4Gateway,0),
	SL_IPV4_BYTE(ipV4.ipV4DnsServer,3),SL_IPV4_BYTE(ipV4.ipV4DnsServer,2),
	SL_IPV4_BYTE(ipV4.ipV4DnsServer,1),SL_IPV4_BYTE(ipV4.ipV4DnsServer,0));

    UART_PRINT("Connect a client to Device\n\r");
    while(!IS_IP_LEASED(g_ulStatus)) /* g_ulStatus is 2 when arriving here */
    {
      //waiting for a client to connect
    }
    UART_PRINT("Client is connected to Device\n\r");

    iPingResult = PingTest(g_ulDestIp);
    if(iPingResult < 0)
    {
        UART_PRINT("WARNING: Ping to client failed\n\r");
    }

    // Create the Binary-Semaphore variable
    xSemaphoreResult = osi_SyncObjCreate( &g_sendSignal );
	if (xSemaphoreResult)
		UART_PRINT("WARNING: Semaphore creation error\n\r");


	// Open and configure the appropriate protocol sockets and then resume other tasks
    switch (g_bUdpOrTcp){
    case TRUE:
    	lRetVal = UdpCLientServerConfig(uiUdpPortNum);
        Task_setPri(UdpHandle, SEND_TASK_PRIORITY);
        Task_setPri(GreenLightHandle, GREEN_LIGHT_PRIORITY);
    	break;
    case FALSE:
    	lRetVal = TcpCLientServerConfig(uiTcpPortNum);
        Task_setPri(TcpHandle, SEND_TASK_PRIORITY);
        Task_setPri(GreenLightHandle, GREEN_LIGHT_PRIORITY);
    	break;
    }
    if (lRetVal < 0){
        UART_PRINT("WARNING: Could not set socket parameters\n\r");
        sl_Stop(SL_STOP_TIMEOUT);
	}


    // Configure the uDMA for communication with the external device
    lRetVal = Uart1ConfigDmaTransfer();
    if (lRetVal < 0){
        UART_PRINT("WARNING: Failed to configure uDMA controller parameters.\n\r");
        sl_Stop(SL_STOP_TIMEOUT);
	}
    else
        UART_PRINT("uDMA controller activated.\n\r");

    // When all Setup processes are done light the red led
	GPIO_IF_LedOn(MCU_RED_LED_GPIO);
}



/*****************************************************************************************
  *brief	  This function will be used in case no computer is available as a means for
  *			  the user to insert input to choose the desired protocol of communication.
  *			  If button SW3 is pressed UDP will be the selected protocol.
  *			  If button SW2 is pressed TCP will be the selected protocol.
  *			  User will have a timelimit specified by TimerA0 timeout to make his choise,
  *			  or else he'll have to restart the device.

  *param[in]  none
  *return     none
*****************************************************************************************/
#ifndef COMPUTER
void DecisionMaking(void)
{
    while ( !g_bButtonPress )
    {
        // Loop here until a decision has been made
    }

	if (g_bUdpOrTcp)
		UART_PRINT("UDP Selected.\n\r");
	else
		UART_PRINT("TCP Selected.\n\r");


	// Disable and unregister TimerA0 interrupt. It's not needed anymore
	MAP_TimerIntDisable(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);
	MAP_IntDisable(INT_TIMERA0A);
	MAP_IntUnregister(INT_TIMERA0A);
	//Clear, Disable and unregister the SW3 Button Interrupt. We no longer need it
	MAP_GPIOIntDisable(GPIOA1_BASE, GPIO_INT_PIN_5);
	MAP_GPIOIntClear(GPIOA1_BASE, GPIO_INT_PIN_5);
	MAP_IntDisable(INT_GPIOA1);
	MAP_IntUnregister(INT_GPIOA1);
	//Clear, Disable and unregister the Button SW2 Interrupt. Served it's purpose
	MAP_GPIOIntDisable(GPIOA2_BASE, GPIO_INT_PIN_6);
	MAP_GPIOIntClear(GPIOA2_BASE, GPIO_INT_PIN_6);
	MAP_IntDisable(INT_GPIOA2);
	MAP_IntUnregister(INT_GPIOA2);
}



/*****************************************************************************************
  *brief    Initialize and setup a Timer peripheral module. Setup Interrupt trigger source
  *			as timeout interrupt. Finally enable the specified timer/counter for operation.

  *param    ulTimerBase       	  = Base Address for the timer peripheral selected
  *param    ulTimerPeriph		  = Specified Timer Peripheral
  *param    ulTimerMode       	  = Mode that the timer will be utilized
  *param    ulPrescaleVal         = Sets the value for the prescaler, for 0 the prescaler
  *    								is not used
  *param    usIntPriority         = Priority of the timer interrupt
  *param    TimerInterruptHandler = Pointer to the Interrupt Handling function, which has
  *									no arguments and returns void/nothing
  *param    ulTimeoutVal 	  	  = Timeout value that upon met the interrupt will hit
  *param    ulTimer				  = Timer subperipheral module in use
  *return   none
*****************************************************************************************/

int TimerTimeoutConfig(unsigned long ulTimerBase, unsigned long ulTimerPeriph,
	unsigned long ulTimerMode, unsigned long ulPrescaleVal, unsigned short usIntPriority,
	void (*TimerInterruptHandler) (void), unsigned long ulTimeoutVal, unsigned long ulTimer )
{
	int lRetVal = -1;

	// Initializing Selected Timer
    MAP_PRCMPeripheralClkEnable(ulTimerPeriph, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(ulTimerPeriph);
    MAP_TimerConfigure(ulTimerBase,ulTimerMode);
    MAP_TimerPrescaleSet(ulTimerBase,ulTimer,ulPrescaleVal);

	// Setup the interrupts for the timer
#if defined(USE_TIRTOS) || defined(USE_FREERTOS) || defined(SL_PLATFORM_MULTI_THREADED)
    if(ulTimer == TIMER_BOTH){
    	lRetVal = osi_InterruptRegister(GetTimerInterruptNum(ulTimerBase, TIMER_A),
    			(P_OSI_INTR_ENTRY)TimerInterruptHandler, usIntPriority);
    	lRetVal = osi_InterruptRegister(GetTimerInterruptNum(ulTimerBase, TIMER_B),
    			(P_OSI_INTR_ENTRY)TimerInterruptHandler, usIntPriority);
    }
    else{
    	lRetVal = osi_InterruptRegister(GetTimerInterruptNum(ulTimerBase, ulTimer),
    			(P_OSI_INTR_ENTRY)TimerInterruptHandler, usIntPriority);
    }
#else
	  MAP_IntPrioritySet(GetTimerInterruptNum(ulTimerBase, ulTimer), usIntPriority);
      MAP_TimerIntRegister(ulTimerBase, ulTimer, TimerInterruptHandler);
#endif
      if ( lRetVal < 0 ){
      	UART_PRINT("CRITICAL ERROR: Timer interrupt register failure.\n\r");
      	ASSERT_ON_ERROR(INTERRUPT_ERROR);
      }

    // Enable the Timer Interrupt trigger source
    if(ulTimer == TIMER_BOTH)
     	MAP_TimerIntEnable(ulTimerBase, TIMER_TIMA_TIMEOUT|TIMER_TIMB_TIMEOUT);
    else
    	MAP_TimerIntEnable(ulTimerBase, ( (ulTimer == TIMER_A) ? TIMER_TIMA_TIMEOUT :
                                   TIMER_TIMB_TIMEOUT) );

	// Set the timer timeout value
	MAP_TimerLoadSet(ulTimerBase,ulTimer,MILLISECONDS_TO_TICKS(ulTimeoutVal));
	// Enable the GPTimer module with all parameters specified
	MAP_TimerEnable(ulTimerBase,ulTimer);

    return SUCCESS;
}



/*****************************************************************************************
  *brief    This function selects and returns the requested Timer Interrupt Number

  *param    ulTimerBase = Base Address of the Timer Peripheral selected
  *param    ulTimer		= The selected Timer module
  *return
*****************************************************************************************/

static unsigned char GetTimerInterruptNum(unsigned long ulTimerBase, unsigned long ulTimer)
{
	if (ulTimer == TIMER_A) {
		switch (ulTimerBase) {
		case TIMERA0_BASE:
			return INT_TIMERA0A;
		case TIMERA1_BASE:
			return INT_TIMERA1A;
		case TIMERA2_BASE:
			return INT_TIMERA2A;
		case TIMERA3_BASE:
			return INT_TIMERA3A;
		default:
			return INT_TIMERA0A;
		}
	} else if (ulTimer == TIMER_B) {
		switch (ulTimerBase) {
		case TIMERA0_BASE:
			return INT_TIMERA0B;
		case TIMERA1_BASE:
			return INT_TIMERA1B;
		case TIMERA2_BASE:
			return INT_TIMERA2B;
		case TIMERA3_BASE:
			return INT_TIMERA3B;
		default:
			return INT_TIMERA0B;
		}
	} else
		return INT_TIMERA0A;

}



/*****************************************************************************************
  *brief 	  Initializes Push Button SW3 and registers an interrupt handler for it

  *param[in]  Sw2ButtonHandler : Interrupt Handler function for SW2 LP button
  *param[in]  Sw3ButtonHandler : Interrupt Handler function for SW3 LP button
  *return 	  none
*****************************************************************************************/

int ButtonSetup ( void (*SW3ButtonInterruptHandler) (void),
		void (*SW2ButtonInterruptHandler) (void) )
{
	long lRetVal = -1;

    if(SW3ButtonInterruptHandler != NULL)
    {
        // Set Interrupt Type for GPIO
        MAP_GPIOIntTypeSet(GPIOA1_BASE,GPIO_PIN_5,GPIO_FALLING_EDGE);

        // Register Interrupt handler
#if defined(USE_TIRTOS) || defined(USE_FREERTOS) || defined(SL_PLATFORM_MULTI_THREADED)
        lRetVal = osi_InterruptRegister(INT_GPIOA1, (P_OSI_INTR_ENTRY)SW3ButtonInterruptHandler,
                                INT_PRIORITY_LVL_2);
        if ( lRetVal < 0 ){
        	UART_PRINT("CRITICAL ERROR: Button SW3 Interrupt register failed.\n\r");
        	ASSERT_ON_ERROR(INTERRUPT_ERROR);
        }
#else
		MAP_IntPrioritySet(INT_GPIOA1, INT_PRIORITY_LVL_2);
        MAP_GPIOIntRegister(GPIOA1_BASE, SW3ButtonInterruptHandler);
#endif
        // Enable Interrupt
        MAP_GPIOIntClear(GPIOA1_BASE,GPIO_INT_PIN_5);
        MAP_IntPendClear(INT_GPIOA1);
        MAP_IntEnable(INT_GPIOA1);
        MAP_GPIOIntEnable(GPIOA1_BASE,GPIO_INT_PIN_5);
    }

    if(SW2ButtonInterruptHandler != NULL)
    {
        // Set Interrupt Type
        MAP_GPIOIntTypeSet(GPIOA2_BASE,GPIO_PIN_6,GPIO_FALLING_EDGE);

        // Register Interrupt handler
#if defined(USE_TIRTOS) || defined(USE_FREERTOS) || defined(SL_PLATFORM_MULTI_THREADED)
        lRetVal = osi_InterruptRegister(INT_GPIOA2,(P_OSI_INTR_ENTRY)SW2ButtonInterruptHandler,
                                INT_PRIORITY_LVL_2);
        if ( lRetVal < 0 ){
        	UART_PRINT("CRITICAL ERROR: Button SW2 Interrupt register failed.\n\r");
        	ASSERT_ON_ERROR(INTERRUPT_ERROR);
        }
#else
		MAP_IntPrioritySet(INT_GPIOA2, INT_PRIORITY_LVL_2);
        MAP_GPIOIntRegister(GPIOA2_BASE, SW2ButtonInterruptHandler);
#endif
        // Enable Interrupt
        MAP_GPIOIntClear(GPIOA2_BASE,GPIO_INT_PIN_6);
        MAP_IntPendClear(INT_GPIOA2);
        MAP_IntEnable(INT_GPIOA2);
        MAP_GPIOIntEnable(GPIOA2_BASE,GPIO_INT_PIN_6);
    }

    return SUCCESS;
}
#endif


/*****************************************************************************************
  *brief 	  UDP socket parameters configuration. Opens a UDP client side
  *		      socket. Then binds the socket to local address to be used in
  *		      listening mode for clients to gain access.
  *		      Does not make transmissions here. The UDP server must be
  *		      already connected and listening to the device on UDP_PORT.

  *param[in]  port number on which the server will be listening on
  *return     0 on success, -1 on error
*****************************************************************************************/

int UdpCLientServerConfig(unsigned short usPort)
{
	int 		    iStatus;
	SlSockAddrIn_t  UdpLocalAddr; // Contains parameters for server-side socket services
#ifdef NON_BLOCKING_SOCKET
    SlSockNonblocking_t enableNBOption;
#endif

    // creating a (Blocking by default) UDP socket - an endpoint for communication.
    // Blocking socket means that the commands sl_Recv/sl_RecvFrom are blocked until the
    // (aka put the task to sleep, until the asynchronous event triggers) buffer
    // specified is completely loaded.
    g_iUdpSockHandle = sl_Socket(SL_AF_INET,SL_SOCK_DGRAM, IPPROTO_UDP);
    if( g_iUdpSockHandle < 0 )
    {
    	UART_PRINT("WARNING: UDP socket creation fault.\n\r");
    	sl_Close(g_iUdpSockHandle);
        ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
    }
    else
    	UART_PRINT("UDP Socket creation complete.\n\r");


	/* CC3200 as UDP CLIENT Setup */

    // filling the connected host's parameters address, port, protocol, name etc.
    g_slSocketParam.sin_family = SL_AF_INET;
    g_slSocketParam.sin_port = sl_Htons( (unsigned short)usPort );
    g_slSocketParam.sin_addr.s_addr = sl_Htonl( (unsigned int)g_ulDestIp );


#ifdef NON_BLOCKING_SOCKET   /* Force non-blocking socket implementation */
    enableNBOption.NonblockingEnabled = 1;

    iStatus = sl_SetSockOpt(g_iUdpSockHandle, SL_SOL_SOCKET, SL_SO_NONBLOCKING,
    		(_u8 *)&enableNBOption, sizeof(enableNBOption));
    if( iStatus < 0 ){
        UART_PRINT("WARNING: Error setting socket option.\n\r");
		sl_Close(g_iUdpSockHandle);
		ASSERT_ON_ERROR(SOCKET_OPT_ERROR);
	}
    else
    	UART_PRINT("UDP Socket set as Non-Blocking.\n\r");
#endif


	/* CC3200 as UDP SERVER Setup */

    // filling the UDP server socket address
    UdpLocalAddr.sin_family = SL_AF_INET;
    // reorder bytes from CPU order to Network order, i.e. in Big-Endian format
    UdpLocalAddr.sin_port = sl_Htons((unsigned short)usPort);
    UdpLocalAddr.sin_addr.s_addr = 0; // 0 indicates local host

	// binding local socket to a specific UDP port
    iStatus = sl_Bind(g_iUdpSockHandle, (SlSockAddr_t *)&UdpLocalAddr, g_iSockBlockSize);
    if( iStatus < 0 )
    {
    	UART_PRINT("WARNING: UDP socket binding error.\n\r");
    	sl_Close(g_iUdpSockHandle);
        ASSERT_ON_ERROR(BIND_ERROR);
    }
    else
    	UART_PRINT("UDP Socket binding complete.\n\r");

    return SUCCESS;
}



/*****************************************************************************************
  *brief 	  This function opens a TCP socket client and server side socket.
  *           It sets the parameters for the TCP sockets, makes the socket a non-blocking
  *           socket to optimize performance during communication.
  *           It tries to connect to a TCP client/server by putting the socket in
  *           listening mode and accepting a connection, before actually
  *           the sending/receiving of useful payload occurs.

  *param[in]  port number on which the server will be listening on
  *return     0 on success, -1 on error

  *note   	  1) This function will wait for an incoming connection until one is established
  *note   	  2) TCP Protocol must first handshake the two endpoints before transfering the
  *		   	  payload.
*****************************************************************************************/

int TcpCLientServerConfig(unsigned short usPort)
{
	int             	iStatus;
#ifdef TCP_CLIENT
	unsigned int 	    uiTimeout = 1000;
#else
	SlSockAddrIn_t 		TcpLocalAddress;
#endif
#ifdef NON_BLOCKING_SOCKET
    SlSockNonblocking_t enableNBOption;
#endif
	SlSockAddrIn_t 		TcpLocalAddress;

//	FD_ISSET(), FD_SET()
	UART_PRINT("TCP Configuration initiated. A TCP mate should be ready & stand by.\n\r");
#ifdef COMPUTER
	UART_PRINT("Press Enter when ready.\n\n\r");
    MAP_UARTCharGet(UARTA0_BASE); // Waits for a char before proceeding further
#else
	MAP_UtilsDelay(100000000);
#endif

    // creating a TCP socket
    g_iTcpSockHandle = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, IPPROTO_TCP);
    if( g_iTcpSockHandle < 0 )
    {
    	UART_PRINT("WARNING: TCP socket creation error.\n\r");
    	sl_Close(g_iTcpSockHandle);
        ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
    }
    else
    	UART_PRINT("TCP Socket creation complete.\n\r");

#ifdef NON_BLOCKING_SOCKET /* Setting socket as a Non-Blocking Socket
		May be useful especially for TCP connections		*/
	 enableNBOption.NonblockingEnabled = 1;
	 iStatus = sl_SetSockOpt( g_iTcpSockHandle, SL_SOL_SOCKET, SL_SO_NONBLOCKING,
	 	 	 	 (_u8 *)&enableNBOption, sizeof(enableNBOption) );
	 if( iStatus < 0 )
	 {
	 	 UART_PRINT("WARNING: Error in modifying TCP socket option.\n\r");
	 	 sl_Close(g_iTcpSockHandle);
	 	 ASSERT_ON_ERROR(SOCKET_OPT_ERROR);
	 }
	 else
	 	 UART_PRINT("Defining TCP Socket as Non-Blocking.\n\r");
#endif

	//filling the TCP server socket address
	TcpLocalAddress.sin_family = SL_AF_INET;
	TcpLocalAddress.sin_port = sl_Htons((unsigned short)usPort);
	TcpLocalAddress.sin_addr.s_addr = 0; // filling my local host IP

	// binding local socket to a specific TCP port
	iStatus = sl_Bind(g_iTcpSockHandle,(SlSockAddr_t *)&TcpLocalAddress, g_iSockBlockSize);
	if( iStatus < 0 ){
		UART_PRINT("WARNING: TCP socket binding error.\n\r");
		sl_Close(g_iTcpSockHandle);
	    ASSERT_ON_ERROR(BIND_ERROR);
	}
	else
		UART_PRINT("TCP Socket binding to local address complete.\n\r");


#ifdef TCP_CLIENT		/* CC3200 as TCP CLIENT Setup */
    // filling the TCP server socket address
    g_slSocketParam.sin_family = SL_AF_INET;
    g_slSocketParam.sin_port = sl_Htons((unsigned short)usPort); // reorder bytes from CPU order to Network byte order
    g_slSocketParam.sin_addr.s_addr = sl_Htonl((unsigned int)g_ulDestIp);

    // Connecting to a TCP server
    while (uiTimeout){
    	iStatus = sl_Connect(g_iTcpSockHandle,( SlSockAddr_t *)&g_slSocketParam,
    						g_iSockBlockSize);
    	if( iStatus < 0 ){
    		if ( iStatus == SL_EALREADY ){
    			MAP_UtilsDelay(10000);
        		--uiTimeout;
    		}else {
				UART_PRINT("WARNING: TCP connection error.\n\r");
				sl_Close(g_iTcpSockHandle);
				ASSERT_ON_ERROR(CONNECT_ERROR);
    		}
    	}else{
    		UART_PRINT("Succesfully connected to a TCP Server.\n\r");
    		break;
    	}
    }


#else					/* CC3200 as TCP SERVER Setup */
	// putting the socket in listening mode for the incoming TCP client connection
	iStatus = sl_Listen(g_iTcpSockHandle, 0);
	if( iStatus < 0 )
	{
    	UART_PRINT("WARNING: Setting socket in listening mode failed.\n\r");
		sl_Close(g_iTcpSockHandle);
		ASSERT_ON_ERROR(LISTEN_ERROR);
	}
    else
    	UART_PRINT("Socket is listening for incoming TCP connections.\n\r");

    // Below function accepts an incoming connection and creates a new separate file descriptor for it
    while( g_iNewTcpSockID < 0 )
    {
        // accepts a connection from a TCP client, if there is any, otherwise returns SL_EAGAIN
        g_iNewTcpSockID = sl_Accept(g_iTcpSockHandle, ( struct SlSockAddr_t *)&g_slSocketParam,
                                (SlSocklen_t*)&g_iSockBlockSize);
        if( g_iNewTcpSockID == SL_EAGAIN )
        {// try again, but wait a bit for the client
           MAP_UtilsDelay(10000);
        }
        else if( g_iNewTcpSockID < 0 )
        {// error
        	UART_PRINT("Unable to accept a connection from a TCP client.\n\r");
            sl_Close(g_iNewTcpSockID);
            sl_Close(g_iTcpSockHandle);
            ASSERT_ON_ERROR(ACCEPT_ERROR);
        }
        else{
        	UART_PRINT("Succesfully connected to a TCP client.\n\r");
        	UART_PRINT("Connection over TCP protocol Established Succesfully.\n\r");
        }
    }

#endif

	return SUCCESS;
}



/*****************************************************************************************
  *brief    Configures UART1-Rx to be used in conjunction with uDMA.
  *		    Sets up UARTA1-Rx uDMA channel. Also enables UART1.
  *		    The UART is configured so that uDMA-RX channel will receive any
  *		    incoming data into a pair of buffers in ping-pong mode.

  *param 	None
  *return   None

  *note		Whenever a DMA transfer is complete uDMA controller will cause an interrupt on
  *			the uART and Uart1IntHandler ISRwill be responsible for the continuation of
  *			the DMA transfers from then onwards.
*****************************************************************************************/

int Uart1ConfigDmaTransfer(void)
{
	long lRetVal 	   = -1;

	// Registers the interrupt handler for serving a UART1 interrupt
#if defined(SL_PLATFORM_MULTI_THREADED) || defined(TI_RTOS) || defined(FREE_RTOS)
	lRetVal = osi_InterruptRegister(INT_UARTA1,(P_OSI_INTR_ENTRY) Uart1IntHandler, \
			INT_PRIORITY_LVL_2);
    if ( lRetVal < 0 ){
    	UART_PRINT("CRITICAL ERROR: UART1 Interrupt register failed.\n\r");
    	ASSERT_ON_ERROR(INTERRUPT_ERROR);
    }
#else
    MAP_IntPrioritySet(INT_UARTA1,INT_PRIORITY_LVL_2);
    MAP_UARTIntRegister(UARTA1_BASE,Uart1IntHandler);
#endif

	// UART1 Init
	MAP_UARTConfigSetExpClk(UARTA1_BASE, MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
	                  UARTA1_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
	                    UART_CONFIG_PAR_NONE));
	MAP_UARTFlowControlSet(UARTA1_BASE, UART_FLOWCONTROL_NONE);

	// Assigns a uDMA-controller peripheral mapping for the selected DMA channel
    MAP_uDMAChannelAssign(UDMA_CH10_UARTA1_RX);
    MAP_uDMAChannelAssign(UDMA_CH11_UARTA1_TX);
	// Selects the uDMA channel for UARTA1-Rx transfers
	UDMAChannelSelect(UDMA_CH10_UARTA1_RX, NULL);
	UDMAChannelSelect(UDMA_CH11_UARTA1_TX, NULL);

    // Set RX trigger threshold to 4. This will be used by the uDMA controller to
    // signal when more data should be transferred.
    // The uDMA RX channel will trigger the Tx and Rx UART1 interrupt when the FIFO
    // is half full, ie it holds 4 out of 8 bytes (best all round method).
    // Each time 4 bytes will be transferred in a burst.
    MAP_UARTFIFOLevelSet(UARTA1_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

    // Read the interrupt status of the UART and clear any pending interrupts
    // This might be useful after warm resets
    lRetVal = MAP_UARTIntStatus(UARTA1_BASE, 1);
    MAP_UARTIntClear(UARTA1_BASE,lRetVal);

    // Enable the UART peripheral interrupts. uDMA controller will cause an
    // interrupt on the UART interrupt signal when a uDMA transfer is complete.
    MAP_UARTIntEnable(UARTA1_BASE,UART_INT_DMARX);

    // Setup the transfers
    UDMASetupTransfer(UDMA_CH10_UARTA1_RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG,
              sizeof(g_cUart1RxBufA), UDMA_SIZE_8, UDMA_ARB_2,
              (void *)(UARTA1_BASE + UART_O_DR), UDMA_SRC_INC_NONE,
			  g_cUart1RxBufA, UDMA_DST_INC_8);

    UDMASetupTransfer(UDMA_CH10_UARTA1_RX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,
              sizeof(g_cUart1RxBufB), UDMA_SIZE_8, UDMA_ARB_2,
              (void *)(UARTA1_BASE + UART_O_DR), UDMA_SRC_INC_NONE,
			  g_cUart1RxBufB, UDMA_DST_INC_8);

    UDMASetupTransfer(UDMA_CH11_UARTA1_TX | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
              sizeof(g_cUart1TxBuf), UDMA_SIZE_8, UDMA_ARB_2,
			  g_cUart1TxBuf, UDMA_SRC_INC_8, (void *)(UARTA1_BASE + UART_O_DR),
              UDMA_DST_INC_NONE);

    // Enable the UART for operation, and enable the uDMA Tx and Rx path channels
    MAP_UARTEnable(UARTA1_BASE);
    MAP_UARTDMAEnable(UARTA1_BASE, UART_DMA_RX | UART_DMA_TX);

    return SUCCESS;
}



/*****************************************************************************************
  *brief   Application startup display on UART

  *param   none
  *return  none
*****************************************************************************************/

static void DisplayBanner(char * AppName)
{
    Report("\n\n\n\r");
    Report("\t\t ***************************************************\n\r");
    Report("\t\t       CC3200 Application       \n\r");
    Report("\t\t %s \n\r", APP_NAME);
    Report("\t\t ***************************************************\n\r");
    Report("\n\n\n\r");
}



/*****************************************************************************************
  *brief   Pin assignment and configuration, setting the clock for peripherals used.

  *param   None
  *return  None
*****************************************************************************************/

void PinMuxConfig(void)
{
    // Enable Peripheral Clocks
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK);

	// Reset peripheral modules to default state first
	MAP_PRCMPeripheralReset(PRCM_UARTA0);
	MAP_PRCMPeripheralReset(PRCM_UARTA1);
	MAP_PRCMPeripheralReset(PRCM_GPIOA1);
	MAP_PRCMPeripheralReset(PRCM_GPIOA2);

    // Configure PIN_55 for UART0_TX
    MAP_PinTypeUART(PIN_55, PIN_MODE_3);

    // Configure PIN_57 for UART0_RX
    MAP_PinTypeUART(PIN_57, PIN_MODE_3);

    // Configure PIN_7 for UART1_TX
    MAP_PinTypeUART(PIN_07, PIN_MODE_5);

    // Configure PIN_8 for UART1_RX
    MAP_PinTypeUART(PIN_08, PIN_MODE_5);

    // Configure PIN_64/Red Led for GPIO Output
    MAP_PinTypeGPIO(PIN_64, PIN_MODE_0, FALSE);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x2, GPIO_DIR_MODE_OUT);

    // Configure PIN_02/Green Led for GPIO Output
    MAP_PinTypeGPIO(PIN_02, PIN_MODE_0, FALSE);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x8, GPIO_DIR_MODE_OUT);

    // Configure PIN_01/Orange Led for GPIO Output
    MAP_PinTypeGPIO(PIN_01, PIN_MODE_0, FALSE);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x4, GPIO_DIR_MODE_OUT);

    // Configure PIN_04 Button - SW3 - for GPIO Input
    MAP_PinTypeGPIO(PIN_04, PIN_MODE_0, FALSE);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x20, GPIO_DIR_MODE_IN);

    // Configure PIN_15 Button - SW2 - for GPIO Input
    MAP_PinTypeGPIO(PIN_15, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA2_BASE, 0x40, GPIO_DIR_MODE_IN);
}



/*****************************************************************************************
  *brief   Board Initialization & Configuration

  *param   None
  *return  None
*****************************************************************************************/

static void BoardInit(void)
{
// In case of TI-RTOS vector table is initialize by OS itself
#ifndef USE_TIRTOS
    // Set vector table base
#if defined(ccs) || defined(gcc)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif

    // Enables Processor and the master interrupt knob
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    // This should be the first call in main() for proper initialization of the device
    PRCMCC3200MCUInit();
}



/*****************************************************************************************
*                   INITIALIZATION/SETUP FUNCTIONS - End
*****************************************************************************************/





/*****************************************************************************************
*                             RTOS TASKS - Start
*****************************************************************************************/



/*****************************************************************************************
  *brief   This thread transmits the buffers with the data it received from
  *		   the external device via the UART1 interface to the remote host. It also
  *		   receives the UDP packets from the connected station and flushes them to the
  *		   external device via the uDMA Tx Channel.
  *		   The Task runs after every UART1-ISR completion. They are synchronized using a
  *		   semaphore variable.

  *param   None
  *return  None

  *note    For UDP protocol initial handshaking is not required (connection-less socket)
*****************************************************************************************/

void TransmitReceiveUdpPackets(void * pvParameters)
{
	long lRetVal = -1;

	while(1){
    	if ( !osi_SyncObjWait( &g_sendSignal , OSI_WAIT_FOREVER) ){	// Waiting on the semaphore
    		/* Transmitting */
    		if ( g_bBufTurn ){// The Rx A buffer of the primary control structure
				              // will send data
				lRetVal = sl_SendTo(g_iUdpSockHandle, g_cUart1RxBufA, UDMA_RXCH_BUF_SIZE, 0,
							  (SlSockAddr_t *)&g_slSocketParam, g_iSockBlockSize);
			}else if ( !g_bBufTurn ){// The Rx B buffer of the alternate control
	              	  	  	  	  	 // structure will send data
				lRetVal = sl_SendTo(g_iUdpSockHandle, g_cUart1RxBufB, UDMA_RXCH_BUF_SIZE, 0,
							  (SlSockAddr_t *)&g_slSocketParam, g_iSockBlockSize);
			}
			if (lRetVal > 0)
				++g_ulTxCount; // used to count the No of transmissions

			/* Receiving */
			lRetVal = sl_RecvFrom(g_iUdpSockHandle, g_cUart1TxBuf, UDMA_TXCH_BUF_SIZE, 0,
					  ( SlSockAddr_t *)&g_slSocketParam, (SlSocklen_t*)&g_iSockBlockSize );
			if (lRetVal > 0){
				++g_ulRxCount; // Counts the No of receptions

				// Leads the received data to the external device.
				UDMASetupTransfer(UDMA_CH11_UARTA1_TX| UDMA_PRI_SELECT, UDMA_MODE_BASIC,
	                  sizeof(g_cUart1TxBuf), UDMA_SIZE_8, UDMA_ARB_2,
	    			  g_cUart1TxBuf, UDMA_SRC_INC_8, (void *)(UARTA1_BASE + UART_O_DR),
	                  UDMA_DST_INC_NONE);
				++g_ulTxBufFillCount;
			}
    	}// end semaphore case
		osi_Sleep(1);
	}
}



/*****************************************************************************************
  *brief   Task is responsible for the actual transfers between the two network hosts,
  *		   operating in TCP/IP protocol. Function sends the TCP stream and also receives
  *		   from the connected host. The received data will also be sent to the device
  *		   via the DMA UART1-Tx channel.

  *param   None
  *return  None
*****************************************************************************************/

void SendRecvTcpStream(void * pvParameters)
{
	int iRetVal;

	while(1){
    	if ( !osi_SyncObjWait( &g_sendSignal , OSI_WAIT_FOREVER) ){	// Waiting on the semaphore
    		/* Sending */
			iRetVal = sl_Connect(g_iTcpSockHandle,( SlSockAddr_t *)&g_slSocketParam,
								g_iSockBlockSize);
			if ( iRetVal == SL_EALREADY )
				osi_Sleep(1); // If TCP endpoint isn't ready wait for a while

			if ( g_bBufTurn )// The Rx A buffer will send data
				iRetVal = sl_Send(g_iTcpSockHandle, g_cUart1RxBufA, UDMA_RXCH_BUF_SIZE, 0);
			else if ( !g_bBufTurn )// The Rx B buffer will send data
				iRetVal = sl_Send(g_iTcpSockHandle, g_cUart1RxBufB, UDMA_RXCH_BUF_SIZE, 0);

			if ( iRetVal > 0 )
				++g_ulTxCount; // Counts the No of succesful transmissions

			/* Receiving */
#ifdef TCP_CLIENT
			iRetVal = sl_Recv(g_iTcpSockHandle, g_cUart1TxBuf, UDMA_TXCH_BUF_SIZE, 0);
#else
			iRetVal = sl_Recv(g_iNewTcpSockID, g_cUart1TxBuf, UDMA_TXCH_BUF_SIZE, 0);
#endif
			if ( iRetVal > 0 ){
				++g_ulRxCount; //increment a counter to signal that data has been received

				// Leads the received data to the external device.
				UDMASetupTransfer(UDMA_CH11_UARTA1_TX| UDMA_PRI_SELECT, UDMA_MODE_BASIC,
					  sizeof(g_cUart1TxBuf), UDMA_SIZE_8, UDMA_ARB_2,
					  g_cUart1TxBuf, UDMA_SRC_INC_8, (void *)(UARTA1_BASE + UART_O_DR),
					  UDMA_DST_INC_NONE);
				++g_ulTxBufFillCount;
			}
    	}//end semaphore case
		osi_Sleep(1);
	}
}



/*****************************************************************************************
  *brief    Lights the green led to indicate that the device is working properly.
  * 		It's transmitting and receiving data as planned.

  *param    None
  *return   None
*****************************************************************************************/

void GreenLight(void * pvParameters)
{
	for(;;) {
		if ( (g_ulTxCount > GOOD_COUNT) && (g_ulRxCount > GOOD_COUNT) && (g_ulRxABufFillCount > GOOD_COUNT)
				&& (g_ulRxBBufFillCount > GOOD_COUNT) && (g_ulTxBufFillCount > GOOD_COUNT) ) {
			GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
		    Task_setPri(GreenLightHandle, INACTIVE_STATE_PRIORITY); // we are done with this task
		}
		else  // we don't want this task to cause overhead so we check it every 10 secs
			osi_Sleep(10000);
	}
}



/*****************************************************************************************
*                                RTOS TASKS - End
*****************************************************************************************/





/*****************************************************************************************
*                                Interrupts - Start
*****************************************************************************************/



/*****************************************************************************************
  *brief     The interrupt handler for UART1 (ISR/HWI).  This interrupt will occur
  *		     when a DMA transfer is complete using the UART1 uDMA channel.
  *          This interrupt handler will switch between Rx ping-pong buffers A and B
  *          and the Tx "basic-mode-DMA" buffer.
  * 		 It will also restart a uDMA transfer if the prior one is complete.
  *  	     This will keep the UART running continuously sending data back and forth.

  *param 	 None
  *return 	 None

  *Note      We should clear the individual interrupt bit-flags early to keep
  *          the interrupt handler from executing immediately after exiting.

  *Warning   Generally the Interrupt Handler Routine should complete as fast as
  *	         possible (no delays in it etc.), or else unexpected events may occur. For
  *	         this purpose it should implement only non-blocking calls
*****************************************************************************************/

void Uart1IntHandler(void)
{
    long lMode   = -1;
    long lStatus = -1;

    // Read the interrupt status. The int source flag is returned.
    lStatus = MAP_UARTIntStatus(UARTA1_BASE, 1);
    // Clear the interrupt assertion flag. Ready for the next interrupt after exiting this one
    MAP_UARTIntClear(UARTA1_BASE, lStatus);

    // Check the DMA control table to see if the ping-pong "A" transfer is complete.
    lMode = MAP_uDMAChannelModeGet(UDMA_CH10_UARTA1_RX | UDMA_PRI_SELECT);

    // If the primary DMA structure indicates stop, then the "A" Rx buf-transfer is done
    if(lMode == UDMA_MODE_STOP)
    {
        // Set up the next transfer for the "A" buffer
        UDMASetupTransfer(UDMA_CH10_UARTA1_RX | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG,
                          sizeof(g_cUart1RxBufA),UDMA_SIZE_8, UDMA_ARB_2,
                          (void *)(UARTA1_BASE + UART_O_DR), UDMA_SRC_INC_NONE,
						  g_cUart1RxBufA, UDMA_DST_INC_8);
        ++g_ulRxABufFillCount;
        g_bBufTurn = TRUE;
    }

    // Check the DMA control table to see if the ping-pong "B" transfer is complete.
    lMode = MAP_uDMAChannelModeGet(UDMA_CH10_UARTA1_RX | UDMA_ALT_SELECT);

    // If the alternate DMA structure indicates stop, then the "B" Rx buf-transfer is done
    if(lMode == UDMA_MODE_STOP)
    {
        // Set up the next transfer for the "B" buffer
        UDMASetupTransfer(UDMA_CH10_UARTA1_RX | UDMA_ALT_SELECT,
                          UDMA_MODE_PINGPONG, sizeof(g_cUart1RxBufB),UDMA_SIZE_8,
                          UDMA_ARB_2,(void *)(UARTA1_BASE + UART_O_DR),
						  UDMA_SRC_INC_NONE, g_cUart1RxBufB, UDMA_DST_INC_8);
        ++g_ulRxBBufFillCount;
		g_bBufTurn = FALSE;
    }

    osi_SyncObjSignalFromISR( &g_sendSignal );
}



/*****************************************************************************************
  *brief	GPIO Interrupt Handler for SW3 button press
  *			If user presses this button within the timelimit of Timer's A0 timeout value
  *			UDP protocol will be selected

  *param	None
  *return	None
*****************************************************************************************/
#ifndef COMPUTER
void Sw3InterruptHandler(void)
{
    unsigned long ulIntStatus;

    // Clear SW3 Interrupt flag
    ulIntStatus = MAP_GPIOIntStatus(GPIOA1_BASE, 1);
	MAP_GPIOIntClear(GPIOA1_BASE, ulIntStatus);

	g_bUdpOrTcp = TRUE; //UDP Select
	g_bButtonPress = TRUE;
}



/*****************************************************************************************
  *brief	GPIO Interrupt Handler for SW2 button press
  *			If user presses this button within the timelimit of Timer's A0 timeout value
  *			TCP protocol will be selected

  *param	None
  *return	None
*****************************************************************************************/

void Sw2InterruptHandler(void)
{
    unsigned long ulIntStatus;

    // Clear SW2 Interrupt flag
    ulIntStatus = MAP_GPIOIntStatus(GPIOA2_BASE, 1);
	MAP_GPIOIntClear(GPIOA2_BASE, ulIntStatus);

	g_bUdpOrTcp = FALSE; //TCP Select
	g_bButtonPress = TRUE;
}



/*****************************************************************************************
  *brief	The interrupt handler for the timer A0. Will be triggered 30 seconds after
  *			startup to quit program execution if user hasn't pressed a button.

  *param  None
  *return none
*****************************************************************************************/

void TimerA0InterruptHandler(void)
{
    // Light Orange LED as a sign that device is inoperable. Has to be restarted to be used
    GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
    // Quit Application
    BIOS_exit(100);
}
#endif


/*****************************************************************************************
*                                Interrupts - End
*****************************************************************************************/





/*****************************************************************************************
*                              MAIN FUNCTION - Start
*****************************************************************************************/

int main()
{
	long  lRetVal 			  = -1;
#ifdef COMPUTER
	short sInvalidInputTries  = 5;
	_Bool bInputCheck 		  = FALSE;
	char  cCmdBuf[10];
	short sInput;
#endif


	// Board Initialization
	BoardInit();

	// Configure the pinmux settings for the peripherals exercised
	PinMuxConfig();

	// Initializing Console UART0
	InitTerm();
	ClearTerm();

	// Displays Application Banner
	DisplayBanner(APP_NAME);

	// uDMA controller Initialization
	UDMAInit();

	// Setup the Leds
	GPIO_IF_LedConfigure(LED1|LED2|LED3);
    GPIO_IF_LedOff(MCU_ALL_LED_IND);


    /*
     * All tasks and interrupts must be registered and enabled prior to starting RTOS.
     * Also, no interrupt or task can be executed prior to starting the RTOS.
     */
#ifndef COMPUTER
	// Setup and Register Timer A0 Interrupt
	lRetVal = TimerTimeoutConfig(TIMERA0_BASE,PRCM_TIMERA0,TIMER_CFG_PERIODIC,0, \
			INT_PRIORITY_LVL_1, TimerA0InterruptHandler, 30000, TIMER_A);
	if (lRetVal < 0){
		UART_PRINT("CRITICAL ERROR: Failed to Initialize Timer-A0.\n\r");
		exit(EXIT_FAILURE);
	}

	// Setup buttons for the user to choose UDP (SW3), or TCP(SW2)
	lRetVal = ButtonSetup (Sw3InterruptHandler, Sw2InterruptHandler);
	if (lRetVal < 0){
		UART_PRINT("WARNING: Failed to configure the Buttons.\n\r");
		exit(EXIT_FAILURE);
	}


	//Enable the SimpleLink Host driver to enable tasks and put them into a queue
	lRetVal = VStartSimpleLinkSpawnTask(SL_SPAWN_TASK_PRIORITY);
	if (lRetVal < 0) {
		ERR_PRINT(lRetVal);
		UART_PRINT("CRITICAL ERROR: Failed to create SimpleLink Task.\n\r");
		exit(EXIT_FAILURE);
	}

	// Create the InitialSetup task
	lRetVal = osi_TaskCreate(InitialSetup,(const signed char*) "Configur-at-ing",
				OSI_TASK_STACK_SIZE, NULL, 7, NULL);
	if (lRetVal < 0) {
		ERR_PRINT(lRetVal);
		UART_PRINT("WARNING: Failed to create task for WLan connection.\n\r");
		exit(EXIT_FAILURE);
	}

	// Creates task for transmitting  UDP packets to the remote host.
	lRetVal = osi_TaskCreate(TransmitReceiveUdpPackets,(const signed char*) "UDP Packet Transmission",
				OSI_TASK_STACK_SIZE, NULL, SEND_TASK_PRIORITY,&UdpHandle);
	if (lRetVal < 0) {
		ERR_PRINT(lRetVal);
		UART_PRINT("WARNING: Failed to create task for UDP communication.\n\r");
		exit(EXIT_FAILURE);
	}


	// Creates task for transmitting  TCP packets to the connected TCP server
	lRetVal = osi_TaskCreate(SendRecvTcpStream,(const signed char*) "TCP Stream Export",
				OSI_TASK_STACK_SIZE, NULL, SEND_TASK_PRIORITY,&TcpHandle);
	if (lRetVal < 0) {
		ERR_PRINT(lRetVal);
		UART_PRINT("WARNING: Failed to create task for TCP communication.\n\r");
		exit(EXIT_FAILURE);
	}

	// Task to light the green led as a sign that the device is working as expected
	lRetVal = osi_TaskCreate(GreenLight,(const signed char*) "GREEN LIGHT",
				OSI_TASK_STACK_SIZE, NULL, GREEN_LIGHT_PRIORITY,&GreenLightHandle);
	if (lRetVal < 0) {
		ERR_PRINT(lRetVal);
		UART_PRINT("WARNING: Failed to create task for Green-LED.\n\r");
		exit(EXIT_FAILURE);
	}

	// Once the task scheduler is called control doesn't return in the function that
	// called it ever again. Only tasks and ints will be executed from now on and all
	// those tasks and interrupts must have already been registered and enabled
	osi_start();


#else
	do{
		--sInvalidInputTries;

		// Prompts the User to select either TCP or UDP communication
		UART_PRINT("Options:\n\rFor UDP press 1\r\nFor TCP press 0\r\nTo exit press 8\r\n");
		GetCmd(cCmdBuf, sizeof(cCmdBuf));
		sInput  = (short)strtoul(cCmdBuf,0,10); 		// Convert to int No

		if ( (sInput == 1) || (sInput == 0) ){
			// Starts the SimpleLink Host to create tasks and add them to a queue
			lRetVal = VStartSimpleLinkSpawnTask(SL_SPAWN_TASK_PRIORITY);
			if(lRetVal < 0)	{
				ERR_PRINT(lRetVal);
				UART_PRINT("CRITICAL ERROR: Failed to create SimpleLink Task.\n\r");
				exit(EXIT_FAILURE);
			}

			// Create the InitialSetup task
			lRetVal = osi_TaskCreate( InitialSetup, (const signed char*)"Configur-at-ing",
					OSI_TASK_STACK_SIZE, NULL, 7, NULL );
			if(lRetVal < 0) {
				ERR_PRINT(lRetVal);
				UART_PRINT("WARNING: Failed to create task for WLan connection.\n\r");
				exit(EXIT_FAILURE);
			}
			bInputCheck = FALSE;
		}

		switch ( sInput ){
		case 1:                /***  UDP-Enable  ***/
			UART_PRINT("UDP Selected.\n\r");
		    bInputCheck = FALSE;
		    // Creates task for transmitting  UDP packets to the connected station
		    lRetVal = osi_TaskCreate( TransmitReceiveUdpPackets, (const signed char*) "UDP Packet Transmission",
					OSI_TASK_STACK_SIZE, NULL, SEND_TASK_PRIORITY, &UdpHandle);
		    if (lRetVal < 0) {
		    	ERR_PRINT(lRetVal);
		    	UART_PRINT("WARNING: Failed to create task for UDP communication.\n\r");
		    	exit(EXIT_FAILURE);
		    }
		    break;

		case 0:                 /***  TCP-Enable  ***/
			UART_PRINT("TCP Selected.\n\r");
			g_bUdpOrTcp = FALSE;
		    bInputCheck = FALSE;
		    // Creates task for transmitting  TCP packets to the remote host.
		    lRetVal = osi_TaskCreate( SendRecvTcpStream, (const signed char*) "TCP Stream Export",
					OSI_TASK_STACK_SIZE, NULL, SEND_TASK_PRIORITY, &TcpHandle);
		    if (lRetVal < 0) {
		    	ERR_PRINT(lRetVal);
		    	UART_PRINT("WARNING: Failed to create task TCP communication.\n\r");
		    	exit(EXIT_FAILURE);
		    }
		    break;

		case 8:
			UART_PRINT("Exiting Application...\n\r");
			bInputCheck = FALSE;
			break;

		default:
			if ( !sInvalidInputTries ){
				UART_PRINT("That's just wrong.\n\r");
				bInputCheck = FALSE;
			}
			else{
				UART_PRINT("Wrong Input. Try Again.\n\r" \
"You have %i tries remaining.\n\r", sInvalidInputTries);
				bInputCheck = TRUE;
			}
			break;
		}

	}while( bInputCheck );


	if ( (sInput == 0) || (sInput == 1) )
		// Task to light the green led as a sign that the device is working as expected
		lRetVal = osi_TaskCreate( GreenLight, (const signed char*) "GREEN LIGHT",
				OSI_TASK_STACK_SIZE, NULL, GREEN_LIGHT_PRIORITY, &GreenLightHandle);
		if (lRetVal < 0) {
			ERR_PRINT(lRetVal);
			UART_PRINT("WARNING: Failed to create task for Green-LED.\n\r");
			exit(EXIT_FAILURE);
		}


	// Invokes the task scheduler
	osi_start();
#endif

	return EXIT_SUCCESS;
}

/*****************************************************************************************
//                           MAIN FUNCTION - End
*****************************************************************************************/


