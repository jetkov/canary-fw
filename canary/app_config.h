#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define NRF_LOG_BACKEND_UART_ENABLED    0
#define NRF_LOG_BACKEND_RTT_ENABLED     1

// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 
#define NRF_LOG_DEFAULT_LEVEL           4

#define NRF_SDH_BLE_GAP_EVENT_LENGTH    500
#define NRF_SDH_BLE_VS_UUID_COUNT       16

#define NRFX_PPI_ENABLED 1
#define NRFX_RTC_ENABLED 1
#define NRFX_RTC0_ENABLED 1
#define NRFX_TIMER_ENABLED 1
#define NRFX_TIMER1_ENABLED 1
#define TIMER_ENABLED 1
#define TIMER1_ENABLED 1

#define NRF_LIBUARTE_ASYNC_WITH_APP_TIMER 1
#define NRF_QUEUE_ENABLED 1

// <h> nrf_libuarte_drv - libUARTE library

//==========================================================
// <q> NRF_LIBUARTE_DRV_HWFC_ENABLED  - Enable HWFC support in the driver
 

#ifndef NRF_LIBUARTE_DRV_HWFC_ENABLED
#define NRF_LIBUARTE_DRV_HWFC_ENABLED 0
#endif

// <q> NRF_LIBUARTE_DRV_UARTE0  - UARTE0 instance
 

#ifndef NRF_LIBUARTE_DRV_UARTE0
#define NRF_LIBUARTE_DRV_UARTE0 1
#endif

// <q> NRF_LIBUARTE_DRV_UARTE1  - UARTE1 instance
 

#ifndef NRF_LIBUARTE_DRV_UARTE1
#define NRF_LIBUARTE_DRV_UARTE1 0
#endif

#endif

#define NRFX_PPI_ENABLED 1
#define PPI_ENABLED 1

#define NRF_LIBUARTE_CONFIG_LOG_ENABLED 1