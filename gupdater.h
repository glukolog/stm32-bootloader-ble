#include <stdint.h>

//#define FLASH_VERIFY

/* Start and end addresses of the user application. */
#define FLASH_APP_START_ADDRESS ((uint32_t)0x08002000u)
#define FLASH_APP_END_ADDRESS   ((uint32_t)FLASH_BANK1_END-0x10u) /**< Leave a little extra space at the end. */

#define UART_TIMEOUTX ((uint16_t)2000u)
#define RX_PKT_SZ 	20
#define MAX_RETRY	10

#define CH_WAIT		'G'
#define CH_RETRY	'T'
#define CH_BADCRC	'X'
#define	CH_ERR		'R'
#define	CH_ERRF		'F'
#define	CH_OK		'K'
#define	CH_END		'E'

typedef void (*fnc_ptr)(void);

uint8_t updater_receiver(void);
void flash_jump_to_app(void);
