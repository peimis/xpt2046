#include <stdio.h>
#include <time.h>

#include "mgos.h"
#include "mgos_config.h"
#include "mgos_spi.h"

#include "xpt2046.h"

/***************************************************************************************
***************************************************************************************/
#define USE_GLOBAL_SPI 1

static struct mgos_spi *xpt2046_spi;
static struct mgos_spi_txn xpt2046_txn;
#if defined(USE_GLOBAL_SPI) && (USE_GLOBAL_SPI == 0)
static struct mgos_config_spi xp2046_bus_cfg;
#endif

uint32_t tp_calx = TP_CALX_XPT2046;
uint32_t tp_caly = TP_CALY_XPT2046;

struct mgos_xpt2046_event_data xpt_last_touch;

static enum mgos_xpt2046_rotation_t _lcd_orientation = XPT2046_PORTRAIT;
static mgos_xpt2046_event_t xpt_event_handler = NULL;
static bool is_touching = false;
static uint16_t s_max_x = 240;
static uint16_t s_max_y = 320;

// ============= Touch panel functions =========================================



// get 16-bit data from touch controller for specified type
// ** Touch device must already be selected **
//----------------------------------------
static int xpt2046_read_data(uint8_t type)
{
	int res;
	uint8_t rxbuf[3];

	xpt2046_txn.fd.len = 3;
	xpt2046_txn.fd.tx_data = &type;
	xpt2046_txn.fd.rx_data = rxbuf;

	if (!mgos_spi_run_txn(xpt2046_spi, true /* full_duplex */, &xpt2046_txn)) {
		LOG(LL_ERROR, ("SPI transaction failed"));
		return -1;
	}

	res = (rxbuf[1] << 8) | rxbuf[2];

/*
    spi_lobo_device_select(xpt0246_spi, 0);

    xpt0246_spi->host->hw->data_buf[0] = type;
    _xspi_transfer_start(xpt0246_spi, 24, 24);
    uint16_t res = (uint16_t)(xpt0246_spi->host->hw->data_buf[0] >> 8);
    res = ((res & 0xFF) << 8) | ((res & 0xFF00) >> 8);

    spi_lobo_device_deselect(xpt0246_spi);
*/
    return res;
}


//-------------------------------------------------------
static int xpt2046_get_touch_data(uint8_t type, int samples)
{
	if (xpt2046_spi == (void *)NULL) return 0;

	int n, result, val = 0;
	int avg = 0;
	uint32_t i = 0;
	uint32_t minval, maxval, dif=0;

    if (samples < 3) samples = 1;
    if (samples > 18) samples = 18;

    // one dummy read
    result = xpt2046_read_data(type);
    avg = result >> 3;

    // read data
	while (i < 10) {
    	minval = 5000;
    	maxval = 0;
		// get values
		for (n=0;n<samples;n++) {
		    result = xpt2046_read_data(type) >> 3;
		    avg = ((avg * 3) + result) / 4;

			if (result < 0) break;

//			vbuf[n] = result;
			if (result < minval) minval = result;
			if (result > maxval) maxval = result;
		}
		if (result < 0) break;
		dif = maxval - minval;
		if (dif < 40) break;
		i++;
    }

	if (result < 0) return -1;
	val = avg;

    return val;
}


//=============================================
//
int xpt2046_read_touch(int *x, int* y, int* z)
{
	int value = -1, res=0;
    int X=0, Y=0, Z=0;

	if (xpt2046_spi == (void *)NULL) return 0;

//	if (spi_lobo_device_select(xpt0246_spi, 0) != ESP_OK) return 0;

    value = xpt2046_get_touch_data(0xB0, 3);  // Z; pressure; touch detect
	Z = value;
	if (value <= 50)  goto exit;

	// touch panel pressed
	value = xpt2046_get_touch_data(0xD0, 10);
	if (value < 0)  goto exit;

	X = value;

	value = xpt2046_get_touch_data(0x90, 10);
	if (value < 0)  goto exit;

	Y = value;
	res = 1;

exit:
//	spi_lobo_device_deselect(xpt0246_spi);

   	if (value == 0) return 0;

	*x = X;
	*y = Y;
	*z = Z;

	return res;
}



static long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	if (x<in_min) x=in_min;
	if (x>in_max) x=in_max;
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


static void xpt2046_map_rotation(uint16_t x, uint16_t y, uint16_t *x_out, uint16_t *y_out)
{
	const int xmax = (tp_calx >> 16) & 0x3FFF;
	const int xmin = tp_calx & 0x3FFF;
	const int ymax = (tp_caly >> 16) & 0x3FFF;
	const int ymin = tp_caly & 0x3FFF;

	switch(_lcd_orientation)
	{
	case XPT2046_LANDSCAPE:
	    *x_out = map(y, ymin, xmax, 0, s_max_x);
	    *y_out = map(x, xmin, ymax, 0, s_max_y);
	    break;
	case XPT2046_PORTRAIT_FLIP:
	    *x_out = map(x, xmin, xmax, 0, s_max_x);
	    *y_out = 4095-map(y, ymin, ymax, 0, s_max_y);
	    break;
	case XPT2046_LANDSCAPE_FLIP:
	    *x_out = 4095-map(y, ymin, xmax, 0, s_max_x);
	    *y_out = 4095-map(x, xmin, ymax, 0, s_max_y);
	    break;
	default: // XPT2046_PORTRAIT
	    *x_out = 4095-map(x, xmin, xmax, 0, s_max_x);
	    *y_out = map(y, ymin, ymax, 0, s_max_y);
	}
}


//
//
void xpt2046_read_timer_cb(void *arg)
{
	int tx, ty, tz;
	int pin = (int)arg;

	bool touch_state = xpt2046_read_touch(&tx, &ty, &tz);

	if (touch_state) {
		xpt2046_map_rotation(tx, ty, &xpt_last_touch.x, &xpt_last_touch.y);
		printf("XPT touch %d %d -> %d %d\n", tx, ty, xpt_last_touch.x, xpt_last_touch.y);

//		mgos_ili9341_drawCircle(tx, ty, 3, ILI9341_GREEN);
		xpt_last_touch.length++;
		xpt_last_touch.z = tz;

		mgos_set_timer(100, 0, xpt2046_read_timer_cb, (void *)pin);
	} else {
		xpt_last_touch.direction = TOUCH_UP;
		mgos_gpio_enable_int(pin);
	}

	if (xpt_event_handler) xpt_event_handler(&xpt_last_touch);
}



//
//
void xpt2046_intr_handler(const int pin, void *arg)
{
 	struct mgos_xpt2046_event_data ed;
	const bool pin_state = mgos_gpio_read(pin);
	int touch_state = 0;
	int tx=0, ty=0, tz;

	if (!pin_state) {
		mgos_gpio_disable_int(pin);

		mgos_set_timer(100, 0, xpt2046_read_timer_cb, (void *)pin);

		if ((touch_state = xpt2046_read_touch(&tx, &ty, &tz))) {


			xpt2046_map_rotation(tx, ty, &ed.x, &ed.y);
//			mgos_ili9341_drawCircle(tx, ty, 5, ILI9341_MAGENTA);
			printf("%d %d -> %d %d\n", tx, ty, ed.x, ed.y);

	        ed.length=1;
	        ed.direction = TOUCH_DOWN;
	        ed.z = tz;

	        // To avoid DOWN events without an UP event, set a timer (see stmpe610_down_cb for details)
	        memcpy((void *)&xpt_last_touch, (void *)&ed, sizeof(xpt_last_touch));
		}
		else
		{
	        ed.length=1;
	        ed.direction = TOUCH_UP;
	        ed.x = 0;
	        ed.y = 0;
	        ed.z = 0;
		}
		if (xpt_event_handler) xpt_event_handler(&ed);
	}
}


// Mongoose-OS init
//
bool mgos_xpt2046_init(void)
{
	esp_err_t ret;

	_lcd_orientation = mgos_sys_config_get_tft_orientation();

	LOG(LL_INFO, ("XPT2046 irq pin '%d' cs index '%d'", mgos_sys_config_get_xpt2046_irq_pin(), mgos_sys_config_get_xpt2046_cs_index()));

	// ====  CONFIGURE SPI DEVICES(s)  ====================================================================================

#if defined(USE_GLOBAL_SPI) && (USE_GLOBAL_SPI == 0)
	struct mgos_config_spi ep_bus_cfg = {
	  .unit_no = 2,
	  .miso_gpio = mgos_sys_config_get_spi_miso_gpio(),
	  .mosi_gpio = mgos_sys_config_get_spi_mosi_gpio(),
	  .sclk_gpio = mgos_sys_config_get_spi_sclk_gpio(),
	  .cs0_gpio = mgos_sys_config_get_xpt2046_cs_pin(),
	  .debug = true,
	};
#endif

#if defined(USE_GLOBAL_SPI) && (USE_GLOBAL_SPI == 0)
	xpt2046_spi = mgos_spi_create(&xpt2046_bus_cfg);
	xpt2046_bus_cfg = xpt2046_bus_cfg;
#else
	xpt2046_spi = mgos_spi_get_global();
#endif

	if (xpt2046_spi == NULL) {
		LOG(LL_ERROR, ("SPI is not configured, make sure spi.enable is true"));
		return 0;
	}

	struct mgos_spi_txn txn = {
	  .cs = mgos_sys_config_get_xpt2046_cs_index(),		/* Use CS0 line as configured by cs0_gpio */
	  .mode = 0,
	  .freq = 500000,
	};
	xpt2046_txn = txn;

	gpio_pad_select_gpio(mgos_sys_config_get_xpt2046_irq_pin());
	mgos_gpio_set_mode(mgos_sys_config_get_xpt2046_irq_pin(), MGOS_GPIO_MODE_INPUT);
	mgos_gpio_set_pull(mgos_sys_config_get_xpt2046_irq_pin(), MGOS_GPIO_PULL_NONE);
	mgos_gpio_set_int_handler(mgos_sys_config_get_xpt2046_irq_pin(), MGOS_GPIO_INT_EDGE_NEG, xpt2046_intr_handler, NULL);
	mgos_gpio_enable_int(mgos_sys_config_get_xpt2046_irq_pin());

	LOG(LL_INFO, ("CS%d IRQ=%d", mgos_sys_config_get_xpt2046_cs_index(), mgos_sys_config_get_xpt2046_irq_pin() ));

	return true;
}

bool mgos_xpt2046_is_touching(void)
{
	return (xpt_last_touch.direction == TOUCH_DOWN);
}

void mgos_xpt2046_set_dimensions(const uint16_t x, const uint16_t y)
{
	s_max_x = x;
	s_max_y = y;
}

void mgos_xpt2046_set_handler(mgos_xpt2046_event_t handler) {
  xpt_event_handler = handler;
}


void mgos_xpt2046_set_rotation(enum mgos_xpt2046_rotation_t rotation) {
  _lcd_orientation = rotation;
}
