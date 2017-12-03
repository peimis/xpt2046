/*
 * High level TFT functions
 * Author:  peimis 04/2017, https://github/loboris
 * 
 */

#ifndef _MGOS_LIB_XPT2046_TOUCH_H_
#define _MGOS_LIB_XPT2046_TOUCH_H_


//==========================================================================================
// ==== Global variables ===================================================================
//==========================================================================================

#define TP_CALX_XPT2046		((3700UL << 16) | (450))
#define TP_CALY_XPT2046		((3800UL << 16) | (310))


// === Screen orientation constants ===
enum mgos_xpt2046_rotation_t {
  XPT2046_PORTRAIT       = 0,
  XPT2046_LANDSCAPE      = 1,
  XPT2046_PORTRAIT_FLIP  = 2,
  XPT2046_LANDSCAPE_FLIP = 3,
};

enum mgos_xpt2046_touch_t {
  TOUCH_DOWN = 0,
  TOUCH_UP = 1,
};

struct mgos_xpt2046_event_data {
  enum mgos_xpt2046_touch_t direction;
  uint16_t x;
  uint16_t y;
  uint8_t z;
  uint8_t length; // Amount of time the TOUCH_DOWN event lasted (always set to 1 for TOUCH_UP)
};


// ===== PUBLIC FUNCTIONS =========================================================================


/*
 * Get the touch panel coordinates.
 * The coordinates are adjusted to screen orientation if raw=0
 *
 * Params:
 * 		x: pointer to X coordinate
 * 		y: pointer to Y coordinate
 * 	  raw: if 0 returns calibrated screen coordinates; if 1 returns raw touch controller coordinates
 *
 * Returns:
 * 		0	if touch panel is not touched; x=y=0
 * 		>0	if touch panel is touched; x&y are the valid coordinates
 */
//----------------------------------------------
int TFT_read_touch(int *x, int* y, uint8_t raw);


bool TFT_Touch_intr_init(void);

bool mgos_xpt2046_is_touching(void);
void mgos_xpt2046_set_dimensions(const uint16_t x, const uint16_t y);

/**
 * @brief MGOS lib init
 */

typedef void (*mgos_xpt2046_event_t)(struct mgos_xpt2046_event_data *);
void mgos_xpt2046_set_handler(mgos_xpt2046_event_t handler);
void mgos_xpt2046_set_rotation(enum mgos_xpt2046_rotation_t rotation);

#endif  // _MGOS_LIB_XPT2046_TOUCH_H_
