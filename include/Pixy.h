#ifndef PIXY_H_
#define PIXY_H_

#include "API.h"

#include <stdbool.h>
#include <stdlib.h>

#define kPixyBlockArraySize 30

// Communication/miscellaneous parameters.
extern const unsigned int kPixyBaudDefault;
extern const unsigned short kPixyMaxSignature;

// Pixy x-y position values.
extern const unsigned short kPixyMinX;
extern const unsigned short kPixyMaxX;
extern const unsigned short kPixyMinY;
extern const unsigned short kPixyMaxY;

// RC-servo values.
extern const unsigned short kPixyRcsMinPos;
extern const unsigned short kPixyRcsMaxPos;
extern const unsigned short kPixyRcsMidPos;

typedef enum PixyBlockType {
	PixyBlockNormal,
	PixyBlockCode
} PixyBlockType;

typedef struct {
	unsigned short signature;
	unsigned short x;
	unsigned short y;
	unsigned short width;
	unsigned short height;
	unsigned short angle;  // Angle is only available for color coded blocks.
} PixyBlock;

typedef struct {
	PROS_FILE *port;
	PixyBlock blocks[kPixyBlockArraySize];
	unsigned short blockCount;
	bool skipStart;
	PixyBlockType blockType;
} Pixy;

/**
 * Create new Pixy object.
 *
 * @param  port UART port the Pixy is plugged into (<code>uart1</code> or
 *              	<code>uart2</code>).
 * @param  baud Baud rate the Pixy is set to use.
 *
 * @return Pixy object.
 */
Pixy pixyCreate(PROS_FILE *port);

/**
 * Update block data for Pixy.
 *
 * @param  pixy Pointer to Pixy struct.
 *
 * @return Number of blocks found.
 */
unsigned short pixyCaptureBlocks(Pixy *pixy);

/**
 * Get number of blocks last recorded by Pixy via pixyCaptureBlocks.
 *
 * @param  pixy Pointer to Pixy struct.
 *
 * @return Number of blocks.
 */
unsigned short pixyBlockCount(Pixy *pixy);

/**
 * Set Pixy camera brightness.
 *
 * @param  pixy       Pointer to Pixy struct.
 * @param  beightness Brightness value.
 *
 * @return Number of bytes sent via fwrite.
 */
size_t pixySetBrightness(Pixy *pixy, unsigned char brightness);

/**
 * Set Pixy LED color.
 *
 * @param  pixy  Pointer to Pixy struct.
 * @param  red   Red intensity.
 * @param  green Green intensity.
 * @param  blue  Blue intensity.
 *
 * @return Number of characters sent via sendChars().
 */
size_t pixySetLed(Pixy *pixy, unsigned char red, unsigned char green, unsigned char blue);

/**
 * Set pan/tilt servo positions.
 *
 * @param  pixy   Pointer to Pixy struct.
 * @param  servo0 Pan position.
 * @param  servo1 Tilt position.
 *
 * @return Number of characters sent via fwrite.
 */
size_t pixySetServos(Pixy *pixy, unsigned short servo0, unsigned short servo1);

/**
 * Print single Pixy block data to debug stream.
 *
 * @param pixy Pointer to PixyBlock struct.
 */
void pixyBlockPrint(PixyBlock *pixy);

/**
 * Print all Pixy block data to debug stream.
 *
 * @param pixy Pointer to Pixy struct.
 */
void pixyPrint(Pixy *pixy);

#endif  // PIXY_H_
