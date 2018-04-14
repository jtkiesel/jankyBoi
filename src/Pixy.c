#include "Pixy.h"

#include "API.h"
#include "log.h"
#include "util.h"

#include <stdbool.h>
#include <stdlib.h>

const unsigned int kPixyBaudDefault = 19200;
const unsigned short kPixyMaxSignature = 7;
static const unsigned short kPixyStartNormal = 0xaa55;
static const unsigned short kPixyStartCode = 0xaa56;
static const unsigned short kPixyStartX = 0x55aa;

const unsigned short kPixyMinX = 0;
const unsigned short kPixyMaxX = 319;
const unsigned short kPixyMinY = 0;
const unsigned short kPixyMaxY = 199;

const unsigned short kPixyRcsMinPos = 0;
const unsigned short kPixyRcsMaxPos = 1000;
const unsigned short kPixyRcsMidPos = 500;

/**
 * Get start word from Pixy.
 *
 * @param  pixy Pointer to Pixy struct.
 *
 * @return <code>true</code> if start code found, <code>false</code> otherwise.
 */
static bool pixyGetStart(Pixy *pixy) {
	if (!pixy) {
		logError("pixyGetStart", "pixy NULL");
		return false;
	}
	int lastW = 0xffff;  // Some inconsequential initial value.
	int w;

	while (true) {
		w = fgetw(pixy->port);
		if (w == 0 && lastW == 0) {
			return false;  // No start code.
		} else if (w == kPixyStartNormal && lastW == kPixyStartNormal) {
			pixy->blockType = PixyBlockNormal;
			return true;  // Code found!
		} else if (w == kPixyStartCode && lastW == kPixyStartCode) {
			pixy->blockType = PixyBlockCode;  // Found color code block.
			return true;
		} else if (w == kPixyStartX) {  // This is important, we might be juxtaposed.
			fgetc(pixy->port);  // We're out of sync (backwards)!
		}
		lastW = w;
	}
}

Pixy pixyCreate(PROS_FILE *port) {
	if (!port) {
		logError("pixyCreate", "port NULL");
		return (Pixy) {};
	}
	return (Pixy) {.port = port, .blockCount = 0, .skipStart = false,
			.blockType = PixyBlockNormal};
}

unsigned short pixyCaptureBlocks(Pixy *pixy) {
	if (!pixy) {
		logError("pixyCaptureBlocks", "pixy NULL");
		return 0;
	}
	unsigned short blockCount = 0;
	int w, checksum, sum;
	PixyBlock *block;

	if (!pixy->skipStart) {
		if (!pixyGetStart(pixy)) {
			pixy->blockCount = 0;
			return 0;
		}
	} else {
		pixy->skipStart = false;
	}
	while (blockCount < kPixyBlockArraySize) {
		checksum = fgetw(pixy->port);
		if (checksum == kPixyStartNormal) {
			// We've reached the beginning of the next frame.
			pixy->skipStart = true;
			pixy->blockType = PixyBlockNormal;
			pixy->blockCount = blockCount;
			break;
		} else if (checksum == kPixyStartCode) {
			pixy->skipStart = true;
			pixy->blockType = PixyBlockCode;
			pixy->blockCount = blockCount;
			break;
		} else if (checksum == 0) {
			pixy->blockCount = blockCount;
			break;
		}
		block = &pixy->blocks[blockCount];

		w = fgetw(pixy->port);
		block->signature = (unsigned short) w;
		sum = w;

		w = fgetw(pixy->port);
		block->x = (unsigned short) w;
		sum += w;

		w = fgetw(pixy->port);
		block->y = (unsigned short) w;
		sum += w;

		w = fgetw(pixy->port);
		block->width = (unsigned short) w;
		sum += w;

		w = fgetw(pixy->port);
		block->height = (unsigned short) w;
		sum += w;

		// No angle for regular block.
		w = (pixy->blockType == PixyBlockNormal) ? 0 : fgetw(pixy->port);
		block->angle = (unsigned short) w;
		sum += w;

		// Check checksum.
		if (checksum == sum) {
			blockCount++;
		} else {
			logError("pixyCaptureBlocks", "checksum error!");
		}
		w = fgetw(pixy->port);
		if (w == kPixyStartNormal) {
			pixy->blockType = PixyBlockNormal;
		} else if (w == kPixyStartCode) {
			pixy->blockType = PixyBlockCode;
		} else {
			pixy->blockCount = blockCount;
			break;
		}
	}
	return blockCount;
}

unsigned short pixyBlockCount(Pixy *pixy) {
	if (!pixy) {
		logError("pixyBlockCount", "pixy NULL");
		return 0;
	}
	return pixy->blockCount;
}

size_t pixySetBrightness(Pixy *pixy, unsigned char brightness) {
	if (!pixy) {
		logError("pixySetBrightness", "pixy NULL");
		return 0;
	}
	unsigned char outBuf[] = {0x00, 0xfe, brightness};
	size_t size = sizeof(outBuf[0]);
	size_t count = size / sizeof(outBuf);

	return fwrite(outBuf, count, size, pixy->port);
}

size_t pixySetLed(Pixy *pixy, unsigned char r, unsigned char g, unsigned char b) {
	if (!pixy) {
		logError("pixySetLed", "pixy NULL");
		return 0;
	}
	unsigned char outBuf[] = {0x00, 0xfd, r, g, b};
	size_t size = sizeof(outBuf[0]);
	size_t count = size / sizeof(outBuf);

	return fwrite(outBuf, count, size, pixy->port);
}

size_t pixySetServos(Pixy *pixy, unsigned short servo0, unsigned short servo1) {
	if (!pixy) {
		logError("pixySetServos", "pixy NULL");
		return 0;
	}
	unsigned char servo0a = (unsigned char) (servo0 & 0xff);
	unsigned char servo0b = (unsigned char) (servo0 >> 8);
	unsigned char servo1a = (unsigned char) (servo1 & 0xff);
	unsigned char servo1b = (unsigned char) (servo1 >> 8);
	unsigned char outBuf[] = {0x00, 0xff, servo0a, servo0b, servo1a, servo1b};
	size_t size = sizeof(outBuf[0]);
	size_t count = size / sizeof(outBuf);

	return fwrite(outBuf, count, size, pixy->port);
}

void pixyBlockPrint(PixyBlock *pixy) {
	if (!pixy) {
		logError("pixyBlockPrint", "pixy NULL");
		return;
	}
	if (pixy->signature > kPixyMaxSignature) {  // Color code!
		printf("signature: %o x: %d y: %d width: %d height: %d angle: %d\n",
				pixy->signature, pixy->x, pixy->y, pixy->width, pixy->height,
				pixy->angle);
	} else {  // Regular block. Note, angle is always 0, so no need to print.
		printf("signature: %d x: %d y: %d width: %d height: %d\n",
				pixy->signature, pixy->x, pixy->y, pixy->width, pixy->height);
	}
}

void pixyPrint(Pixy *pixy) {
	if (!pixy) {
		logError("pixyPrint", "pixy NULL");
		return;
	}
	printf("Detected %d:\n", pixy->blockCount);

	for (unsigned short i = 0; i < pixy->blockCount; i++) {
		printf("\tBlock %d: ", i);
		pixyBlockPrint(&pixy->blocks[i]);
	}
}
