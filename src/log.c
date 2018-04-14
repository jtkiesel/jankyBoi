#include "log.h"

#include "API.h"

static void logPrint(const char* level, const char* functionName, const char* message) {
	printf("%s - %s: %s\n", level, functionName, message);
}

void logError(const char* functionName, const char* message) {
	logPrint("ERROR", functionName, message);
}

void logWarning(const char* functionName, const char* message) {
	logPrint("WARNING", functionName, message);
}

void logDebug(const char* functionName, const char* message) {
	logPrint("DEBUG", functionName, message);
}

void logInfo(const char* functionName, const char* message) {
	logPrint("INFO", functionName, message);
}
