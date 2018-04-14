#ifndef LOG_H_
#define LOG_H_

void logError(const char* functionName, const char* message);

void logWarning(const char* functionName, const char* message);

void logDebug(const char* functionName, const char* message);

void logInfo(const char* functionName, const char* message);

#endif  // LOG_H_
