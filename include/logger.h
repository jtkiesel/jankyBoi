// logger.h
//
// Author: Justin Marple, Team BNS
// Contact: justinjmarple@gmail.com
// Date: 2/4/2018
//
// The liblogger library for PROS contains a simple way to turn on and off
// logging streams at varying levels.  It is especially useful for developing
// code where file and line descriptions are printed out along with the message.
// Being able to turn on and off debug messages at runtime is also important
// when developing libraries for PROS.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef __LOGGER_H__
#define __LOGGER_H__

#include "API.h"

#define LOGLEVEL_NONE     99
#define LOGLEVEL_CRITICAL 50
#define LOGLEVEL_ERROR    40
#define LOGLEVEL_WARNING  30
#define LOGLEVEL_INFO     20
#define LOGLEVEL_DEBUG    10
#define LOGLEVEL_NOTSET    0

// vfprintf is implemented in src/printf.c in the pros source.
// Copied from comm.h
int vfprintf(PROS_FILE *stream, const char *formatString, va_list arguments);
int fprintf(PROS_FILE *stream, const char *formatString, ...);

struct Logger
{
  // Level for which logs to print out to the console
  // CRITICAL: 50
  // ERROR   : 40
  // WARNING : 30
  // INFO    : 20
  // DEBUG   : 10
  // NOTSET  :  0
  int level;

  // File handler for where to print data
  FILE* output;
};

// Initializes the logger structure.  By default, the stream is set to stdout
void logger_init(struct Logger* log, int level);

// Sets the debug level.  It's recommended to use the LOGLEVEL_##### statements.
// For instance, if LOGLEVEL_WARNING is used, all messages with a log level of
// warning or higher would be printed. Specifically log_warning, log_error
// and log_critical would be printed.  Others would be ignored.
void logger_set_level(struct Logger* log, int level);

// Sets which stream to send log data to.  Default is stdout.
void logger_set_stream(struct Logger* log, FILE* stream);

// Returns a global logger shared by every source file.
struct Logger* logger_get_global_log();

// Macros are used for the logging calls so that __FILE__ and __LINE__ will be
// from where the logger is called, instead of where the logger function exists
#define logger_debug(l, s, ...) \
  _logger_generic(l, LOGLEVEL_DEBUG, "DEBUG:", __FILE__, __LINE__, s, ##__VA_ARGS__)

#define logger_info(l, s, ...) \
  _logger_generic(l, LOGLEVEL_INFO, "INFO:", __FILE__, __LINE__, s, ##__VA_ARGS__)

#define logger_warning(l, s, ...) \
  _logger_generic(l, LOGLEVEL_WARNING, "WARNING:", __FILE__, __LINE__, s, ##__VA_ARGS__)

#define logger_error(l, s, ...) \
  _logger_generic(l, LOGLEVEL_ERROR, "ERROR:", __FILE__, __LINE__, s, ##__VA_ARGS__)

#define logger_critical(l, s, ...) \
  _logger_generic(l, LOGLEVEL_CRITICAL, "CRITICAL:", __FILE__, __LINE__, s, ##__VA_ARGS__)

// A generic logging printf.
//  - Prefix is a string that is put at the beginning of each log
//  - filename and line_num should be written as __FILE__ and __LINE__
//  - fmt and ... are your normal printf string and variables
// It is recommended to use the above macros instead.
int _logger_generic(struct Logger* log, int level,
  const char* prefix,
  const char* file_name, int line_num,
  const char* fmt, ...);

#endif
