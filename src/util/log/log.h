#ifndef __LOG_H__
#define __LOG_H__

#define LOG_LEVEL_DEBUG     2
#define LOG_LEVEL_ERROR     1
#define LOG_LEVEL_NONE      0

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#define LOG_D(msg)   /*debug.print("[Debug] : ");*/ Serial.print(msg)
#else
#define LOG_D(msg)
#endif

#if LOG_LEVEL >= LOG_LEVEL_ERROR
#define LOG_E(msg)   /*debug.print("[Error] : ");*/ Serial.print(msg)
#else
#define LOG_E(msg)
#endif

#define LOG_LEVEL_VERBOSE




#endif // __LOG_H__