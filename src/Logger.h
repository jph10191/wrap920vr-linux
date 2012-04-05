#ifndef LOGGER_H
#define LOGGER_H
#define LOG(format, args...) printf("%d:", __LINE__); printf(format, args)

#endif
