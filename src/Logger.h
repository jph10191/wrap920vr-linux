#ifndef LOGGER_H
#define LOGGER_H
#define LOG(string, args...) \
printf (string"\n", ##args)
#endif
