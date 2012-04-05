#ifndef LOGGER_H
#define LOGGER_H
#define LOG(string, args...) \
fprintf (string"\n", ##args)
#endif
