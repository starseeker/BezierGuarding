#ifndef TIME_H
#define TIME_H

#include <mach/mach_time.h>
#include <string>

using std::string;

extern uint64_t t_start;
extern uint64_t t_end;
extern std::string original_file_name;

double subtractTimes(uint64_t endTime, uint64_t startTime);

#endif // TIME_H
