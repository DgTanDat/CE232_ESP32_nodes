#ifndef EPOCH_TIME_H
#define EPOCH_TIME_H

#include "time.h"

/**
 * @brief Lấy thời gian epoch hiện tại
 *
 * @return Thời gian epoch tính bằng giây
 */
time_t get_epoch_time();
void format_time(time_t epoch_time, char *buffer, size_t buffer_size);

#endif // EPOCH_TIME_H
