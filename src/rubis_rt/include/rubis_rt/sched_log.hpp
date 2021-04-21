#ifndef RUBIS_OMP_LOG_HPP_
#define RUBIS_OMP_LOG_HPP_

#include "spdlog/spdlog.h"
namespace rubis {

struct sched_data {
  int task_id;
  int iter;
  double runtime;
  double period;
  double deadline;
} SchedData;

// class SchedLog {
// public:
//     sched_data data;
//     std::shared_ptr<spdlog::logger> async_logger;
//     SchedLog();
//     SchedLog(std::string _logger_name, std::string _logger_out_path);
//     bool log_to_file(sched_data _data);
//     bool write_header();
//     bool compare(const sched_data_thread &a, const sched_data_thread &b);
//     bool file_exist(std::string fileName);
// };

} // namespace rubis
#endif