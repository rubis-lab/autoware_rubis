#ifndef RUBIS_OMP_LOG_HPP_
#define RUBIS_OMP_LOG_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/basic_file_sink.h"

namespace rubis {
namespace sched_log {

struct sched_data {
  int task_id;
  int iter;
  double runtime;
  double period;
  double deadline;
};

class SchedLog {
public:
  sched_data sdat;
  SchedLog();
  SchedLog(std::string _name, std::string _file);

private:
  std::string generate_header();
  std::shared_ptr<spdlog::logger> _log;
};

SchedLog::SchedLog() {
  return;
}

SchedLog::SchedLog(std::string _name, std::string _file) {  
  std::ifstream _fs(_file);
  _log = spdlog::basic_logger_mt<spdlog::async_factory>(_name, _file);
  if(_fs) { // file exists
    std::cout << "Appending log to: " << _file << std::endl;
  } else {
    std::cout << "Log file created in: " << _file << std::endl;
    _log->info(generate_header());
  }
  _fs.close();
  return;
}

std::string SchedLog::generate_header() {
  std::string h_str;
  h_str += "tid\t";
  h_str += "iter\t";
  h_str += "exec_t\t";
  h_str += "period\t";
  h_str += "dead\t";
  h_str += "sta_t\t";
  h_str += "end_t\t";
  h_str += "resp_t\t";
  h_str += "slack\n";
  return h_str;
}

} // namespace sched_log
} // namespace rubis
#endif