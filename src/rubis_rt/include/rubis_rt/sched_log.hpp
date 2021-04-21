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
  double resp_time;
  double period;
  double deadline;
};

class SchedLog {
public:
  SchedLog();
  SchedLog(std::string _name, std::string _file);
  void add_entry(sched_data _sd);

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
  h_str += "start_t\t";
  h_str += "end_t\t";
  h_str += "resp_t\t";
  h_str += "slack\t";
  return h_str;
}

void SchedLog::add_entry(sched_data _sd) {
  std::string e_str = "";
  e_str += std::to_string(_sd.task_id) + "\t";
  e_str += std::to_string(_sd.iter) + "\t";
  e_str += std::to_string(0.0) + "\t";  // exec_time
  e_str += std::to_string(_sd.period) + "\t";
  e_str += std::to_string(_sd.deadline) + "\t";
  e_str += std::to_string(0.0) + "\t";  // start_time
  e_str += std::to_string(0.0) + "\t";  // end_time
  e_str += std::to_string(_sd.resp_time) + "\t";
  e_str += std::to_string(0.0) + "\t";  // slack
  _log->info(e_str);
  return;
}

} // namespace sched_log
} // namespace rubis
#endif