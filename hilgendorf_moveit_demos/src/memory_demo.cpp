#include <iostream>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <unistd.h>

/**
 * \brief process_mem_usage(double &, double &) - takes two doubles by reference,
 *        attempts to read the system-dependent data for a process' virtual memory
 *        size and resident set size, and
 * \return the results in MB. On failure, returns 0.0, 0.0
 */
void process_mem_usage(double& vm_usage, double& resident_set)
{
  using std::ios_base;
  using std::ifstream;

  vm_usage     = 0.0;
  resident_set = 0.0;

  // 'file' stat seems to give the most reliable results
  //
  std::ifstream stat_stream("/proc/self/stat",ios_base::in);

  // dummy vars for leading entries in stat that we don't care about
  //
  std::string pid, comm, state, ppid, pgrp, session, tty_nr;
  std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  std::string utime, stime, cutime, cstime, priority, nice;
  std::string O, itrealvalue, starttime;

  // the two fields we want
  //
  unsigned long vsize;
  long rss;

  stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
              >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
              >> utime >> stime >> cutime >> cstime >> priority >> nice
              >> O >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest

  stat_stream.close();

  long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
  vm_usage     = vsize / 1024.0;
  resident_set = rss * page_size_kb;

  // Convert to MB
  vm_usage     = vm_usage / 1024.0;
  resident_set = resident_set / 1024.0;
}

class StateType
{
public:
  StateType()
  {
  }
  double *values;
};

int main(int argc, char** argv)
{
  double vm, rss;

  double array_size_b = 100000*sizeof(double); // bytes
  double state_size_b = sizeof(StateType); // bytes

  std::cout << "Size of state: " << state_size_b << " bytes" << std::endl;
  std::cout << "Size of 100000 doubles: " << array_size_b << " bytes" << std::endl;

  // Get computer's base measure of memory usage
  process_mem_usage(vm, rss);

  // On first loop, add base memory
  double my_count_mb = vm;

  std::vector<StateType*> states;

  // Load memory
  while (vm < 1000)
  {
    states.push_back(new StateType());
    states.back()->values = new double[100000];

    // My count
    my_count_mb += (array_size_b + state_size_b) / 1048576.0; // convert byte to megabyte

    // Get computer's measure of memory usage
    process_mem_usage(vm, rss);

    // Output
    //std::cout << "VM: " << vm << " MB    |    my_count: " << my_count_mb << " MB" << std::endl;
  }

  // Unload array
  /*
  for (std::size_t i = 0; i < states.size(); ++i)
  {
    // Unload memory
    delete[] states[i]->values;

    // My count
    my_count_mb -= (array_size_b) / 1048576.0; // convert byte to megabyte

    // Get computer's measure of memory usage
    process_mem_usage(vm, rss);

    // Output
    std::cout << "VM: " << vm << " MB    |    my_count: " << my_count_mb << " MB" << std::endl;
  }
  */

  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  //usleep(500000);

  // Unload state
  for (std::size_t i = 0; i < states.size(); ++i)
  {
    // Unload memory
    delete[] states[i]->values;
    delete states[i];

    // My count
    my_count_mb -= (state_size_b + array_size_b) / 1048576.0; // convert byte to megabyte

    // Get computer's measure of memory usage
    process_mem_usage(vm, rss);

    // Output
    std::cout << "VM: " << vm << " MB    |    my_count: " << my_count_mb << " MB" << std::endl;
  }



  return 0;
}
