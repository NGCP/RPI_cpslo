#include <stdio.h>
#include <ctime>
#include <cstdio>
#include <mavlink_control.h>

class Writer {
   private:
      time_t cur_time_stamp;
      FILE *data_log;
      fpos_t local_pos_cursor;
      fpos_t global_pos_cursor;
      fpos_t attitude_cursor;
      fpos_t imu_cursor;

   public:
      void createFile();
      void createHeaders();
      void write_initial_position(Autopilot_Interface &api);
      void write_data(struct api_data cur_struct);
      void write_set_point_reached();
      void closeFile();
}
