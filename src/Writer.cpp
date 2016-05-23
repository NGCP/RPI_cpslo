// ------------------------------------------------------------------------------
// Writer Class definition
// ------------------------------------------------------------------------------

class Writer {
   private:
      time_t cur_time_stamp;
      FILE *data_log;
      fpos_t local_pos_cursor;
      fpos_t global_pos_cursor;
      fpos_t attitude_cursor;
      fpos_t imu_cursor;

   public:
      void createFile() {
         time(&cur_time_stamp);
         data_log = fopen(ctime(&cur_time_stamp), "w");

         if (data_log == NULL)
            perror ("Error opening file");
      }

      void createHeaders() {
         int num_blank_lines = 100;
         int cur;

         fprintf(data_log, "Timestamp [usec], X Position, Y Position, Z Position, "
                 "X Target, Y Target, Z Target\n");
         fprintf(data_log, "LocalPos = [...\n");
         fgetpos(data_log, &local_pos_cursor);

         for (cur = 0; cur < num_blank_lines; cur++)
            fprintf(data_log, "\n");

         fprintf(data_log, "Timestamp [usec], "
                 "Latitude [deg * 1e7], Longitude [deg * 1e7], Altitude [m?], "
                 "Target Lat [deg * 1e7], Target Long [deg * 1e7], Target Alt [m?]\n");
         fprintf(data_log, "GlobalPos = [...\n");
         fgetpos(data_log, &global_pos_cursor);

         for (cur = 0; cur < num_blank_lines; cur++)
            fprintf(data_log, "\n");

         fprintf(data_log, "Timestamp [usec], phi (roll) [rad], theta (pitch) [rad], psi "
                 "(yaw) [rad], p (roll rate) [rad/s], q (pitch rate) [rad/s], r (yaw rate) [rad/s]\n");
         fprintf(data_log, "Attitude = [...\n");
         fgetpos(data_log, &attitude_cursor);

         for (cur = 0; cur < num_blank_lines; cur++)
            fprintf(data_log, "\n");

         fprintf(data_log, "Timestamp [usec], xacc [m/s2], yacc [m/s2], zacc [m/s2], "
                 "xgyro [rad/s], ygyro [rad/s], zgyro [rad/s], "
                 "xmag [Gauss], ymag [Gauss], zmag [Gauss]\n");
         fprintf(data_log, "IMU = [...\n");
         fgetpos(data_log, &imu_cursor);

         for (cur = 0; cur < num_blank_lines; cur++)
            fprintf(data_log, "\n");
      }

      /* append initial position line to Local Position NED file */
      void write_initial_position(Autopilot_Interface api) {
         fsetpos(data_log, &local_pos_cursor);
         fprintf(data_log, "Initial Position NED XYZ = [%f, %f, %f] [m]\n", api.initial_position.x, api.initial_position.y, api.initial_position.z);
         fgetpos(data_log, &local_pos_cursor);
      }

      void write_data(struct api_data cur_struct) {
         //print lpos and ltar
         fsetpos(data_log, &local_pos_cursor);
         fprintf(data_log, "%u, %f, %f, %f, %f, %f, %f\n", cur_struct.imu.time_usec, cur_struct.lpos.x, cur_struct.lpos.y,
                  cur_struct.lpos.z, cur_struct.ltar.x, cur_struct.ltar.y, cur_struct.ltar.z);
         printf("Local Pos and target: %f, %f, %f, %f, %f, %f\n", cur_struct.lpos.x, cur_struct.lpos.y, cur_struct.lpos.z,
                  cur_struct.ltar.x, cur_struct.ltar.y, cur_struct.ltar.z);
         fgetpos(data_log, &local_pos_cursor);

         /*Local_Pos << imu.time_usec << ", " <<
                 lpos.x << ", " << lpos.y << ", " << lpos.z << ", " <<
                 ltar.x << ", " << ltar.y << ", " << ltar.z << "\n";
         std::cout << "Local Pos and target: " << lpos.x << ", " << lpos.y << ", " << lpos.z << ", " <<
                 ltar.x << ", " << ltar.y << ", " << ltar.z << "\n";*/

         //print gpos and gtar
         fsetpos(data_log, &global_pos_cursor);
         fprintf(data_log, "%u, %d, %d, %d, %d, %d, %f\n", cur_struct.imu.time_usec, cur_struct.gpos.lat, cur_struct.gpos.lon,
                  cur_struct.gpos.alt, cur_struct.gtar.lat_int, cur_struct.gtar.lon_int, cur_struct.gtar.alt);
         fgetpos(data_log, &global_pos_cursor);

         /*Global_Pos << imu.time_usec << ", " <<
                 gpos.lat << ", " << gpos.lon << ", " << gpos.alt << ", " <<
                 gtar.lat_int << ", " << gtar.lon_int << ", " << gtar.alt << "\n";*/

         //print Attitude
         fsetpos(data_log, &attitude_cursor);
         fprintf(data_log, "%u, %f, %f, %f, %f, %f, %f\n", cur_struct.imu.time_usec, cur_struct.att.roll, cur_struct.att.pitch,
                  cur_struct.att.yaw, cur_struct.att.rollspeed, cur_struct.att.pitchspeed, cur_struct.att.yawspeed);
         fgetpos(data_log, &attitude_cursor);

         /*Attitude << imu.time_usec << ", " <<
                 att.roll << ", " << att.pitch << ", " << att.yaw << ", " <<
                 att.rollspeed << ", " << att.pitchspeed << ", " << att.yawspeed << "\n";*/

         //print IMU Data
         fsetpos(data_log, &imu_cursor);
         fprintf(data_log, "%u, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", cur_struct.imu.time_usec, cur_struct.imu.xacc, cur_struct.imu.yacc,
                  cur_struct.imu.zacc, cur_struct.imu.xgyro, cur_struct.imu.ygyro, cur_struct.imu.zgyro, cur_struct.imu.xmag,
                  cur_struct.imu.ymag, cur_struct.imu.zmag);
         fgetpos(data_log, &imu_cursor);

         /*HR_IMU << imu.time_usec << ", " <<
                 imu.xacc << ", " << imu.yacc << ", " << imu.zacc << ", " <<
                 imu.xgyro << ", " << imu.ygyro << ", " << imu.zgyro << ", " <<
                 imu.xmag << ", " << imu.ymag << ", " << imu.zmag << "\n";*/
      }

      void write_set_point_reached() {
         fsetpos(data_log, &local_pos_cursor);
         fprintf(data_log, "Out_Local Position and Target\n");
         fgetpos(data_log, &local_pos_cursor);

         fsetpos(data_log, &global_pos_cursor);
         fprintf(data_log, "Out_Local Position and Target\n");
         fgetpos(data_log, &global_pos_cursor);

         fsetpos(data_log, &attitude_cursor);
         fprintf(data_log, "Out_Local Position and Target\n");
         fgetpos(data_log, &attitude_cursor);

         fsetpos(data_log, &imu_cursor);
         fprintf(data_log, "Out_Local Position and Target\n");
         fgetpos(data_log, &imu_cursor);
      }

      void closeFile() {
         fclose(data_log);
      }
};
