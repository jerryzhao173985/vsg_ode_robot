#include <stdlib.h>
#include <time.h>
#include <sys/stat.h>

#include "trackrobots.h"
#include "abstractrobot.h"
#include "matrix.h"

bool TrackRobot::open(const Trackable* robot) {
    if (!robot) {
        std::cout << "DEBUG: Failed - robot pointer is null" << std::endl;
        return false;
    }

    if (isTrackingSomething() && conf.writeFile) {
        std::cout << "DEBUG: Tracking something and writeFile is true" << std::endl;
        
        if (file) {
            std::cout << "DEBUG: Closing existing file" << std::endl;
            fclose(file);
        }

        char filename[1024];
        if (conf.autoFilename) {
            std::cout << "DEBUG: Using auto filename" << std::endl;
            
            char date[128];
            time_t t = time(0);
            struct tm* timeinfo = localtime(&t);
            if (!timeinfo) {
                std::cout << "DEBUG: Failed - localtime returned null" << std::endl;
                return false;
            }
            
            if (strftime(date, sizeof(date), "%F_%H-%M-%S", timeinfo) == 0) {
                std::cout << "DEBUG: Failed - strftime returned 0" << std::endl;
                return false;
            }
            
            std::cout << "DEBUG: Got date string: " << date << std::endl;
            
            if (conf.scene.empty()) {
                std::cout << "DEBUG: Failed - scene name is empty" << std::endl;
                return false;
            }
            
            std::string trackableName = robot->getTrackableName();
            if (trackableName.empty()) {
                std::cout << "DEBUG: Failed - trackable name is empty" << std::endl;
                return false;
            }
            
            std::cout << "DEBUG: Scene: " << conf.scene << ", Trackable: " << trackableName << std::endl;
            
            if (conf.id >= 0) {
                int result = snprintf(filename, sizeof(filename), "%s_track_%s_%i_%s.log",
                            trackableName.c_str(), conf.scene.c_str(), conf.id, date);
                if (result >= sizeof(filename)) {
                    std::cout << "DEBUG: Failed - filename buffer would overflow (auto with id)" << std::endl;
                    return false;
                }
            } else {
                int result = snprintf(filename, sizeof(filename), "%s_track_%s_%s.log",
                            trackableName.c_str(), conf.scene.c_str(), date);
                if (result >= sizeof(filename)) {
                    std::cout << "DEBUG: Failed - filename buffer would overflow (auto without id)" << std::endl;
                    return false;
                }
            }
        } else {
            std::cout << "DEBUG: Using manual filename" << std::endl;
            
            if (conf.scene.empty()) {
                std::cout << "DEBUG: Failed - scene name is empty" << std::endl;
                return false;
            }
            
            if (conf.id >= 0) {
                int result = snprintf(filename, sizeof(filename), "%s_%i.log", conf.scene.c_str(), conf.id);
                if (result >= sizeof(filename)) {
                    std::cout << "DEBUG: Failed - filename buffer would overflow (manual with id)" << std::endl;
                    return false;
                }
            } else {
                int result = snprintf(filename, sizeof(filename), "%s.log", conf.scene.c_str());
                if (result >= sizeof(filename)) {
                    std::cout << "DEBUG: Failed - filename buffer would overflow (manual without id)" << std::endl;
                    return false;
                }
            }
        }
        
        std::cout << "DEBUG: Attempting to open file: " << filename << std::endl;
        file = fopen(filename, "w");
        if (!file) {
            std::cout << "DEBUG: Failed to open file: " << filename << std::endl;
            return false;
        }

        std::cout << "DEBUG: Successfully opened file" << std::endl;
        
        fprintf(file, "#C t");
        if (conf.trackPos) fprintf(file, " x y z");
        if (conf.trackSpeed) fprintf(file, " vx vy vz wx wy wz");
        if (conf.trackOrientation) fprintf(file, " o11 o12 o13 o21 o22 o23 o31 o32 o33");
        fprintf(file, "\n");
        fprintf(file, "# Recorded every %ith time step\n", conf.interval);
    } else {
        std::cout << "DEBUG: Not tracking or writeFile is false" << std::endl;
        std::cout << "DEBUG: isTrackingSomething() = " << (isTrackingSomething() ? "true" : "false") << std::endl;
        std::cout << "DEBUG: conf.writeFile = " << (conf.writeFile ? "true" : "false") << std::endl;
    }

    return true;
}

void TrackRobot::track(const Trackable* robot, double time)
{
  if(!file || !robot)
    return;

  if(cnt % conf.interval==0){
    //   fprintf(file, "%li ", cnt);
    fprintf(file, "%f", time);
    if(conf.trackPos){
      Position p = robot->getPosition();
      fprintf(file, " %g %g %g", p.x, p.y, p.z);
      // std::cout<< "Position: " << p.x << " " << p.y << " " << p.z << std::endl;
    }
    if(conf.trackSpeed){
      Position s = robot->getSpeed();
      fprintf(file, " %g %g %g", s.x, s.y, s.z);
      s = robot->getAngularSpeed();
      fprintf(file, " %g %g %g", s.x, s.y, s.z);
    }
    if( conf.trackOrientation){
      const matrix::Matrix& o = robot->getOrientation();
      for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
          fprintf(file, " %g", o.val(i,j));
        }
      }
    }
    fprintf(file, "\n");
  }
  cnt++;
}

void TrackRobot::close()
{
  if(file)
    fclose(file);
  file = 0;
}

// void TrackRobot::deepcopy (TrackRobot &lhs, const TrackRobot &rhs)
// {
//   lhs.conf          = rhs.conf;
//   lhs.cnt           = rhs.cnt;
//   lhs.conf.id       = rhs.conf.id+100;
//   lhs.file          = 0;
// }



