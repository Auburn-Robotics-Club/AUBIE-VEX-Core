#include "../logger.h"

Log coreLogger = Log();

//Log
//--------------------------------------------------------------------------------------------------
std::string replaceStr(std::string str, const std::string& from, const std::string& to) {
  size_t start_pos = 0;
  while((start_pos = str.find(from, start_pos)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length(); // Handles case where 'to' is a substring of 'from'
  }
  return str;
}
  
void Log::formatInput(std::string &s) {
  s = replaceStr(s, "\n", "");
}
Log::Log(){
  setActive(false);
  clear();
}
Log::Log(bool activeIn){
  setActive(activeIn);
  clear();
}
void Log::setActive(bool activeIn){
  doLog = activeIn;
}
void Log::clear(){
  internalString.clear();
}
void Log::print(bool clearBuf){
  if(doLog){
    std::cout << internalString << std::flush;
    if(clearBuf){
      clear();
    }
  }
}
void Log::save(const char* fileName, bool clearBuf){
  if(doLog){
    std::ofstream file(fileName);
    if(file.is_open()){
      file.clear();
      file << internalString << std::flush;
      file.close();
    }
    if(clearBuf){
      clear();
    }
  }
}
void Log::append(const char* fileName, bool clearBuf){
  if(doLog){
    std::ofstream file(fileName, std::ios_base::app);
    if(file.is_open()){
      file << internalString << std::flush;
      file.close();
      if(clearBuf){
        clear();
      }
    }
  }
}
//FRAME start, FRAME end, etc
void Log::meta(std::string label, std::string data){
  if(doLog){
    formatInput(label);
    formatInput(data);
    char buffer [100];
    sprintf_s(buffer, "%d,%s,%s\n", META_DATA, label.c_str(), data.c_str());
    std::string result = std::string(buffer);
    result.shrink_to_fit();
    internalString += result;
  }
}
void Log::logTime(double t){
  if(doLog){
    char buffer [100];
    sprintf_s(buffer, "%d,TIME,%.2f\n", META_DATA, t);
    std::string result = std::string(buffer);
    result.shrink_to_fit();
    internalString += result;
  }
}
void Log::logTimeout(std::string label, int isDone, int isMoving, int atTarget, int timedOut){
  if(doLog){
    formatInput(label);
    char buffer [100];
    sprintf_s(buffer, "%d,%s,%d,%d,%d,%d\n", TIMEOUT_DATA, label.c_str(), isDone, isMoving, atTarget, timedOut);
    std::string result = std::string(buffer);
    result.shrink_to_fit();
    internalString += result;
  }
}
void Log::logPoint(std::string label, Point2d p){
  if(doLog){
    formatInput(label);
    char buffer [100];
    sprintf_s(buffer, "%d,%s,%.2f,%.2f\n", POINT_DATA, label.c_str(), p.x, p.y);
    std::string result = std::string(buffer);
    result.shrink_to_fit();
    internalString += result;
  }
}
void Log::logVector(std::string label, Vector2d p){
  if(doLog){
    formatInput(label);
    char buffer [100];
    sprintf_s(buffer, "%d,%s,%.5f,%.5f\n", VECTOR_DATA, label.c_str(), p.getX(), p.getY());
    std::string result = std::string(buffer);
    result.shrink_to_fit();
    internalString += result;
  }
}
void Log::logPointSet(std::string label, positionSet p){
  if(doLog){
    formatInput(label);
    char buffer [100];
    sprintf_s(buffer, "%d,%s,%.2f,%.2f,%.2f\n", POINTSET_DATA, label.c_str(), p.p.x, p.p.y, radToDeg(p.head));
    std::string result = std::string(buffer);
    result.shrink_to_fit();
    internalString += result;
  }
}
void Log::logPID(std::string label, double error, double p, double i, double d){
  if(doLog){
    formatInput(label);
    char buffer [200];
    sprintf_s(buffer, "%d,%s,%.2f,%.2f%.2f%.2f\n", PID_DATA, label.c_str(), error, p, i, d);
    std::string result = std::string(buffer);
    result.shrink_to_fit();
    internalString += result;
  }
}
void Log::status(std::string label, bool stat, int priority){
  if(doLog){
    formatInput(label);
    char buffer [100];
    sprintf_s(buffer, "%d,%s,%d\n", priority, label.c_str(), stat);
    std::string result = std::string(buffer);
    result.shrink_to_fit();
    internalString += result;
  }
}
void Log::status(std::string label, double stat, int priority){
  if(doLog){
    formatInput(label);
    char buffer [100];
    sprintf_s(buffer, "%d,%s,%.3f\n", priority, label.c_str(), stat);
    std::string result = std::string(buffer);
    result.shrink_to_fit();
    internalString += result;
  }
}
void Log::info(std::string label, std::string data){
  if(doLog){
    formatInput(label);
    formatInput(data);
    char buffer [200];
    sprintf_s(buffer, "%d,%s,%s\n", INFO, label.c_str(), data.c_str());
    std::string result = std::string(buffer);
    result.shrink_to_fit();
    internalString += result;
  }
}
void Log::warning(std::string label, std::string data){
  if(doLog){
    formatInput(label);
    formatInput(data);
    char buffer [200];
    sprintf_s(buffer, "%d,%s,%s\n", WARNING, label.c_str(), data.c_str());
    std::string result = std::string(buffer);
    result.shrink_to_fit();
    internalString += result;
  }
}
void Log::error(std::string label, std::string data){
  if(doLog){
    formatInput(label);
    formatInput(data);
    char buffer [200];
    sprintf_s(buffer, "%d,%s,%s\n", ERROR, label.c_str(), data.c_str());
    std::string result = std::string(buffer);
    result.shrink_to_fit();
    internalString += result;
  }
}