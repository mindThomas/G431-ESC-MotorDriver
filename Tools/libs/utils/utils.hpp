#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include <chrono>

#include <unistd.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

namespace utils {

    /********************************/
    /******* Static functions *******/
    /********************************/
    int mkdir_recursive(const char* file_path_, mode_t mode = 775);
    std::string FindFileFromPath(std::string filenameSubpart, std::string searchPath);
    std::vector<std::string> ListOfFilesPattern(std::string path, std::string pattern);
    std::vector<std::string> ListOfFiles(std::string path);
    std::string SystemCall(std::string cmd);
    bool PathExist(const std::string& path);
    bool FileExist(const std::string& path);
    bool FolderExist(const std::string& path);
    bool DeleteFile(const std::string& path);
    std::string getFileExtension(std::string filePath);
    std::string getFileNameWithExtensionFromPath(std::string filePath);
    std::string getFileNameFromPath(std::string filePath);
    std::string getPathFromFilePath(std::string filePath);
    std::string getHomeDirectory();

    std::string stringCenter(const std::string s, const int w);
    std::string printDouble(const double x, const int decDigits, const int width);
    std::string stringRight(const std::string s, const int w);
    void WriteFormattedTimestamp(std::string header, int64_t utime);
    std::string GetLogFormattedTimestamp(int64_t utime);
    int64_t GetTimestampFromImageFilename(std::string filename);
    int64_t utime();
    long int GetCurrentMicroseconds();
    long int GetCurrentMicroseconds2();
    std::chrono::milliseconds GetCurrentMicrosecondsChrono();
    std::string GetFormattedTimestamp(int64_t utime);
    std::string GetFormattedTimestampCurrent();

    void write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<float>>> dataset);
    std::vector<std::pair<std::string, std::vector<float>>> read_csv(std::string filename);

    void popupError(const char * appName, const char * fmt, ...);
    void popupError(std::string appName, const char * fmt, ...);
    void textPopup(std::string text);

    int CURL_Download(std::string URL, std::string filePath);

    float Parse2Float(std::string str);
    float Parse2RoundedFloat(std::string str);
    int Parse2Int(std::string str);
    bool Parse2Bool(std::string str);
    
    template <typename T>
    std::string to_string_with_precision(const T a_value, const int n = 6);


    /***********************/
    /******* Classes *******/
    /***********************/
    class EventSignal
    {
       boost::mutex mtx;   
       boost::condition cv;

    public:
       void trigger();       
       bool waitForEvent(long milliseconds);       
       void waitForEvent();
    };

};

#endif
