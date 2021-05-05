#include "utils.hpp"

#include <stdarg.h>
#include <sstream>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/convenience.hpp>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <sys/time.h>
#include <iomanip>
#include <chrono>

// for file access (eg. directory traversing)
#include <sys/types.h>
#include <sys/stat.h>

#include <sys/types.h>
#include <dirent.h>
#include <pwd.h>

#include <curl/curl.h>

using namespace utils;

/* sys/stat.h provides you with several integers you can bytewise-OR (|) together to create your mode_t:
 * User: S_IRUSR (read), S_IWUSR (write), S_IXUSR (execute)
 * Group: S_IRGRP (read), S_IWGRP (write), S_IXGRP (execute)
 * Others: S_IROTH (read), S_IWOTH (write), S_IXOTH (execute)
 */
int utils::mkdir_recursive(const char* file_path_, mode_t mode) {
    assert(file_path_ && *file_path_);
    char file_path[strlen(file_path_)];
    strcpy(file_path, file_path_);
    char * pfile_path = file_path;
    char* p;
    for (p=strchr(pfile_path+1, '/'); p; p=strchr(p+1, '/')) {
        *p='\0';
        if (mkdir(pfile_path, mode)==-1) {
            if (errno!=EEXIST) { *p='/'; return -1; }
        }
        *p='/';
    }
    return 0;
}

std::string utils::FindFileFromPath(std::string filenameSubpart, std::string searchPath)
{
    DIR *dir;
    struct dirent *ent;
    std::string output;
    if ((dir = opendir (searchPath.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            if (ent->d_type != DT_DIR) {
                if (strstr(ent->d_name, filenameSubpart.c_str()) != NULL) {
                    output = searchPath.substr(0, searchPath.find_last_of("/")) + "/" + std::string(ent->d_name);
                    return output;
                }
            }
        }
        closedir (dir);
    } else { // could not open directory
        printf("Error: Could not open directory: %s\n", searchPath.c_str());
    }

    return output;
}

std::vector<std::string> utils::ListOfFilesPattern(std::string path, std::string pattern)
{
    DIR *dir;
    struct dirent *ent;
    std::string output;
    std::vector<std::string> files;

    if ((dir = opendir (path.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            if (ent->d_type != DT_DIR) {
                if (strstr(ent->d_name, pattern.c_str()) != NULL) {
                    files.push_back(std::string(ent->d_name));
                }
            }
        }
        closedir (dir);
    } else { // could not open directory
        printf("Error: Could not open directory: %s\n", path.c_str());
    }

    std::sort(std::begin(files), std::end(files));

    return files;
}

std::vector<std::string> utils::ListOfFiles(std::string path)
{
    DIR *dir;
    struct dirent *ent;
    std::string output;
    std::vector<std::string> files;

    if ((dir = opendir (path.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            if (ent->d_type != DT_DIR) {
                files.push_back(std::string(ent->d_name));
            }
        }
        closedir (dir);
    } else { // could not open directory
        printf("Error: Could not open directory: %s\n", path.c_str());
    }

    std::sort(std::begin(files), std::end(files));

    return files;
}

std::string utils::SystemCall(std::string cmd)
{

    std::string data;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    cmd.append(" 2>&1");

    stream = popen(cmd.c_str(), "r");
    if (stream) {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
        pclose(stream);
    }
    return data;
}

bool utils::PathExist(const std::string& path) {
    struct stat sb;
    return (stat(path.c_str(), &sb) == 0);
}

bool utils::FileExist(const std::string& path) {
    struct stat sb;
    return (stat(path.c_str(), &sb) == 0 && S_ISREG(sb.st_mode));
}

bool utils::FolderExist(const std::string& path) {
    struct stat sb;
    return (stat(path.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)); // folder exists
}

bool utils::DeleteFile(const std::string& path) {
    if (remove(path.c_str()) != 0)
        return false; // error
    else
        return true;
}

/*
 * Get File extension from File path or File Name
 */
std::string utils::getFileExtension(std::string filePath)
{
    // Create a Path object from given string
    boost::filesystem::path pathObj(filePath);
    // Check if file name in the path object has extension
    if (pathObj.has_extension()) {
        // Fetch the extension from path object and return
        return pathObj.extension().string();
    }
    // In case of no extension return empty string
    return "";
}

std::string utils::getFileNameWithExtensionFromPath(std::string filePath)
{
    return boost::filesystem::path(filePath).filename().string();
}

std::string utils::getFileNameFromPath(std::string filePath)
{
    return boost::filesystem::path(filePath).stem().string();
}

std::string utils::getPathFromFilePath(std::string filePath)
{
    return boost::filesystem::path(filePath).parent_path().string() + "/";
}

std::string utils::getHomeDirectory()
{
    const char *homedir;

    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }

    return std::string(homedir);
}


/* Center-aligns string within a field of width w. Pads with blank spaces
    to enforce alignment. */
std::string utils::stringCenter(const std::string s, const int w) {
    std::stringstream ss, spaces;
    int padding = w - s.size();                 // count excess room to pad
    for(int i=0; i<padding/2; ++i)
        spaces << " ";
    ss << spaces.str() << s << spaces.str();    // format with padding
    if(padding>0 && padding%2!=0)               // if odd #, add 1 space
        ss << " ";
    return ss.str();
}

/* Convert double to string with specified number of places after the decimal
   and left padding. */
std::string utils::printDouble(const double x, const int decDigits, const int width) {
    std::stringstream ss;
    ss << std::fixed << std::right;
    ss.fill(' ');        // fill space around displayed #
    ss.width(width);     // set  width around displayed #
    ss.precision(decDigits); // set # places after decimal
    ss << x;
    return ss.str();
}

/* Convert double to string with specified number of places after the decimal
   and left padding. */
std::string utils::stringRight(const std::string s, const int w) {
    std::stringstream ss;
    ss << std::fixed << std::right;
    ss.fill(' ');        // fill space around displayed #
    ss.width(w);     // set  width around displayed #
    ss << s;
    return ss.str();
}

void utils::WriteFormattedTimestamp(std::string header, int64_t utime)
{
    std::time_t seconds = utime / 1000000;
    int milliseconds = (utime % 1000000) / 1000;
    std::tm* t = std::gmtime(&seconds); // std::localtime
    std::cout << header << ": ";
    std::cout << std::put_time(t, "%Y-%m-%d %H:%M:%S");
    printf(".%03d\n", milliseconds);
}

std::string utils::GetLogFormattedTimestamp(int64_t utime)
{
    std::string output;
    std::stringstream ss;

    std::time_t seconds = utime / 1000000;
    int milliseconds = (utime % 1000000) / 1000;
    std::tm* t = std::gmtime(&seconds); // std::localtime

    ss << std::put_time(t, "%Y-%m-%d_%H-%M-%S");
    ss << boost::format("-%03d") % milliseconds;
    output = ss.str();

    return output;
}

int64_t utils::GetTimestampFromImageFilename(std::string filename)
{
    std::tm t;
    std::istringstream ss(filename.substr(0, 19));
    std::istringstream ss2(filename.substr(20, 3));
    int milliseconds;
    int64_t utime;

    ss >> std::get_time(&t, "%Y-%m-%d_%H-%M-%S.jpg");
    if (ss.fail()) return 0;

    std::time_t seconds = std::mktime(&t);
    if (!(ss2 >> milliseconds)) return 0;

    utime = 1000000 * seconds + 1000 * milliseconds;
    return utime;
}

int64_t utils::utime()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

long int utils::GetCurrentMicroseconds()
{
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long int us = tp.tv_sec * 1000000 + tp.tv_usec;
    return us;
}

long int utils::GetCurrentMicroseconds2()
{
    std::chrono::microseconds us = std::chrono::duration_cast< std::chrono::microseconds >(
            std::chrono::system_clock::now().time_since_epoch()
    );

    return us.count();
}

std::chrono::milliseconds GetCurrentMicrosecondsChrono()
{
    using namespace std::chrono;
    milliseconds ms = duration_cast< milliseconds >(
            system_clock::now().time_since_epoch()
    );
}

std::string utils::GetFormattedTimestamp(int64_t utime)
{
    std::string output;
    std::stringstream ss;

    std::time_t seconds = utime / 1000000;
    int milliseconds = (utime % 1000000) / 1000;
    std::tm* t = std::localtime(&seconds);

    ss << std::put_time(t, "%Y-%m-%d_%H-%M-%S");
    ss << boost::format("-%03d") % milliseconds;
    output = ss.str();

    return output;
}

std::string utils::GetFormattedTimestampCurrent()
{
    std::string output;
    std::stringstream ss;

    int64_t utime = GetCurrentMicroseconds();

    std::time_t seconds = utime / 1000000;
    int milliseconds = (utime % 1000000) / 1000;
    std::tm* t = std::localtime(&seconds);

    ss << std::put_time(t, "%Y-%m-%d_%H-%M-%S");
    ss << boost::format("-%03d") % milliseconds;
    output = ss.str();

    return output;
}

void utils::write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<float>>> dataset)
{
    // Make a CSV file with one or more columns of integer values
    // Each column of data is represented by the pair <column name, column data>
    //   as std::pair<std::string, std::vector<int>>
    // The dataset is represented as a vector of these columns
    // Note that all columns should be the same size

    // Create an output filestream object
    std::ofstream myFile(filename);

    // Send column names to the stream
    for(int j = 0; j < dataset.size(); ++j)
    {
        myFile << dataset.at(j).first;
        if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
    }
    myFile << "\n";

    // Send data to the stream
    for(int i = 0; i < dataset.at(0).second.size(); ++i)
    {
        for(int j = 0; j < dataset.size(); ++j)
        {
            myFile << dataset.at(j).second.at(i);
            if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
        }
        myFile << "\n";
    }

    // Close the file
    myFile.close();
}

std::vector<std::pair<std::string, std::vector<float>>> utils::read_csv(std::string filename)
{
    // Reads a CSV file into a vector of <string, vector<int>> pairs where
    // each pair represents <column name, column values>

    // Create a vector of <string, int vector> pairs to store the result
    std::vector<std::pair<std::string, std::vector<float>>> result;

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if(!myFile.is_open()) throw std::runtime_error("Could not open file");

    // Helper vars
    std::string line, colname;
    float val;

    // Read the column names
    if(myFile.good())
    {
        // Extract the first line in the file
        std::getline(myFile, line);

        // Create a stringstream from line
        std::stringstream ss(line);

        // Extract each column name
        while(std::getline(ss, colname, ',')){

            // Initialize and add <colname, int vector> pairs to result
            result.push_back({colname, std::vector<float> {}});
        }
    }

    // Read data, line by line
    while(std::getline(myFile, line))
    {
        // Create a stringstream of the current line
        std::stringstream ss(line);

        // Keep track of the current column index
        int colIdx = 0;

        // Extract each integer
        while(ss >> val){

            // Add the current integer to the 'colIdx' column's values vector
            result.at(colIdx).second.push_back(val);

            // If the next token is a comma, ignore it and move on
            if(ss.peek() == ',') ss.ignore();

            // Increment the column index
            colIdx++;
        }
    }

    // Close file
    myFile.close();

    return result;
}

void utils::popupError(const char * appName, const char * fmt, ...)
{
    int n;
    int size = 100;     /* Guess we need no more than 100 bytes */
    char *p, *np;
    va_list args;

    if ((p = (char *)malloc(size)) == NULL)
        return;

    while (1) {
        /* Try to print in the allocated space */
        va_start(args, fmt);
        n = vsnprintf(p, size, fmt, args);
        va_end(args);

        /* Check error code */
        if (n < 0)
            return;

        /* If that worked, use the string */
        if (n < size)
            break;

        /* Else try again with more space */
        size = n + 1;       /* Allocate exactly what is needed */

        if ((np = (char *)realloc(p, size)) == NULL) {
            free(p);
            return;
        } else {
            p = np;
        }
    }

    printf("Error: %s", p);

    std::string notifyCmd = "notify-send '" + std::string(appName) + "' '" + std::string(p) + "'";
    system(notifyCmd.c_str());

    free(p);
}

void utils::popupError(std::string appName, const char * fmt, ...)
{
    va_list args;

    va_start(args, fmt);

    popupError(appName.c_str(), fmt, args);

    va_end(args);
}

void utils::textPopup(std::string text)
{
    std::string popupCmd = "yad --text '" + std::string(text) + "'";
    std::cout << text << std::endl;
    system(popupCmd.c_str());
}

typedef struct MemoryStruct {
    char *memory;
    size_t size;
} MemoryStruct;

static size_t
WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    size_t realsize = size * nmemb;
    MemoryStruct *mem = (MemoryStruct *)userp;

    char *ptr = (char *)realloc(mem->memory, mem->size + realsize + 1);
    if(ptr == NULL) {
        /* out of memory! */
        printf("not enough memory (realloc returned NULL)\n");
        return 0;
    }

    mem->memory = ptr;
    memcpy(&(mem->memory[mem->size]), contents, realsize);
    mem->size += realsize;
    mem->memory[mem->size] = 0;

    return realsize;
}

int utils::CURL_Download(std::string URL, std::string filePath)
{
    CURLcode ret;
    CURL *curl;
    FILE *fptr;

    MemoryStruct chunk;
    chunk.memory = (char *)malloc(1);  /* will be grown as needed by the realloc above */
    chunk.size = 0;    /* no data at this point */

    fptr = fopen(filePath.c_str(), "wb");
    if (!fptr) return -1;

    curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, URL.c_str());
    curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 1L);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "curl/7.35.0");
    curl_easy_setopt(curl, CURLOPT_MAXREDIRS, 50L);
    curl_easy_setopt(curl, CURLOPT_TCP_KEEPALIVE, 1L);

    curl_easy_setopt(curl, CURLOPT_HEADER, 1L);  //Enable Headers
    //curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeDataOnStream);
    //curl_easy_setopt(curl, CURLOPT_WRITEDATA, stdout);   //Print data in STDOUT

    /* send all data to this function  */
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);

    /* we pass our 'chunk' struct to the callback function */
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&chunk);

    /* Here is a list of options the curl code used that cannot get generated
       as source easily. You may select to either not use them or implement
       them yourself.

       CURLOPT_WRITEDATA set to a objectpointer
       CURLOPT_WRITEFUNCTION set to a functionpointer
       CURLOPT_READDATA set to a objectpointer
       CURLOPT_READFUNCTION set to a functionpointer
       CURLOPT_SEEKDATA set to a objectpointer
       CURLOPT_SEEKFUNCTION set to a functionpointer
       CURLOPT_ERRORBUFFER set to a objectpointer
       CURLOPT_STDERR set to a objectpointer
       CURLOPT_HEADERFUNCTION set to a functionpointer
       CURLOPT_HEADERDATA set to a objectpointer

     */

    ret = curl_easy_perform(curl);

    curl_easy_cleanup(curl);
    curl = NULL;

    const char pattern[4] = {0x0D, 0x0A, 0x0D, 0x0A};
    size_t offset = strstr(chunk.memory, pattern) - chunk.memory + 4;

    // Write content to file
    fwrite(&chunk.memory[offset], 1, chunk.size - offset, fptr);
    fclose(fptr);

    free(chunk.memory);

    return (int) ret;
}


float utils::Parse2Float(std::string str)
{
    float value;
    try {
        value = std::stof(str);
    }
    catch (std::invalid_argument &e) {
        return 0;
    }  // value could not be parsed (eg. is not a number)
    catch (std::out_of_range &e) {
        return 0;
    }
    return value;
}

float utils::Parse2RoundedFloat(std::string str)
{
    float value;
    try {
        value = std::stof(str);
        value = roundf(value * 1000) / 1000; // round to 3 decimals
    }
    catch (std::invalid_argument &e) {
        return 0;
    }  // value could not be parsed (eg. is not a number)
    catch (std::out_of_range &e) {
        return 0;
    }
    return value;
}

int utils::Parse2Int(std::string str)
{
    int value;
    try {
        value = std::stoi(str);
    }
    catch (std::invalid_argument &e) {
        return 0;
    }  // value could not be parsed (eg. is not a number)
    catch (std::out_of_range &e) {
        return 0;
    }
    return value;
}

bool utils::Parse2Bool(std::string str)
{
    if (!str.compare("true"))
        return true;
    else if (!str.compare("false"))
        return false;
    else {
        return false;
    }
}

template <typename T>
std::string utils::to_string_with_precision(const T a_value, const int n)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}


void utils::EventSignal::trigger()
{
    cv.notify_one();
}

bool utils::EventSignal::waitForEvent(long milliseconds)
{
    boost::mutex::scoped_lock lk(mtx);
    boost::posix_time::time_duration wait_duration = boost::posix_time::milliseconds(milliseconds); 
    const boost::system_time timeout = boost::get_system_time() + wait_duration; 
    return cv.timed_wait(lk, timeout); // wait until signal Event 
}

void utils::EventSignal::waitForEvent()
{
    boost::mutex::scoped_lock lk(mtx);
    cv.wait(lk);
}
