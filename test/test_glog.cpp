#define GOOGLE_GLOG_DLL_DECL
#include <glog/logging.h>
int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);

    google::SetLogDestination(google::INFO, ".\\logs"); // 日志存放目录和日志文件前缀

    // 如果需要同时输出到文件和控制台
    FLAGS_alsologtostderr = 1;

    LOG(INFO) << "INFO : Hello, glog!";       // 信息
    LOG(WARNING) << "WARNING : Hello, glog!"; // 警告
    LOG(ERROR) << "ERROR : Hello, glog!";     // 错误
    LOG(FATAL) << "FATAL : Hello, glog!";     // 致命 ,有FATAL会直接终止程序

    return 0;
}
