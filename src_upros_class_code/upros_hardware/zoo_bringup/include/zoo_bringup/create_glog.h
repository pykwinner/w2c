#ifndef CREATE_GLOG_H__
#define CREATE_GLOG_H__

#include <glog/logging.h>
#include <iostream>
#include <sys/stat.h>

inline void mkdirs(const char *dir)
{
    int i, len;
    char str[100];
    strcpy(str, dir); // 缓存文件路径
    len = strlen(str);
    for (i = 0; i < len; i++)
    {
        if (str[i] == '/')
        {
            str[i] = '\0';
            if (access(str, 0) != 0)
                mkdir(str, S_IRWXU | S_IRWXG | S_IRWXO);
            str[i] = '/';
        }
    }
    if (len > 0 && access(str, 0) != 0) // 检测是否创建成功
        mkdir(str, S_IRWXU | S_IRWXG | S_IRWXO);
}

inline void initLog(char *const *argv)
{
    std::string program_path(argv[0]);
    int last_line = program_path.find_last_of('/');
    // 进程名
    std::string program_name = program_path.substr(last_line + 1, program_path.size() - last_line);
    int second_line = program_path.find('/', 6);
    std::string home_name = program_path.substr(0, second_line);
    std::string logDirStr = home_name + "/" + "zyzx_hardware" + "_log";
    time_t t;
    time(&t);
    std::string time_str(ctime(&t));
    logDirStr = logDirStr;
    mkdirs(logDirStr.c_str());

    FLAGS_logtostderr = false;             // 设置日志消息是否转到标准输出而不是日志文件(false)
    FLAGS_stderrthreshold = google::ERROR; // 严重性级别在该门限值以上的日志信息除了写入日志文件以外，还要输出到stderr。
    // 各严重性级别对应的数值：INFO—0，WARNING—1，ERROR—2，FATAL—3  默认值为2.
    FLAGS_minloglevel = google::INFO;            // 严重性级别在该门限值以上的日志信息才进行记录。默认值为0.
    FLAGS_log_dir = logDirStr;                   // 设置日志文件保存目录,这个目录必须是已经存在的,否则不能生成日志文件.
    FLAGS_log_prefix = true;                     // 设置日志前缀是否应该添加到每行输出
    FLAGS_logbufsecs = 0;                        // 设置可以缓冲日志的最大秒数，0指实时输出
    FLAGS_max_log_size = 10;                     // 设置最大日志文件大小（以MB为单位）
    FLAGS_stop_logging_if_full_disk = true;      // 设置是否在磁盘已满时避免日志记录到磁盘
    google::InitGoogleLogging(argv[0]);          // 全局初始化glog，argv[0]是程序名
    google::SetStderrLogging(google::GLOG_INFO); // 设置glog的输出级别，这里的含义是输出INFO级别以上的信息
    google::SetLogDestination(google::GLOG_INFO, std::string(logDirStr + "/info_move_base_").c_str());
    google::SetLogDestination(google::GLOG_WARNING, std::string(logDirStr + "/warn_move_base_").c_str());
    google::SetLogDestination(google::GLOG_ERROR, std::string(logDirStr + "/error_move_base_").c_str());
    FLAGS_colorlogtostderr = true; // 开启终端颜色区分

    LOG(INFO) << "current process id is " << getpid();
}

#endif
