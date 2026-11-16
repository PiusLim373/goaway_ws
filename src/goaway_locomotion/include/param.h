// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <memory>
#include <iomanip>

/* ---------- logger ---------- */
namespace spdlog
{
inline void create_logger(std::string log_path)
{
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_path, 5 * 1024 * 1024, 5);

    std::vector<spdlog::sink_ptr> sinks {console_sink, rotating_sink};
    auto logger = std::make_shared<spdlog::logger>("unitree", sinks.begin(), sinks.end());

    logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%^%l%$] %v");
    logger->flush_on(spdlog::level::info);

    spdlog::set_default_logger(logger);
}

} // namespace spdlog


namespace param
{
inline std::string VERSION = "1.0.0.1";
inline std::filesystem::path bin_path;
inline std::filesystem::path proj_dir;
inline std::filesystem::path config_dir;
inline YAML::Node config;

inline std::filesystem::path get_bin_path() {
    std::vector<char> path(1024);
    ssize_t len = readlink("/proc/self/exe", &path[0], path.size());
    if (len != -1) {
        path[len] = '\0';  // Null-terminate the result
        return std::filesystem::path(&path[0]);
    } else {
        spdlog::error("Failed to get executable path.");
        exit(1);
    }
}

/* ---------- config.yaml ---------- */
// Looks in: --config_dir, then <bin>/../config (build/bin folders), then the colcon
// install layout (lib/<pkg>/<bin> -> share/<pkg>/config), then next to the binary.
// proj_dir is the folder holding config/, so policy_dir paths stay relative to it.
inline void load_config_file(const std::string & config_dir_arg = "")
{
    assert(std::filesystem::exists(bin_path)); // run param::helper before this function
    const auto bin_dir = bin_path.parent_path();
    const auto installed = bin_dir.parent_path().parent_path() / "share" / bin_dir.filename() / "config";
    if(!config_dir_arg.empty())
    {
        config_dir = std::filesystem::absolute(config_dir_arg);
    }
    else if(bin_dir.filename() == "bin" || bin_dir.filename() == "build")
    {
        config_dir = bin_dir.parent_path() / "config";
    }
    else if(std::filesystem::exists(installed / "config.yaml"))
    {
        config_dir = installed;
    }
    else
    {
        config_dir = bin_dir;
    }
    proj_dir = config_dir.filename() == "config" ? config_dir.parent_path() : config_dir;

    const std::string config_file = (config_dir / "config.yaml").string();
    if(!std::filesystem::exists(config_file))
    {
        spdlog::critical("config.yaml not found in {}", config_dir.string());
        exit(1);
    }
    try {
        config = YAML::LoadFile(config_file);
    } catch (const YAML::Exception& e) {
        spdlog::error("Failed to load config.yaml: {}", e.what());
        exit(1);
    }
    spdlog::info("Config: {}", config_file);
}

inline std::filesystem::path parser_policy_dir(std::filesystem::path policy_dir)
{
    // Load Policy
    if (policy_dir.is_relative()) {
        policy_dir = param::proj_dir / policy_dir;
    }

    // If there is no `exported` folder in this folder,
    // then sort all the folders under this folder and take the last folder
    if (!std::filesystem::exists(policy_dir / "exported")) {
        auto dirs = std::filesystem::directory_iterator(policy_dir);
        std::vector<std::filesystem::path> dir_list;
        for (const auto& entry : dirs) {
            if (entry.is_directory()) {
                dir_list.push_back(entry.path());
            }
        }
        if (!dir_list.empty()) {
            std::sort(dir_list.begin(), dir_list.end());
            // Check if there is an `exported` folder starting from the last folder
            for (auto it = dir_list.rbegin(); it != dir_list.rend(); ++it) {
                if (std::filesystem::exists(*it / "exported")) {
                    policy_dir = *it;
                    break;
                }
            }
        }
    }
    spdlog::info("Policy directory: {}", policy_dir.string());
    return policy_dir;
}

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    bin_path = get_bin_path();

    po::options_description desc("Unitree Controller");
    desc.add_options()
        ("help,h", "produce help message")
        ("version,v", "show version")
        ("log", "record log file")
        ("log_dir", po::value<std::string>()->default_value(""), "log folder (default <proj_dir>/log)")
        ("config_dir", po::value<std::string>()->default_value(""), "folder with config.yaml")
        ("network,n", po::value<std::string>()->default_value(""), "dds network interface")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }
    if (vm.count("version"))
    {
        std::cout << "Version: " << VERSION << std::endl;
        exit(0);
    }

    load_config_file(vm["config_dir"].as<std::string>());

#ifndef NDEBUG
    spdlog::set_level(spdlog::level::debug);
#else
    spdlog::set_level(spdlog::level::info);
#endif
    if(vm.count("log"))
    {
        const auto log_arg = vm["log_dir"].as<std::string>();
        const std::filesystem::path log_dir = log_arg.empty() ? proj_dir / "log" : std::filesystem::path(log_arg);
        std::filesystem::create_directories(log_dir);
        spdlog::create_logger((log_dir / "log.txt").string());
    }

    return vm;
}

}