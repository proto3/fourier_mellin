#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>

namespace fs = boost::filesystem;

class ConfigLoader
{
public:
    //------------------------------------------------------------------------//
    explicit ConfigLoader(std::string filename);
    //------------------------------------------------------------------------//
    static void set_config_folder_path(std::string path_str)
    {
		config_folder_path = fs::path(path_str);

        if(!exists(config_folder_path) || !is_directory(config_folder_path))
        {
            std::cerr << "Incorrect path for configuration files." << std::endl;
            exit(-1);
        }
    }
    //------------------------------------------------------------------------//
    static std::string get_config_folder_path()
    {
        return config_folder_path.string();
    }
    //------------------------------------------------------------------------//
    template <typename T>
    T get(std::string value_name)
    {
        try
        {
            return config[value_name].as<T>();
        }
        catch(YAML::TypedBadConversion<T>)
        {
            std::cerr << "Missing or bad type key \"" << value_name << "\" in \"" << config_file_path.string() << "\"" << std::endl;
            exit(-1);
        }
    }
    //------------------------------------------------------------------------//
    template <typename T>
    T get_or_default(std::string value_name, T value_default)
    {
        try
        {
            return config[value_name].as<T>();
        }
        catch(YAML::TypedBadConversion<T>)
        {
            return value_default;
        }
    }
    //------------------------------------------------------------------------//
    //use a vector of at least 2 elements
    template <class T>
    T get(std::vector<std::string> v)
    {
        YAML::Node node = config[v[0]];

        unsigned long i;
        for(i = 1; i < v.size()-1; i++)
        {
            node = node[v[i]];
        }
        return node[v[i]].as<T>();
    }
    //------------------------------------------------------------------------//
private:
    static fs::path config_folder_path;
    fs::path config_file_path;
    YAML::Node config;
};

#endif
