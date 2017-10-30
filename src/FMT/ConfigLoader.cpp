#include <FMT/ConfigLoader.h>
#include <iostream>

fs::path ConfigLoader::config_folder_path;
//----------------------------------------------------------------------------//
ConfigLoader::ConfigLoader(std::string filename)
{
    config_file_path = config_folder_path / fs::path(filename);

    if(!exists(config_file_path))
    {
        std::cerr << config_file_path.string() << " does not exist." << std::endl;
        exit(-1);
    }

    if(is_directory(config_file_path))
    {
        std::cerr << config_file_path.string() << " is not a valid config file." << std::endl;
        exit(-1);
     }

    config = YAML::LoadFile(config_file_path.string());
}
//----------------------------------------------------------------------------//
