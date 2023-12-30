#include "DirUtils.h"

#include <iostream>
#include <fstream>
//#include <qdir.h>
#include <sys/stat.h>
#include <filesystem> // iteration over the directory...

// For getting the current directory
#ifdef _WIN32
#include <direct.h>
// MSDN recommends against using getcwd & chdir names
#define cwd _getcwd
#define cd _chdir
namespace fs = std::filesystem; // .. in Visual Studio style
#else
#include "unistd.h"
#include <stdio.h>
#define cwd getcwd
#define cd chdir
#include <dirent.h> // DIR
#endif



#include <regex>

DirUtils::DirUtils() 
{
}

DirUtils::~DirUtils()
{
}


bool DirUtils::PathExists(const std::string &path)
{
	struct stat buffer;
	return (stat(path.c_str(), &buffer) == 0);
		
	//return fs::exists(fs::path(path));

	//return QDir(path.c_str()).exists();
}
bool DirUtils::IsPathRelative(const std::string &path)
{
	#ifdef _WIN32
		return fs::path(path).is_relative(); 
	#else
		return (path[0] == '/' ? true : false);
	#endif
	//return QDir::isRelativePath(path.c_str());
}
bool DirUtils::IsDirectory(const std::string &path)
{
	#ifdef _WIN32
		return fs::is_directory(fs::path(path));
	#else
		struct stat s;
		if (stat(path.c_str(), &s) == 0)
		{
			if (s.st_mode & S_IFDIR)
			{
				//it's a directory
				return true;
			}
		}
		return false;
	#endif
	//return QDir(path.c_str()).isReadable();
}
bool DirUtils::IsFile(const std::string &path)
{
	#ifdef _WIN32
		return fs::is_regular_file(fs::path(path));
	#else
		struct stat s;
		if (stat(path.c_str(), &s) == 0)
		{
			if (s.st_mode & S_IFREG )
			{
				//it's a file
				return true;
			}
		}
		return false;
	#endif
		
	//return QFileInfo(path.c_str()).isReadable();
}

void DirUtils::DeleteFile(const std::string &path)
{
	#ifdef _WIN32
		fs::remove(fs::path(path));
	#else
		if (remove(path.c_str()) != 0)
			DebugLogger::Append("Error deleting file: "+path);
	#endif
}
void DirUtils::CreateDir(const std::string &path)
{
	if (!PathExists(path))
	{
		#ifdef _WIN32
			fs::create_directory(fs::path(path));
		#else
			int stat = _mkdir(path.c_str());
			if (stat)
				DebugLogger::Append("Error: Folder not created " + path);
		#endif
			
		//QDir(GetParentDir(path).c_str()).mkdir(path.c_str());
	}
}
void DirUtils::SetCurrentDir(const std::string &path)
{
	if (PathExists(path))
	{
		cd(path.c_str());
		//QDir::setCurrent(path.c_str());
	}
}

std::string DirUtils::JoinPaths(const std::string &upper_path, const std::string &sub_path)
{
	if (sub_path.size() == 0) return upper_path;
	std::string joined{ upper_path + GetSeparator() + sub_path };
	return joined;
	//return (fs::path(upper_path) / fs::path(sub_path)).string();
}
std::string DirUtils::GetCurrentDir()
{
	char buf[4096]; // never know how much is needed
	cwd(buf, sizeof buf); // will return the current dir of the project you opened in GoCAD.
	std::string current{buf};
	return current;

	// the 3 lines below works as well
	// #include <experimental/filesystem>
	//std::experimental::filesystem::v1::path currentPath = std::experimental::filesystem::current_path();
	//DebugLogger::m_Append(currentPath.generic_string());

	//fs::path curr = fs::current_path(); -> not working
	//return curr.string();

	//return QDir::currentPath().toStdString();


	/*
	OBS: the 2 blocks below gives the path to Gocad installation

	char* path;
	_get_pgmptr(&path);
	DebugLogger::m_Append(path);

	char result[MAX_PATH];
	DebugLogger::m_Append(std::string(result, GetModuleFileNameA(NULL, result, MAX_PATH)));
	*/
}
std::string DirUtils::GetFileName(const std::string &filepath)
{
	std::string name{ filepath };
	// removing the extension
	size_t found = name.find_last_of(".");
	if (found != std::string::npos)
	{
		name = name.substr(0, found);
	}
	//else name = "found != std::string::npos 1";
	// removing the parent dir
	found = name.find_last_of("\\");
	if (found != std::string::npos)
	{
		name = name.substr(found + 1);
	}
	else
	{
		found = name.find_last_of("/");
		if (found != std::string::npos)
		{
			name = name.substr(found + 1);
		}
	}
	//else name = "found != std::string::npos 2";
	name += "\0";
	return name;

	/*std::string name{ fs::path(filepath).filename() };
	// removing the extension
	size_t found = name.find_last_of(".");
	if (found != std::string::npos)
	{
		name = name.substr(0, found);
	}
	return name;*/
}
std::string DirUtils::GetFileExtension(const std::string &filepath)
{
	std::string extension{ filepath };
	size_t found = extension.find_last_of(".");
	if (found != std::string::npos)
	{
		extension = extension.substr(found+1);
	}
	else extension = "found != std::string::npos";
	extension += '\0';
	return extension;

	//return fs::path(filepath).extension().substr(1); // I don't want the dot 
}
std::string DirUtils::GetParentDir(const std::string &filepath)
{
	std::string parent_dir{ filepath };
	std::replace_if(parent_dir.begin(), parent_dir.end(), [](const char a){ return a == '/'; }, GetSeparator()[0]);
	std::replace_if(parent_dir.begin(), parent_dir.end(), [](const char a){ return a == '\\'; }, GetSeparator()[0]);
	size_t found = parent_dir.find_last_of(GetSeparator());
	if (found != std::string::npos)
	{
		parent_dir = parent_dir.substr(0, found);
	}
	//else parent_dir = "found != std::string::npos";
	else parent_dir = filepath;
	return parent_dir;

	//return fs::path(filepath).parent_path();
	//return QDir(filepath.c_str()).dirName().toStdString();
}
std::string DirUtils::GetSeparator()
{
	std::string separator{ "" };
	#if _WIN32
		separator = "\\";
	#else
		separator = "/";
	#endif // _WIN32

	return separator;
}
	
// recursive implementation
std::vector<std::string> listFilesWindows(fs::path path, bool recursive)
{
	std::vector<std::string> files;

	try
	{
		if (fs::exists(path)) files.push_back(fs::absolute(path).string());

		if (fs::is_directory(path))
		{
			using iterator = fs::directory_iterator; // < fs::path >;
			for (iterator iter(path); iter != iterator(); ++iter)
			{
				files.push_back(fs::absolute(iter->path()).string());
				if (recursive)
				{
					for (const std::string& p : listFilesWindows(iter->path(), recursive))
					{
						files.push_back(std::move(p));
					}
				}
			}
		}
	}
	catch (const std::exception&) 
	{
		//error 
	}

	return files;
}

#ifndef _WIN32
std::vector<std::string> listFilesUnix(const std::string &path, bool recursive)
{
	std::vector<std::string> files;
	DIR *d;
	struct dirent *dir;
	d = opendir(path.c_str());
	if (d)
	{
		while ((dir = readdir(d)) != NULL)
		{
			if (dir->d_name[0] == '.' || dir->d_name[1] == '.') continue;
			std::string str = path + "\\" + dir->d_name; //realpath(dir->d_name, NULL);
			files.push_back(str);
			if (recursive)
			{
				for (const std::string& p : listFilesUnix(str, recursive))
				{
					files.push_back(std::move(p));
				}
			}
		}
		closedir(d);
	}
	return files;
}
#endif
// entry point
std::vector<std::string> DirUtils::ListFiles(const std::string &path, bool recursive)
{
	#ifdef _WIN32
		return listFilesWindows(fs::path(path), recursive);
	#else
		return listFilesUnix(path, recursive);
	#endif
}


void DirUtils::ReadAllLines(const std::string &filepath, std::vector<std::string> &lines)
{
	lines.clear();
	std::ifstream in_file(filepath);
	std::string str;
	while (std::getline(in_file, str))
	{
		lines.push_back(str);
	}
	in_file.close();
}



void DirUtils::Tokenize(const std::string &text, const std::string &delimiters,
	std::vector<std::string> &tokenized)
{
	tokenized.clear();
	if (delimiters.size() == 0) return;

	// [\\s,]+ means the white space and "," are delimiters

	std::string delimiters_{ delimiters };
	// replacing the white space with the symbol of white space in between []+
	size_t f = delimiters.find(" ");
	if (f != std::string::npos)
	{
		// replacing the content from/to with some string
		delimiters_.replace(f, f + 1, "\\s");
	}
	delimiters_ = "[" + delimiters_ + "]+";

	// setting the regular expression
	const std::regex ws_re(delimiters_);
	// std::copy do a thing for every element.
	// Don't worry, empty strings are not added
	std::copy(std::sregex_token_iterator(text.begin(), text.end(), ws_re, -1),
		std::sregex_token_iterator(),
		std::back_inserter(tokenized));
	/*
	const std::regex ws_re("\\s+"); // whitespace
	std::sregex_token_iterator tok_it = std::sregex_token_iterator(text.begin(), text.end(), ws_re, -1);
	while(tok_it != std::sregex_token_iterator())
	{
	//LOG(tok_it);
	std::cout<< *tok_it <<std::endl;
	tok_it++;
	}
	*/

	//const std::regex ws_re("\\s+"); // whitespace
	//std::copy( std::sregex_token_iterator(text.begin(), text.end(), ws_re, -1),
	//           std::sregex_token_iterator(),
	//           std::ostream_iterator<std::string>(std::cout, "\n"));
}

		

/*std::string DirUtils::GetPluginHomeDirectory()
{
	char * val = std::getenv(std::string("PLUGINHOME").c_str());
	return val == nullptr ? std::string("") : std::string(val);
}*/
