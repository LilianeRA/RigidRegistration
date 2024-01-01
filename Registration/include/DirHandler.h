// A static class to manipulate files, directories and paths
// This is a substitute for std::filesystem os C++20, which does not work here
#ifndef DIRHANDLER_H
#define DIRHANDLER_H

#include <vector>
#include <string>

class DirHandler
{
public:
	DirHandler();
	virtual ~DirHandler();

	static bool PathExists(const std::string &path);
	static bool IsPathRelative(const std::string &path);
	static bool IsDirectory(const std::string &path);
	static bool IsFile(const std::string &path);

	static void DeleteFile(const std::string &path);
	static void CreateDir(const std::string &path);
	static void SetCurrentDir(const std::string &path);

	static std::string JoinPaths(const std::string &upper_path, const std::string &sub_path); //pode dar problema com as \ e /
	static std::string GetCurrentDir(); 
	static std::string GetFileName(const std::string &filepath);
	static std::string GetFileExtension(const std::string &filepath);
	static std::string GetParentDir(const std::string &filepath);
	static std::string GetSeparator();

	static std::vector<std::string> ListFiles(const std::string &path, bool recursive = true);

	static void ReadAllLines(const std::string &filepath, std::vector<std::string> &lines);
	static void Tokenize(const std::string &text, const std::string &delimiters, std::vector<std::string> &tokenized);


	//static std::string GetPluginHomeDirectory(); // get plugin instalation directory

};


#endif