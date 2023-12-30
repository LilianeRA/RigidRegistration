#ifndef COMMANDLINEPARSER_H
#define COMMANDLINEPARSER_H

#include <string>

enum struct Param {
    INT = 0, BOOLEAN = 1, REAL = 2, STRING = 3, MAT = 4, MAT_VECTOR = 5, ALGORITHM = 6, FLOAT = 7,
    UNSIGNED_INT = 8, UINT64 = 9, UCHAR = 11, SCALAR = 12
};



class CommandLineParser
{
public:

    /** @brief Constructor

    Initializes command line parser object

    @param argc number of command line arguments (from main())
    @param argv array of command line arguments (from main())
    @param keys string describing acceptable command line parameters (see class description for syntax)
    */
    CommandLineParser(int argc, const char* const argv[], const std::string& keys);

    /** @brief Copy constructor */
    CommandLineParser(const CommandLineParser& parser);

    /** @brief Assignment operator */
    CommandLineParser& operator = (const CommandLineParser& parser);

    /** @brief Destructor */
    ~CommandLineParser();

    /** @brief Returns application path

    This method returns the path to the executable from the command line (`argv[0]`).

    For example, if the application has been started with such a command:
    @code{.sh}
    $ ./bin/my-executable
    @endcode
    this method will return `./bin`.
    */
    std::string getPathToApplication() const;

    /** @brief Access arguments by name

    Returns argument converted to selected type. If the argument is not known or can not be
    converted to selected type, the error flag is set (can be checked with @ref check).

    For example, define:
    @code{.cpp}
    String keys = "{N count||}";
    @endcode

    Call:
    @code{.sh}
    $ ./my-app -N=20
    # or
    $ ./my-app --count=20
    @endcode

    Access:
    @code{.cpp}
    int N = parser.get<int>("N");
    @endcode

    @param name name of the argument
    @param space_delete remove spaces from the left and right of the string
    @tparam T the argument will be converted to this type if possible

    @note You can access positional arguments by their `@`-prefixed name:
    @code{.cpp}
    parser.get<String>("@image");
    @endcode
     */
    template <typename T>
    T get(const std::string& name, bool space_delete = true) const
    {
        T val = T();
        getByName(name, space_delete, ParamType<T>::type, (void*)&val);
        return val;
    }

    /** @brief Access positional arguments by index

    Returns argument converted to selected type. Indexes are counted from zero.

    For example, define:
    @code{.cpp}
    String keys = "{@arg1||}{@arg2||}"
    @endcode

    Call:
    @code{.sh}
    ./my-app abc qwe
    @endcode

    Access arguments:
    @code{.cpp}
    String val_1 = parser.get<String>(0); // returns "abc", arg1
    String val_2 = parser.get<String>(1); // returns "qwe", arg2
    @endcode

    @param index index of the argument
    @param space_delete remove spaces from the left and right of the string
    @tparam T the argument will be converted to this type if possible
     */
    template <typename T>
    T get(int index, bool space_delete = true) const
    {
        T val = T();
        getByIndex(index, space_delete, ParamType<T>::type, (void*)&val);
        return val;
    }

    /** @brief Check if field was provided in the command line

    @param name argument name to check
    */
    bool has(const std::string& name) const;

    /** @brief Check for parsing errors

    Returns false if error occurred while accessing the parameters (bad conversion, missing arguments,
    etc.). Call @ref printErrors to print error messages list.
     */
    bool check() const;

    /** @brief Set the about message

    The about message will be shown when @ref printMessage is called, right before arguments table.
     */
    void about(const std::string& message);

    /** @brief Print help message

    This method will print standard help message containing the about message and arguments description.

    @sa about
    */
    void printMessage() const;

    /** @brief Print list of errors occurred

    @sa check
    */
    void printErrors() const;

protected:
    void getByName(const std::string& name, bool space_delete, Param type, void* dst) const;
    void getByIndex(int index, bool space_delete, Param type, void* dst) const;

    struct Impl;
    Impl* impl;


    inline int CV_XADD(int* addr, int delta) { int tmp = *addr; *addr += delta; return tmp; };
};

template<typename _Tp, typename _EnumTp = void> struct ParamType {};

template<> struct ParamType<bool>
{
    typedef bool const_param_type;
    typedef bool member_type;

    static const Param type = Param::BOOLEAN;
};

template<> struct ParamType<int>
{
    typedef int const_param_type;
    typedef int member_type;

    static const Param type = Param::INT;
};

template<> struct ParamType<double>
{
    typedef double const_param_type;
    typedef double member_type;

    static const Param type = Param::REAL;
};

template<> struct ParamType<std::string>
{
    typedef const std::string& const_param_type;
    typedef std::string member_type;

    static const Param type = Param::STRING;
};

template<> struct ParamType<float>
{
    typedef float const_param_type;
    typedef float member_type;

    static const Param type = Param::FLOAT;
};

template<> struct ParamType<unsigned int>
{
    typedef unsigned int const_param_type;
    typedef unsigned int member_type;

    static const Param type = Param::UNSIGNED_INT;
};

template<> struct ParamType<unsigned char>
{
    typedef unsigned char const_param_type;
    typedef unsigned char member_type;

    static const Param type = Param::UCHAR;
};






#endif // COMANDLINEPARSER_H
