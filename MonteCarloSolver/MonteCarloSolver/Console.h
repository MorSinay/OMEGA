#pragma once
#include <stdarg.h>  
#include <iostream>
#include <string>
using namespace std;
class Console
{
public:
	static constexpr const char* NewLine = "\n\r";
	static void WriteLine();
	static void WriteLine(const string& s);
	static void WriteLine(char * format, ...);
	static void Write(char * format, ...);


	static char ReadChar();
	static string ReadLine();
	static string Replace(string s, string find, string replace);
	static int ReadInt32();
	static float ReadSingle();
	static double ReadDouble();
};

