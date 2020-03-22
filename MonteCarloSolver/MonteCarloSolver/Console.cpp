/**
* Authors: Oleg Maksimov, Guy Levy, Aviad Fuchs and Mor Sinay, AIM Lab, Bar-Ilan University
*
* The MIT License (MIT)
*
* Copyright (c) 2018 AIM Lab, Bar-Ilan University
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

#include "Console.h"
#include <stdarg.h>  
#include <iostream>
#include <string>
using namespace std;

void Console::Write(char* format, ...)
{
	va_list argptr;
	va_start(argptr, format);
	vfprintf(stderr, format, argptr);
	va_end(argptr);
}

void Console::WriteLine() { Console::WriteLine(""); }

void Console::WriteLine(const string& s) {
	
	return WriteLine(s.c_str());
}

void Console::WriteLine(char* format, ...)
{
	va_list argptr;
	va_start(argptr, format);
	vfprintf(stderr, format, argptr);
	va_end(argptr);
	printf(Console::NewLine);
}

char Console::ReadChar() { return getchar(); }

string Console::ReadLine()
{
	string line;
	getline(cin, line);
	return line;
}


string Console::Replace(string s, string find, string replace)
{
	auto replaced = s;
	auto pos = string::npos;
	while ((pos = replaced.find(find)) != string::npos)
		replaced.replace(pos, find.length(), replace);
	return replaced;
}

int Console::ReadInt32() { return std::stoi(Replace(Replace(ReadLine(), "m", "000000"), "k", "000")); }
float Console::ReadSingle() { return std::stof(ReadLine()); }
double Console::ReadDouble() { return std::stod(ReadLine()); }
