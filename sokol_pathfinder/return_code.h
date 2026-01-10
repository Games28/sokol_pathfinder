#pragma once
#ifndef RETURN_CODE_STRUCT_H
#define RETURN_CODE_STRUCT_H

#include <string>

//instead of exceptions?
struct ReturnCode {
	bool valid;
	std::string msg;
};
#endif