#ifndef PARAMPARSE_H
#define PARAMPARSE_H

#include <iostream>
#include "yaml-cpp/yaml.h"
#include "utils/common.h"

using namespace std;

void getParameter(const string& addr, InitParam& params);

#endif