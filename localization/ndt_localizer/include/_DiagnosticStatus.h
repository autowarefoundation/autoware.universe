#ifndef DIAGNOSTICSTATUS_H
#define DIAGNOSTICSTATUS_H
#include "_KeyValue.h"
#include <string>
#include <vector>
struct DiagnosticStatus {
	   std::string name;
	   std::string hardware_id;
	   int level; // Use an enum or integer to represent OK, WARN, ERROR
	   std::string message;
	   std::vector<KeyValue> values; // Replace KeyValue with pair
	 
	   // Define an enum for the levels
	   enum Level {
	       OK = 0,
	       WARN = 1,
	       ERROR = 2,
           STALE = 3
	   };
	};
#endif DIAGNOSTICSTATUS_H