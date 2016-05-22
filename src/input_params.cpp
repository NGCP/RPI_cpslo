/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   input_params.cpp
 * Author: karthik
 * 
 * Created on May 19, 2016, 2:52 PM
 */

#include "input_params.h"
#include <string.h>
#include <fstream>

using namespace std;

// <TODO: Fix default constructor!>
input_params::input_params() {
}

input_params::input_params(string filename) {
	this->filename = filename;
	parseFile(this->filename);
}

input_params::~input_params() { }

void input_params::parseFile(string fname) {
	string line;
	try {
		ifstream inputs(fname.c_str());
		while (getline(inputs, line)) {
			parseLine(line);
		}
		inputs.close();
	}	catch (ifstream::failure e) {
		// implement error log output
		throw e;
	}
}

void input_params::parseLine(string line) {
	char *duplicate = strdup(line.c_str());
	char *tok = strtok(duplicate, " ,=");
	if (strcmp(tok, "altitude") == 0 && !altitude.isWritten) {
		tok = strtok(NULL, " ,=");
		altitude.value = strtof(tok, NULL);
		altitude.isWritten = true;
	}
	if (strcmp(tok, "diagonal") == 0 && !diagonal.isWritten) {
		tok = strtok(NULL, " ,=");
		diagonal.value = strtof(tok, NULL);
		diagonal.isWritten = true;
	}
	if (strcmp(tok, "tolerance") == 0 && !tolerance.isWritten) {
		tok = strtok(NULL, " ,=");
		tolerance.value = strtof(tok, NULL);
		tolerance.isWritten = true;
	}
	if (strcmp(tok, "frame_width") == 0 && !frame_width.isWritten) {
		tok = strtok(NULL, " ,=");
		frame_width.value = strtof(tok, NULL);
		frame_width.isWritten = true;
	}
	if (strcmp(tok, "baud") == 0 && !baud.isWritten) {
		tok = strtok(NULL, " ,=");
		baud.value = strtod(tok, NULL);
		baud.isWritten = true;
	}
	if (strcmp(tok, "video_id") == 0 && !video_id.isWritten) {
		tok = strtok(NULL, " ,=");
		video_id.value = strtod(tok, NULL);
		video_id.isWritten = true;
	}
	if (strcmp(tok, "video_width") == 0 && !video_width.isWritten) {
		tok = strtok(NULL, " ,=");
		video_width.value = strtod(tok, NULL);
		video_width.isWritten = true;
	}
	if (strcmp(tok, "video_height") == 0 && !video_height.isWritten) {
		tok = strtok(NULL, " ,=");
		video_height.value = strtod(tok, NULL);
		video_height.isWritten = true;
	}
}

void input_params::setInputFile(std::string filename) {
	this->filename = filename;
	parseFile(this->filename);
}

// Might need more error outputs?
float input_params::getDiagonal() const {
	return diagonal.isWritten ? diagonal.value : 50.0;
}

float input_params::getAltitude() const {
	return altitude.isWritten ? altitude.value : -6.0;
}

float input_params::getTolerance() const {
	return tolerance.isWritten ? tolerance.value : 1.0;
}

float input_params::getFrame_Width() const {
	return frame_width.isWritten ? frame_width.value : 9.0;
}

int input_params::getBaud() const {
	return baud.isWritten ? baud.value : 57600;
}

int input_params::getVideo_ID() const {
	return video_id.isWritten ? video_id.value : 0;
}

int input_params::getVideo_Width() const {
	return video_width.isWritten ? video_width.value : 640;
}

int input_params::getVideo_Height() const {
	return video_height.isWritten ? video_height.value : 360;
}