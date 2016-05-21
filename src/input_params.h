/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   input_params.h
 * Author: karthik
 *
 * Created on May 19, 2016, 2:52 PM
 */

#ifndef INPUT_PARAMS_H
#define INPUT_PARAMS_H

#include <iostream>
#include <string>

class input_params {
public:
	input_params();
	input_params(std::string filename);
	float getDiagonal() const;
	float getAltitude() const;
	float getTolerance() const;
	float getFrame_Width() const;
	int getBaud() const;
	int getVideo_ID() const;
	int getVideo_Width() const;
	int getVideo_Height() const;
	void setInputFile(std::string filename);
	virtual ~input_params();
private:
	template<typename T>
	class Item {
	public:
		T value;
		bool isWritten;
	};
	Item<float> altitude;
	Item<float> diagonal;
	Item<float> tolerance;
	Item<float> frame_width; 
	Item<int> video_id;
	Item<int> video_width;
	Item<int> video_height;
	Item<int> baud;
	std::string filename;
	void parseFile(std::string fname);
	void parseLine(std::string line);
};

// <TODO: Implement file write out if no input parameter file is set>

#endif /* INPUT_PARAMS_H */

