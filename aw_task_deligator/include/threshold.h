/*
 *   Copyright (c) 2007 John Weaver
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#if !defined(_AW_THRESHOLD_H_)
#define _AW_THRESHOLD_H_

#include "matrix.h"

#include <utility>
#include <vector>
class ThresholdAlgorithm {
public:
    std::vector<int> solve(Matrix<double> &m);
private:
    Matrix<double> matrix;
    std::vector<bool> col_mask;

	double costH;
	double costL;
	double getHighest();
	double getLowest();
	double getMedianWithThreshold(double lowThreshold,double highThreshold);
	void printMatrix(Matrix<double> &tempMatrix);
	bool findBipartianSolution(Matrix<double>& tempMatrix,std::vector<int>& rowCol);
	bool findPairing(Matrix<double>& tempMatrix,std::vector<std::pair<int,int> >& rowOrder, std::vector<int>& rowCol,int row = 0);
	//bool findBipartianSolution(int row = 0);
	void setMatixZerosBelowThreshold(Matrix<double>& tempMatrix, double threshold);
	void setMatixINFINITYAboveThreshold(double threshold,Matrix<double>& m);


};

#endif 
