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

#include "threshold.h"

#include <iostream>
#include <cmath>
#include <set>


#ifdef _MSC_VER 
#ifndef _INFINITY_numeric_limits
#define _INFINITY_numeric_limits
#include <limits>
float INFINITY = std::numeric_limits<float>::infinity();
#endif
#endif

double ThresholdAlgorithm::getHighest()
{
	double hVal = 0;
	for ( int row = 0 ; row < matrix.rows() ; row++ ){
		for ( int col = 0 ; col < matrix.columns() ; col++ ){
			if (matrix(row,col) > hVal )
			{
				hVal = matrix(row,col);
			}
		}
	}
	return hVal;
}
double ThresholdAlgorithm::getLowest()
{
	double lVal = INFINITY;
	for ( int row = 0 ; row < matrix.rows() ; row++ ){
		for ( int col = 0 ; col < matrix.columns() ; col++ ){
			if (matrix(row,col) < lVal  )
			{
				lVal = matrix(row,col);
			}
		}
	}
	return lVal;
}
double ThresholdAlgorithm::getMedianWithThreshold(double lowThreshold = 0,double highThreshold = 0)
{
	//std::vector<double> values;
	std::multiset<double> values;
	int rows = matrix.rows();
	int cols = matrix.columns();

	for ( int row = 0 ; row < rows ; row++ )
		for ( int col = 0 ; col < cols ; col++ )
			if (matrix(row,col) <= highThreshold && matrix(row,col) >= lowThreshold)
			{
				values.insert(matrix(row,col));
			}
			//if(values.size() == 0) return lowThreshold;
			std::multiset<double>::iterator it = values.begin();
			std::advance(it,(values.size()-1)/2);
			if(values.size() % 2 == 0)
			{
				double mid = *(it++);
				double next = *it; 
				return (mid + next)/2;
			}
			else return *it;
}
void ThresholdAlgorithm::printMatrix()
{
	int nrows = matrix.rows();
	int ncols = matrix.columns();
	for ( int row = 0 ; row < nrows ; row++ ) {
		for ( int col = 0 ; col < ncols ; col++ ) {
			std::cout.width(2);
			std::cout << matrix(row,col) << ",";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
}

bool ThresholdAlgorithm::findBipartianSolution(std::vector<int>& rowCol,int row)
{
	for ( int col = 0 ; col < matrix.columns() ; col++ ) {
		
		if(matrix(row,col) == 0 && !col_mask[col])
		{	
			rowCol[row] = col;
			if(row == matrix.rows()-1){
				return true;
			}
			else{
				col_mask[col] = true;
				if(findBipartianSolution(rowCol,row+1)){
					return true;
				}
				else
				{
					rowCol[row] = -1;
					col_mask[col] = false;
				}
			}
		}
	}
	return false;
}
bool ThresholdAlgorithm::findBipartianSolution(int row)
{
	for ( int col = 0 ; col < matrix.columns() ; col++ ) {
		if(matrix(row,col) == 0 && !col_mask[col])
		{	
			if(row == matrix.rows()-1){
				return true;
			}
			else{
				col_mask[col] = true;
				if(findBipartianSolution(row+1)){
					return true;
				}
				else
				{
					col_mask[col] = false;
				}
			}
		}
	}
	return false;
}
void ThresholdAlgorithm::setMatixZerosBelowThreshold(double threshold)
{
	for ( int row = 0 ; row < matrix.rows() ; row++ ){
		for ( int col = 0 ; col < matrix.columns() ; col++ ){
			if(matrix(row,col) <= threshold)
				matrix(row,col) = 0;
		}
	}
}
void ThresholdAlgorithm::setMatixINFINITYAboveThreshold(double threshold, Matrix<double>& m)
{
	for ( int row = 0 ; row < m.rows() ; row++ ){
		for ( int col = 0 ; col < m.columns() ; col++ ){
			if(m(row,col) > threshold)
				m(row,col) = 99999;
		}
	}
}

std::vector<int> ThresholdAlgorithm::solve(Matrix<double> &m) {
	this->matrix = m;
	assert(this->matrix.rows() == this->matrix.columns());
	col_mask = std::vector<bool>(matrix.columns(),false);
	//row_mask = std::vector<bool>(matrix.rows(),false);

	this->costH = getHighest();
	std::cout << "High val:" << this->costH << std::endl;
	this->costL = getLowest();
	std::cout << "Low val:" << this->costL << std::endl;
	//double median = getMedianWithThreshold(costL,costH);
	//std::cout << "Median:" << median << std::endl;
	//double median2 = getMedianWithThreshold(7,costH);
	//std::cout << "Median:" << median2 << std::endl;
	//double median3 = getMedianWithThreshold(costL,1);
	//std::cout << "Median:" << median3 << std::endl;
	//printMatrix();

	std::vector<int> rowCol(matrix.rows(),-1);
	//if(findBipartianSolution(rowCol))
	//	std::cout << "success" << std::endl;
	//else std::cout << "failed" << std::endl;

	//for ( int row = 0 ; row < matrix.rows() ; row++ ) {
	//	std::cout.width(2);
	//	std::cout << rowCol[row] << ",";
	//}
	//std::cout << std::endl;

	//Matrix tempMatrix = m;

	double cost = costH;
	bool lowBound = true;
	double median;
	double prevThreshold = -1; 
	while(costL < cost || cost < costH) //This implementation is kind of messed up but it works good enough.
	{

		setMatixZerosBelowThreshold(costL);
		if(lowBound) {
			median = getMedianWithThreshold(costL,cost);
		} else { 
			median = getMedianWithThreshold(cost,costH);
		}

		cost = median;
		setMatixZerosBelowThreshold(cost);
		//printMatrix();

		
		if(prevThreshold == cost) {
			break;
		}
		rowCol = std::vector<int>(matrix.rows(),-1);
		col_mask = std::vector<bool>(matrix.columns(),false);
		if(findBipartianSolution(rowCol))
		{
			costH = median;
			matrix = m;
			lowBound = true;
		}
		else
		{
			costL = median;
			lowBound = false;
		}
		prevThreshold = cost;
	}
	//setMatixZerosBelowThreshold(cost);
	//std::cout << "Final" << std::endl;
	//printMatrix();
	setMatixINFINITYAboveThreshold(cost,m);
	//std::cout << counter++ <<",L:" << costL <<",H:" << costH << std::endl;
	return rowCol;
}
