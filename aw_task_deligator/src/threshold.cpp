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
			if(values.size() == 0) return lowThreshold;
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
void ThresholdAlgorithm::printMatrix(Matrix<double> &tempMatrix)
{
	int nrows = tempMatrix.rows();
	int ncols = tempMatrix.columns();
	for ( int row = 0 ; row < nrows ; row++ ) {
		for ( int col = 0 ; col < ncols ; col++ ) {
			std::cout.width(3);
			std::cout << tempMatrix(row,col) << ",";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
}

bool ThresholdAlgorithm::findBipartianSolution(Matrix<double>& tempMatrix,std::vector<int>& rowCol)
{
	//Yes, this is slow for large n, but there exists ways solve this quickly.
	std::vector<std::pair<int,int> > rowOrder;
	col_mask = std::vector<bool>(tempMatrix.columns(),false);
	for ( int row = 0 ; row < tempMatrix.rows() ; row++ ) {
		int zeroCols = 0;
		for ( int col = 0 ; col < tempMatrix.columns() ; col++ ) {
			if(tempMatrix(row,col) == 0)
			{
				zeroCols++;
			}
		}
		if(zeroCols==0)
		{
			return false;
		}
		else
		{
			rowOrder.push_back(std::make_pair(zeroCols,row));
		}
	}
	std::sort(rowOrder.begin(),rowOrder.end());

	return findPairing(tempMatrix,rowOrder,rowCol);
}

bool ThresholdAlgorithm::findPairing(Matrix<double>& tempMatrix,std::vector<std::pair<int,int> >& rowOrder,std::vector<int>& rowCol,int row)
{
	int matrixRow = rowOrder[row].second;
	for ( int col = 0 ; col < tempMatrix.columns() ; col++ ) {
		
		if(tempMatrix(matrixRow,col) == 0 && !col_mask[col])
		{	
			rowCol[matrixRow] = col;
			if(row == tempMatrix.rows()-1){
				return true;
			}
			else{
				col_mask[col] = true;
				if(findPairing(tempMatrix,rowOrder,rowCol,row+1)){
					return true;
				}
				else
				{
					rowCol[matrixRow] = -1;
					col_mask[col] = false;
				}
			}
		}
	}
	return false;
}
//bool ThresholdAlgorithm::findBipartianSolution(int row)
//{
//	for ( int col = 0 ; col < matrix.columns() ; col++ ) {
//		if(matrix(row,col) == 0 && !col_mask[col])
//		{	
//			if(row == matrix.rows()-1){
//				return true;
//			}
//			else{
//				col_mask[col] = true;
//				if(findBipartianSolution(row+1)){
//					return true;
//				}
//				else
//				{
//					col_mask[col] = false;
//				}
//			}
//		}
//	}
//	return false;
//}
void ThresholdAlgorithm::setMatixZerosBelowThreshold(Matrix<double>& tempMatrix, double threshold)
{
	for ( int row = 0 ; row < tempMatrix.rows() ; row++ ){
		for ( int col = 0 ; col < tempMatrix.columns() ; col++ ){
			if(tempMatrix(row,col) <= threshold)
				tempMatrix(row,col) = 0;
		}
	}
}
void ThresholdAlgorithm::setMatixINFINITYAboveThreshold(double threshold, Matrix<double>& m)
{
	for ( int row = 0 ; row < m.rows() ; row++ ){
		for ( int col = 0 ; col < m.columns() ; col++ ){
			if(m(row,col) > threshold)
				m(row,col) = INFINITY;
		}
	}
}

std::vector<int> ThresholdAlgorithm::solve(Matrix<double> &m) {
	this->matrix = m;
	assert(this->matrix.rows() == this->matrix.columns());
	col_mask = std::vector<bool>(matrix.columns(),false);

	this->costH = getHighest();
	this->costL = getLowest();

	std::vector<int> rowCol(matrix.rows(),-1);

	double prevMedian = -1;
	double median;

	while(costL < costH)
	{
		median = getMedianWithThreshold(costL,costH);
		Matrix<double> tempMatrix = this->matrix;
		
		if(median == prevMedian)
		{
			setMatixZerosBelowThreshold(tempMatrix,costH-1);
			if(findBipartianSolution(tempMatrix,rowCol))
			{
				//printf("Here???\n");
				costH = costH-1;
				continue;
			}
			else
				break;
		}
		//printf("costL:%f, costH %f,median %f\n",costL,costH,median);
		setMatixZerosBelowThreshold(tempMatrix,median);
		//printMatrix(tempMatrix);
		if(findBipartianSolution(tempMatrix,rowCol))
		{
			costH = median;
		}
		else 
		{
			costL = median;
		}
		
		prevMedian = median;
	}
	
	setMatixZerosBelowThreshold(matrix,costH);
	//printMatrix(matrix);

	findBipartianSolution(matrix,rowCol);
	setMatixINFINITYAboveThreshold(costH,m);
	//std::cout << counter++ <<",L:" << costL <<",H:" << costH << std::endl;
	return rowCol;
}
