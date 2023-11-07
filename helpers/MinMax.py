import sys
from numba import njit



@njit
def xMax(polygonCoordinates):
	l = len(polygonCoordinates)
	xMax = -sys.maxsize

	for i in range(l):
		if polygonCoordinates[i][0] > xMax:
			xMax = polygonCoordinates[i][0]

	return xMax

@njit
def yMax(polygonCoordinates):
	l = len(polygonCoordinates)
	yMax = -sys.maxsize

	for i in range(l):
		if polygonCoordinates[i][1] > yMax:
			yMax = polygonCoordinates[i][1]

	return yMax

@njit
def xMin(polygonCoordinates):
	l = len(polygonCoordinates)
	xMix = sys.maxsize

	for i in range(l):
		if polygonCoordinates[i][0] < xMix:
			xMix = polygonCoordinates[i][0]

	return xMix

@njit
def yMin(polygonCoordinates):
	l = len(polygonCoordinates)
	yMix = sys.maxsize

	for i in range(l):
		if polygonCoordinates[i][1] < yMix:
			yMix = polygonCoordinates[i][1]

	return yMix