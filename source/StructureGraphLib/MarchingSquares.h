#pragma once

#include <vector>
#include <Eigen/Core>

/* Adapted from https://github.com/sakri/MarchingSquaresJS */
struct MarchingSquares
{
	typedef Eigen::Vector2d PixelType;
	enum Direction{ NONE, UP, LEFT, DOWN, RIGHT };

	Eigen::MatrixXd image;
	double fillValue;
	int width, height;
	Direction previousStep, nextStep;

	MarchingSquares( const Eigen::MatrixXd & image, double fillValue ) : image(image), fillValue(fillValue)
	{
		width = image.cols();
		height = image.rows();
	}

	std::vector<PixelType> march()
	{
		return walkPerimeter( getFirstBorderPixelTopDown() );
	}

	static std::vector<PixelType> march( Eigen::MatrixXd image, double fillValue ){
		MarchingSquares mc(image, fillValue);
		return mc.march();
	}

	PixelType getFirstBorderPixelTopDown()
	{
		PixelType pixel(-1,-1);

		for(int y = 0; y < height; y++){
			for(int x = 0; x < width; x++){
				if(image(y,x) == fillValue)
					return PixelType(x,y);
			}
		}

		return pixel;
	}

	std::vector<PixelType> walkPerimeter(PixelType startingPoint)
	{
		int startX = startingPoint.x();
		int startY = startingPoint.y();

		// Set up our return list
		std::vector<PixelType> pointList;

		if(startX < 0 || startY < 0) return pointList;
		
		// Our current x and y positions, initialized
		// to the init values passed in
		int x = startX;
		int y = startY;

		// The main while loop, continues stepping until
		// we return to our initial points
		do{
			// Evaluate our state, and set up our next direction
			step( PixelType(x - 1, y - 1) );

			// If our current point is within our image add it to the list of points
			if (x >= 0 && x < width && y >= 0 && y < height)
				pointList.push_back( PixelType(x - 1, y - 1) );

			switch ( nextStep )
			{
				case UP:    y--; break;
				case LEFT:  x--; break;
				case DOWN:  y++; break;
				case RIGHT: x++; break;
			}

		} while (x != startX || y != startY);

		pointList.push_back( PixelType(x - 1, y - 1) );

		return pointList;
	}

	inline bool emptyPixel( int x, int y )
	{
		if(x < 0 || x > width - 1 || y < 0 || y > height - 1) return true;
		return image(y,x) != fillValue;
	}

	void step( PixelType pixel )
	{
		bool upLeft = emptyPixel( pixel.x(), pixel.y() );
		bool upRight = emptyPixel( pixel.x()+1, pixel.y() );
		bool downLeft = emptyPixel( pixel.x(), pixel.y()+1 );
		bool downRight = emptyPixel( pixel.x()+1, pixel.y()+1 );

		// Store our previous step
		previousStep = nextStep;

		// Determine which state we are in
		int state = 0;
		if (upLeft)		state |= 1;
		if (upRight)	state |= 2;
		if (downLeft)	state |= 4;
		if (downRight)	state |= 8;

		// So we can use a switch statement to determine our
		// next direction based on
		switch (state){
		case 1: nextStep = UP; break;
		case 2: nextStep = RIGHT; break;
		case 3: nextStep = RIGHT; break;
		case 4: nextStep = LEFT; break;
		case 5: nextStep = UP; break;
		case 6:
			if (previousStep == UP){
				nextStep = LEFT;
			}else{
				nextStep = RIGHT;
			}
			break;
		case 7: nextStep = RIGHT; break;
		case 8: nextStep = DOWN; break;
		case 9:
			if (previousStep == RIGHT){
				nextStep = UP;
			}else{
				nextStep = DOWN;
			}
			break;
		case 10: nextStep = DOWN; break;
		case 11: nextStep = DOWN; break;
		case 12: nextStep = LEFT; break;
		case 13: nextStep = UP; break;
		case 14: nextStep = LEFT; break;
		default:
			nextStep = NONE;//this should never happen
			break;
		}
	}
};
