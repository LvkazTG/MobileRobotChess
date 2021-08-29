#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h" 
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <iostream>
#include <math.h>
#include "fuzzylite.h"
#include <map>
#include <string>
#include <fstream>

using namespace std;

std::map<std::string, std::string> chess_pieces{{"W_pawn_1", "a2"},
													   {"W_pawn_2", "b2"},
													   {"W_pawn_3", "c2"},
													   {"W_pawn_4", "d2"},
													   {"W_pawn_5", "e2"},
													   {"W_pawn_6", "f2"},
													   {"W_pawn_7", "g2"},
													   {"W_pawn_8", "h2"},
													   {"W_rook_1", "a1"},
													   {"W_rook_2", "h1"},
													   {"W_knight_1", "b1"},
													   {"W_knight_2", "g1"},
													   {"W_bishop_1", "c1"},
													   {"W_bishop_2", "f1"},
													   {"W_queen", "d1"},
													   {"W_king", "e1"},
													   {"B_pawn_1", "a7"},
													   {"B_pawn_2", "b7"},
													   {"B_pawn_3", "c7"},
													   {"B_pawn_4", "d7"},
													   {"B_pawn_5", "e7"},
													   {"B_pawn_6", "f7"},
													   {"B_pawn_7", "g7"},
													   {"B_pawn_8", "h7"},
													   {"B_rook_1", "a8"},
													   {"B_rook_2", "h8"},
													   {"B_knight_1", "b8"},
													   {"B_knight_2", "g8"},
													   {"B_bishop_1", "c8"},
													   {"B_bishop_2", "f8"},
													   {"B_queen", "d8"},
													   {"B_king", "e8"}};

std::map<std::string, int> chessHouseLetterValue{{"a", -7}, {"b", -5}, {"c", -3}, {"d", -1}, {"e", 1}, {"f", 3}, {"g", 5}, {"h", 7}};
std::map<std::string, int> chessHouseNumValue{{"1", 7}, {"2", 5}, {"3", 3}, {"4", 1}, {"5", -1}, {"6", -3}, {"7", -5}, {"8", -7}};

// -10 should be a border, but goes a little less so robot wont fall but piece will
// This pos will be used for "X" pos in captured pieces
std::array<float, 2> piecesCapturedXPos{{0, 10-0.05}};

double x{99};
double y{99};
double ang{0};

bool waitLoadOdom{true};

double distObsLeft{99};
double distObsFront{99};
double distObsRight{99};

bool waitLoadDistObs{true};

geometry_msgs::Twist msg;

bool posOk{false};
float desPos[2];
float errorPos{99};
float errorAng{99};

float tolerancePos{0.03};
float tolerancePosBacking{0.2};
float toleranceAng{0.05};

bool useObsAvoidance{false};

double DIST_OBS_AVOID_FAR{0.9};
double DIST_OBS_AVOID_CLOSE{0.8};
double DIST_OBS_AVOID_VERY_CLOSE{0.4};

double distObsAvoid{DIST_OBS_AVOID_FAR};

double lidar_ignorar_percentage = 0.3;
double lidar_laterais_percentage = 0.5;

std::list<std::string> moveSequence{};

void readPosFromFile(const std::string& fileName)
{
	std::ifstream file(fileName);
	if (file.is_open())
	{
		
		std::string line;
		while (std::getline(file, line))
		{
			if(line.size() >= 4)
			{
				moveSequence.push_back(line.substr(0,4));
			}
		}

		file.close();
	}
}

void updatePiecesPositions(const std::string& pos, bool& pieceCaptured)
{
	if(pos.size() == 4)
	{
		std::string foundStartPieceMove{};
		std::string foundEndPieceMove{};

		for(const auto& piece : chess_pieces)
		{
			if(foundStartPieceMove.empty() && (pos.substr(0,2) == piece.second))
			{
				foundStartPieceMove = piece.first;
			}
			else if(foundEndPieceMove.empty() && (pos.substr(2,2) == piece.second))
			{
				// Last condition is used in case of foundStartPieceMove first
				foundEndPieceMove = piece.first;
				pieceCaptured = true;
			}

			if(!foundStartPieceMove.empty() && !foundEndPieceMove.empty())
			{
				// Found both, dont need to iter in rest of things
				break;
			}
		}

		if(!foundStartPieceMove.empty())
		{
			chess_pieces[foundStartPieceMove] = pos.substr(2,2);
		}
		else
		{
			// ERROR 
			printf("Piece to move was not detected, move %s, ending program\n", pos.c_str());
			exit(-2);
		}

		// Captured piece may or may not happen
		if(!foundEndPieceMove.empty())
		{
			chess_pieces[foundEndPieceMove] = "X";
		}

	}
	else
	{
		// ERROR 
		printf("Error checking new piece positions, ending program\n");
		exit(-2);
	}

}

bool isKnight(const std::string& pos)
{
	bool isKnightInPos{false};

	for(const auto& piece : chess_pieces)
	{
		if(pos == piece.second)
		{
			if(std::string::npos != piece.first.find("knight"))
			{
				isKnightInPos = true;
			}

			// Only 1 piece can occupy a square per time
			break;
		}
	}

	return isKnightInPos;
}

std::list<std::tuple<std::array<float, 2>, bool, bool>> createMovesRemoveCapturedPiece(const std::string& capturedSquare)
{
	std::list<std::tuple<std::array<float, 2>, bool, bool>> steps{};

	if(capturedSquare.size() == 2)
	{
		const std::array<float, 2> startPos{static_cast<float>(chessHouseNumValue.at(std::string{capturedSquare.at(1)})),
					                        static_cast<float>(chessHouseLetterValue.at(std::string{capturedSquare.at(0)}))};
		const std::array<float, 2> endPos{piecesCapturedXPos};

		const std::array<float, 2> moveNeeded{endPos.at(0) - startPos.at(0), endPos.at(1) - startPos.at(1)};


		float moveDiffX{0};
		float moveDiffY{0};
		if(moveNeeded.at(0) != 0)
		{
			moveDiffX = (moveNeeded.at(0)/std::abs(moveNeeded.at(0)));
			
		}

		if(moveNeeded.at(1) != 0)
		{
			moveDiffY = (moveNeeded.at(1)/std::abs(moveNeeded.at(1)));			
		}

		const float thing_size_half{0.48/2}; // To reference later


		steps.push_back({{startPos.at(0), startPos.at(1) - moveDiffY}, false, false}); // Position "left" line of the piece
		steps.push_back({{startPos.at(0), startPos.at(1) + moveDiffY - moveDiffY*thing_size_half}, true, false}); // Position and move piece to "right" line
		steps.push_back({{startPos.at(0), startPos.at(1)}, false, true}); // Move back to center of that square

		steps.push_back({{startPos.at(0) - moveDiffX, startPos.at(1) + moveDiffY}, false, false}); // Position line cross "bellow" "right" of the piece
		steps.push_back({{endPos.at(0) - moveDiffX*thing_size_half , startPos.at(1) + moveDiffY}, true, false}); // Position and move piece to the line cross "above" "right"
		steps.push_back({{endPos.at(0) - moveDiffX, startPos.at(1) + moveDiffY}, false, true}); // Move back in line "right", "bellow" piece

		steps.push_back({{endPos.at(0), startPos.at(1) + moveDiffY - moveDiffY}, false, false}); // Position "left" of the piece
		steps.push_back({{endPos.at(0), endPos.at(1) - moveDiffY*thing_size_half + moveDiffY*0.05f}, true, false}); // Position and move piece to destination (with a little extra to guarantee piece push off board)
		steps.push_back({{endPos.at(0), endPos.at(1) - moveDiffY}, false, true}); // Move back to "left" of piece
	}

	return steps;
}

std::list<std::tuple<std::array<float, 2>, bool, bool>> readStartEndCoords(const std::string& uciNotation)
{
	// Coords, if orient needs to be corrected first, if move is backward
	std::list<std::tuple<std::array<float, 2>, bool, bool>> steps{};

	float thing_size_half{0.48/2}; // To reference later

	std::array<float, 2> startPos{};
	std::array<float, 2> endPos{};
	if(uciNotation.size() == 4)
	{
		startPos = {static_cast<float>(chessHouseNumValue.at(std::string{uciNotation.at(1)})),
					static_cast<float>(chessHouseLetterValue.at(std::string{uciNotation.at(0)}))};
		endPos = {static_cast<float>(chessHouseNumValue.at(std::string{uciNotation.at(3)})),
				  static_cast<float>(chessHouseLetterValue.at(std::string{uciNotation.at(2)}))};

		std::array<float, 2> moveNeeded{endPos.at(0) - startPos.at(0), endPos.at(1) - startPos.at(1)};

		float moveDiffX{0};
		float moveDiffY{0};
		if(moveNeeded.at(0) != 0)
		{
			moveDiffX = (moveNeeded.at(0)/std::abs(moveNeeded.at(0)));
			
		}

		if(moveNeeded.at(1) != 0)
		{
			moveDiffY = (moveNeeded.at(1)/std::abs(moveNeeded.at(1)));			
		}

		if((moveNeeded.at(0) != 0) || (moveNeeded.at(1) != 0))
		{
			if(isKnight(uciNotation.substr(0,2)))
			{
				// Need to check the move double
				bool moveDoubledInY{false};
				// Double move in Y (so 2), plus 2 because board is doubled in size
				if(static_cast<uint>(std::fabs(moveNeeded.at(1))) == 4)
				{
					moveDoubledInY = true;
					std::swap(startPos[0], startPos[1]);
					std::swap(endPos[0], endPos[1]);
					std::swap(moveNeeded[0], moveNeeded[1]);
					std::swap(moveDiffX, moveDiffY);
				}

				steps.push_back({{startPos.at(0), startPos.at(1) - moveDiffY}, false, false}); // Position "left" line of the piece
				steps.push_back({{startPos.at(0), startPos.at(1) + moveDiffY - moveDiffY*thing_size_half}, true, false}); // Position and move piece to "right" line
				steps.push_back({{startPos.at(0), startPos.at(1)}, false, true}); // Move back to center of that square

				steps.push_back({{startPos.at(0) - moveDiffX, startPos.at(1) + moveDiffY}, false, false}); // Position line cross "bellow" "right" of the piece
				// Try to move a little faster to end point, last move needs to have just 1 desloc house to succeed
				// using 0.4 to adjust dist better, "corect would be 0.5", maybe it was needed because of adjust for piece at end
				steps.push_back({{endPos.at(0) - moveDiffX*thing_size_half + moveDiffX*0.4f , startPos.at(1) + moveDiffY}, true, false}); // Position and move piece to the line cross "above" "right"
				steps.push_back({{endPos.at(0) - moveDiffX, startPos.at(1) + moveDiffY}, false, true}); // Move back in line cross "right", "bellow" piece 

				steps.push_back({{endPos.at(0) + moveDiffX, startPos.at(1) + moveDiffY - moveDiffY}, false, false}); // Position "half above" "left" of the piece

				// Need to check this dist with trigonometry to push correcy thing percentage for each side
				steps.push_back({{static_cast<float>(endPos.at(0) + moveDiffX*std::pow((thing_size_half/5), 0.5)),
								  static_cast<float>(endPos.at(1) - moveDiffY*2*std::pow((thing_size_half/5), 0.5))}, true, false}); // Position and move piece to destination 
				steps.push_back({{endPos.at(0) + moveDiffX*0.5f , startPos.at(1) + moveDiffY}, false, true}); // Move back to "half above" "left" of piece

				if(moveDoubledInY)
				{
					// Swap steps
					for(auto& step : steps)
					{
						std::swap(std::get<0>(step)[0], std::get<0>(step)[1]);						
					}
				}

			}
			else
			{
				steps.push_back({{startPos.at(0) - moveDiffX, startPos.at(1) - moveDiffY}, false, false});

				if(static_cast<int>(moveDiffX) == 0 || static_cast<int>(moveDiffX))
				{
					steps.push_back({{endPos.at(0) - moveDiffX*thing_size_half, endPos.at(1) - moveDiffY*thing_size_half}, true, false}); // before move, need to orientate ang
				}
				else
				{
					// Move in both directions, need to correcy a little the thing_size_half multiplication
					float heighted_thing_size_half{thing_size_half/(2^(1/2))};
					steps.push_back({{endPos.at(0) - moveDiffX*heighted_thing_size_half, endPos.at(1) - moveDiffY*heighted_thing_size_half}, true, false}); // before move, need to orientate ang	
				}
				steps.push_back({{endPos.at(0) - moveDiffX, endPos.at(1)  - moveDiffY}, false, true}); // move backward
			}
		}
		else
		{
			// ERROR 
			printf("Error decoding move, ending program\n");
			exit(-1);
		}

		// Verify and adjust pieces in board, including capture
		bool capturedPiece{false};
		updatePiecesPositions(uciNotation, capturedPiece);

		if(capturedPiece)
		{
			std::list<std::tuple<std::array<float, 2>, bool, bool>> stepsCaptureFirst{createMovesRemoveCapturedPiece(uciNotation.substr(2,2))};

			// Captured pieces shall go first
			for(const auto& normalStep : steps)
			{
				stepsCaptureFirst.push_back(normalStep);
			}

			// Update steps variable to have all moves
			steps = stepsCaptureFirst;
		}
	}

	return steps;
}


void subCallback_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
    tf::Pose pose{};

    tf::poseMsgToTF(msg->pose.pose, pose);
    ang = tf::getYaw(pose.getRotation());

	// Some problems because of bad msg mounting in simulation
    x = msg->pose.pose.position.y;
    y = msg->pose.pose.position.z;

	waitLoadOdom = false;
}

void subCallback_lidar(const sensor_msgs::LaserScan::ConstPtr& lidar)
{	
	const int lidar_size_received{lidar->ranges.size()};
	const int lidar_size{floor(lidar_size_received*(1-lidar_ignorar_percentage))};

	const int size_1{floor((lidar_size_received - lidar_size)/2)};
	const int size_2{floor(lidar_size * lidar_laterais_percentage)/2};
	const int size_4{size_2};
	const int size_5{size_1};
	const int size_3{lidar_size_received - size_1 - size_2 - size_4 - size_5};

	distObsRight = 1.5;
	int iterCounter{size_1};
	for( ; iterCounter < (size_1 + size_2); iterCounter++)
	{
		if(lidar->ranges[iterCounter] < distObsRight && lidar->ranges[iterCounter] > 0.01)
		{
			distObsRight = lidar->ranges[iterCounter];
		}
	}
	
	distObsFront = 1.5;
	for( ; iterCounter < (size_1 + size_2 + size_3); iterCounter++)
	{
		if(lidar->ranges[iterCounter] < distObsFront && lidar->ranges[iterCounter] > 0.01)
		{
			distObsFront = lidar->ranges[iterCounter];
		}
	}

	distObsLeft = 1.5;
	for( ; iterCounter < (size_1 + size_2 + size_3 + size_4); iterCounter++)
	{
		if(lidar->ranges[iterCounter] < distObsLeft && lidar->ranges[iterCounter] > 0.01)
		{
			distObsLeft = lidar->ranges[iterCounter];
		}
	}

	FuzzyliteData::setObstacleDists(distObsLeft, distObsFront, distObsRight);
	
//	distObsAvoid = errorPos < DIST_OBS_AVOID_CLOSE ? DIST_OBS_AVOID_CLOSE : DIST_OBS_AVOID_FAR;
//	distObsAvoid = errorPos < DIST_OBS_AVOID_VERY_CLOSE ? DIST_OBS_AVOID_VERY_CLOSE : distObsAvoid;
	
	
	double smallestDistObs{distObsLeft < distObsFront ? distObsLeft : distObsFront};
	smallestDistObs = (smallestDistObs < distObsRight ? smallestDistObs : distObsRight);
	// Check if any "sensor" was so near that it should trigger obs avoidance

	useObsAvoidance = ((smallestDistObs < DIST_OBS_AVOID_CLOSE) && ((errorPos*2 + 0.1) >  smallestDistObs));
//	useObsAvoidance = ((distObsLeft < distObsAvoid) || (distObsFront < distObsAvoid) ||
//					   (distObsRight < distObsAvoid));

	waitLoadDistObs = false;
}

float calcErrorAng(const float curAng)
{
	float desAng{atan2(desPos[1]-y, desPos[0]-x)};
	float errorAng{desAng-curAng};

	// Adding 0.01 just to be sure
	if(std::abs(errorAng) > M_PI + 0.01)
	{
		// desAng gives values between pi and -pi
		if(ang >= 0)
		{
			desAng += 2*M_PI;
		}
		else
		{
			desAng -= 2*M_PI; 
		}

		// Recalculate
		errorAng = desAng-ang;
	}

	// Limit errorAng, betwwen -M_PI and M_PI, but it should not be necessary
	if(errorAng > M_PI)
	{
		errorAng = M_PI;
	}
	else if(errorAng < -M_PI)
	{
		errorAng = -M_PI;
	}
	else
	{
		// No need to change
	}

	return errorAng;
}

int main(int argc, char **argv)
{
	if(argc < 2)
	{
		printf("Bad use of program, need to pass file with moves in uci\r\n");
		exit(-1);
	}

	ros::init(argc, argv, "WaiterBob_Fuzzy_2");

	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Subscriber sub = n.subscribe("/odom", 1000, subCallback_odom);
	ros::Subscriber sub_lidar = n.subscribe("/scan", 1000, subCallback_lidar);

	ros::Rate loop_rate(10);

	FuzzyliteData::loadFuzzyData();

	if (ros::ok())
	{		
		ros::spinOnce();

		readPosFromFile(std::string{argv[1]});

		std::list<std::tuple<std::array<float, 2>, bool, bool>> totalSteps{};
		for(const auto& move : moveSequence)
		{
			std::cout << "Process move " << move << std::endl;
			const std::list<std::tuple<std::array<float, 2>, bool, bool>> steps{readStartEndCoords(move)};
			for(const auto& step : steps)
			{
				totalSteps.push_back(step);
			}
		}

		// For debug, list moves
		{
			std::cout << "Total steps num: " <<totalSteps.size() << std::endl;
			int moveCount{0};
			for(const auto& step : totalSteps)
			{
				std::cout << "Move: " << moveCount << " Pos " << std::get<0>(step).at(0) << " "
						  << std::get<0>(step).at(1) << " Special " << std::get<1>(step)
						  << " Back " << std::get<2>(step) << std::endl;
				moveCount++;
			}
		}

		while(!totalSteps.empty())
		{
			desPos[0] = std::get<0>(totalSteps.front()).at(0);
			desPos[1] = std::get<0>(totalSteps.front()).at(1);
			bool specialMove{std::get<1>(totalSteps.front())};
			bool moveBack{std::get<2>(totalSteps.front())};
			
			totalSteps.pop_front();

			posOk = false;
			bool angOk = false;

			waitLoadOdom = true;
			waitLoadDistObs = true;


			while(true)
			{
				if(waitLoadDistObs || waitLoadOdom)
				{
					msg.linear.x = 0;
					msg.angular.z = 0;
					pub.publish(msg);
					ros::spinOnce();

					loop_rate.sleep();					
					continue;
				}

				errorPos =  sqrt(pow(desPos[0]-x,2)+pow(desPos[1]-y,2));
				float desAng{atan2(desPos[1]-y, desPos[0]-x)};
				float errorAng{calcErrorAng(ang)};

				FuzzyliteData::setErrorValues(errorPos, errorAng);

				if(specialMove && !angOk)
				{
					if(std::fabs(errorAng) < toleranceAng)
					{
						angOk = true;
						waitLoadOdom = true;
						waitLoadDistObs = true;
						continue;
					}

					msg.linear.x = 0;
					msg.angular.z = FuzzyliteData::getAngSpeed();

					system("clear");
					printf("Controle Ativo: Ajuste Angular.\n");
					printf("Des pos X %.1f Y %.1f \n", desPos[0], desPos[1]);
					printf("Erro de orientação: %.2f rad\n",errorAng);
					printf("Erro de posição: %.2f m\n",errorPos);
					printf("Velocidade angular enviada: %.2f rad/s\n",msg.angular.z);
					printf("Velocidade linear enviada: %.2f m/s\n\n",msg.linear.x);


					pub.publish(msg);
					ros::spinOnce();
					loop_rate.sleep();

					continue;
				}
				else if(moveBack)
				{
					// Check coord first
					if(errorPos < tolerancePosBacking)
					{
						posOk = true;
						break;
					}

					// Uses normal navigation, but reverse it
					float newAngReverse{ang + M_PI};
					errorAng = calcErrorAng(newAngReverse);

					FuzzyliteData::setErrorValues(errorPos, errorAng);

					msg.linear.x = -FuzzyliteData::getLinSpeed();
//					msg.angular.z = FuzzyliteData::getAngSpeedBackward();
					msg.angular.z = 0;

					system("clear");
					printf("Controle Ativo: RE.\n");
					printf("Des pos X %.1f Y %.1f \n", desPos[0], desPos[1]);					
					printf("Erro de orientação: %.2f rad\n",errorAng);
					printf("Erro de posição: %.2f m\n",errorPos);
					printf("Velocidade angular enviada: %.2f rad/s\n",msg.angular.z);
					printf("Velocidade linear enviada: %.2f m/s\n\n",msg.linear.x);

					pub.publish(msg);
					ros::spinOnce();
					loop_rate.sleep();

					continue;
				}
				else if (errorPos < tolerancePos)
				{
					posOk = true;
					break;
				}

				// Check if its necessary to use obs avoidance
				if(useObsAvoidance && !specialMove)
				{
					msg.linear.x = FuzzyliteData::getLinSpeedAvoidObs();
					msg.angular.z = FuzzyliteData::getAngSpeedAvoidObs();

					system("clear");
					printf("Controle Ativo: DESVIO DE OBSTACULO.\n");
					printf("Des pos X %.1f Y %.1f \n", desPos[0], desPos[1]);
					printf("Erro de orientação: -\n");
					printf("Erro de posição: %.2f m\n",errorPos);
					printf("Velocidade angular enviada: %.2f rad/s\n",msg.angular.z);
					printf("Velocidade linear enviada: %.2f m/s\n\n",msg.linear.x);


					pub.publish(msg);
					ros::spinOnce();
					loop_rate.sleep();
				}
				else
				{
					// Uses normal navigation
					msg.linear.x = FuzzyliteData::getLinSpeed();
					msg.angular.z = FuzzyliteData::getAngSpeed();

					system("clear");
					printf("Controle Ativo: POSIÇÃO CARTESIANA.\n");
					printf("Des pos X %.1f Y %.1f \n", desPos[0], desPos[1]);
					printf("Erro de orientação: %.2f rad\n",errorAng);
					printf("Erro de posição: %.2f m\n",errorPos);
					printf("Velocidade angular enviada: %.2f rad/s\n",msg.angular.z);
					printf("Velocidade linear enviada: %.2f m/s\n\n",msg.linear.x);

					pub.publish(msg);
					ros::spinOnce();
					loop_rate.sleep();
				}
			}

			msg.linear.x = 0;
			msg.angular.z = 0;
			pub.publish(msg);
			ros::spinOnce();

			printf("Reached destination!!\n");

		}
		

		msg.linear.x = 0;
		msg.angular.z = 0;
		pub.publish(msg);
		ros::spinOnce();

		printf("Reached destination!!\n");
	}

  	return 0;
}
