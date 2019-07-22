/**
* Copyright (C) 2017 Chalmers Revere
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
* USA.
*/

#include "pathplanner.hpp"
/*----------------Utility functions----------------------*/
double getNorm(double x1, double y1, double x2, double y2){
  return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

//Find angle difference from vector (x1,y1) to vector (x2,y2) in 2d shperical coordinate
double getDisplacementAngle(std::vector<Point2D> pastCones, std::vector<Cone> currentCones){
  double x1 = pastCones[0].x;
  double y1 = pastCones[0].y;
  double x2 = currentCones[0].m_x;
  double y2 = currentCones[0].m_y;
  return (atan2(y2,x2) - atan2(y1,x1));
}

//Find translation difference from vector (x1,y1) to vector (x2,y2) in 2d spherical coordinate
Eigen::ArrayXXf getDisplacementTranslation(std::vector<Point2D> pastCones, std::vector<Cone> currentCones){
  double x1 = pastCones[0].x;
  double y1 = pastCones[0].y;
  double x2 = currentCones[0].m_x;
  double y2 = currentCones[0].m_y;
  Eigen::ArrayXXf vec(1,2);
  vec << (float)(x2-x1), (float)(y2-y1);
  return vec;
}
/*-------------------------------------------------------*/
PathPlanner::PathPlanner(cluon::OD4Session &od4, std::map<std::string, std::string> commandlineArguments) :
    m_od4(od4),
    m_pastBlue{},
    m_pastYellow{},
    m_Cones{},
    middlePath{},
    m_currentConeFrame{},
    m_status(),
    m_isSkidpad(),
    m_verbose(),
    m_debug(),  
    currentAim()
    
{
  m_verbose = static_cast<bool>(commandlineArguments.count("verbose") != 0);
  m_debug = static_cast<bool>(commandlineArguments.count("debug") != 0);
}

/* Sort based on: nearest neighbour */
std::vector<Cone> PathPlanner::SortConesCFSD19(std::vector<Cone> cones){
  std::vector<Cone> SortedCones;
  //Choose starting position, if there's new cones added to the sides then start from those cones positon...
  double currentX;
  double currentY;
  if(cones[0].m_objectId == 999){
    currentX = cones[0].m_x;
    currentY = cones[0].m_y;
  }
  //otherwise, start from the car's position
  currentX = 0.0;
  currentY = 0.0;
  
  while(cones.size() > 1)
  {
    int closestIndex=0;
    double closestDistance = 999.0;
    //Loop through the remaining cones, find the closest one to the current cone
    for(uint32_t i=0;i<cones.size();i++)
    {
      double distance = getNorm(currentX, currentY, cones[i].m_x, cones[i].m_y);
      if(distance < closestDistance){
        closestIndex = i;
        closestDistance = distance;
      }
    }

    //Assign next starting cone to the closest cone found
    currentX = cones[closestIndex].m_x;
    currentY = cones[closestIndex].m_y;

    //push the closest cone to the shorted list, and remove it from the test list
    Cone newSortedCone = cones[closestIndex];
    SortedCones.push_back(newSortedCone);
    cones.erase (cones.begin()+closestIndex);   
  }
  //When there is only one cone left, push it to the sorted list
  SortedCones.push_back(cones[0]);
  return SortedCones;
}

void PathPlanner::ProcessFrameCFSD19(){
    if(middlePath.size())
      middlePath.clear();
               
    if(m_currentConeFrame.size() > 0){
      if(m_debug)
      std::cout << "Current frame has: " << m_currentConeFrame.size() << " cones\n";
			
      //Copy cones of different colors to their own containers for processing
      std::vector<Cone> tempYellowCones;
      std::vector<Cone> tempBlueCones;
      std::vector<Cone> tempOrangeCones;     
      while(m_currentConeFrame.size() >0){
      
      std::cout << "m_currentConeFrame.front().m_color = " << m_currentConeFrame.front().m_color << std::endl;       
        if(m_currentConeFrame.front().m_color == 1){ //blue
      //  Cone cone = m_currentConeFrame.front();
      //  tempBlueCones.push_back(cone);
      }
      else
      if(m_currentConeFrame.front().m_color == 0){ //yellow
        Cone cone = m_currentConeFrame.front();
        tempYellowCones.push_back(cone);
      }
      else
      if(m_currentConeFrame.front().m_color == 2){ //orange
        Cone cone = m_currentConeFrame.front();
        tempOrangeCones.push_back(cone);
      }
      //Done copying, delete pointers to free memory
      m_currentConeFrame.pop();
      }
			
      if(m_debug){
        std::cout << "frame: blues: " << tempBlueCones.size() << " , yellow: " << tempYellowCones.size() << std::endl;
      }
			 
      //Filter out unncessary cones for Skidpad, based on current Skidpad state-machine
      //Shoud this be Mission Controller's job ?
      if(m_isSkidpad){
			
      }
			
      //Find matched cone pairs between current and last frame
      std::vector<Point2D> matchedBlues;
      std::vector<Point2D> matchedYellows;

			
      if(tempBlueCones.size()  && m_pastBlue.size())
        matchedBlues = MatchConePair(tempBlueCones, m_pastBlue);
      if(tempYellowCones.size()  && m_pastYellow.size())
        matchedYellows = MatchConePair(tempYellowCones, m_pastYellow);
			    
      //Calculate displacement angle and distance of matched cone pairs
      
      Eigen::ArrayXXf displaceDistanceYellow(1,2);     
      Eigen::ArrayXXf displaceDistanceBlue(1,2);
      Eigen::ArrayXXf displaceDistance(1,2);
      
      displaceDistanceYellow << 0,0;
      displaceDistanceBlue << 0,0;
      displaceDistance << 0,0;
      
      //std::cout << "matchedBlues.size() " << matchedBlues.size() <<std::endl;
      //std::cout << "matchedYellows.size() " << matchedYellows.size() <<std::endl;
      
      if(matchedBlues.size() && matchedYellows.size()){
        displaceDistanceBlue = getDisplacementTranslation(matchedBlues, tempBlueCones);
        displaceDistanceYellow = getDisplacementTranslation(matchedYellows, tempYellowCones);
        displaceDistance = 0.5*(displaceDistanceBlue + displaceDistanceYellow);
      }
      else
      if(matchedBlues.size())
      {
        displaceDistance = getDisplacementTranslation(matchedBlues, tempBlueCones);
      }
      else
      if(matchedYellows.size())
      {
        displaceDistance = getDisplacementTranslation(matchedYellows, tempYellowCones);
      }
			        
      //Predict curret cone positions from last frame's cones
      std::vector<Point2D> PredictedBlues;
      std::vector<Point2D> PredictedYellows;
			
      if(tempBlueCones.size() && m_pastBlue.size())
        PredictedBlues = PredictConePositions(tempBlueCones, &m_pastBlue, displaceDistance);
      if(tempYellowCones.size() && m_pastYellow.size())
        PredictedYellows = PredictConePositions(tempYellowCones, &m_pastYellow, displaceDistance);
			
			//If there is only one cone in this frame, place two virtual cones on the car's side
			/*
			if(tempBlueCones.size() <= 1 && tempYellowCones.size() <= 1)
			{
			  double trackWidth = 2.8;
			  Cone virtualBlue(999, -trackWidth/2, 0, 1);
			  Cone virtualYellow(999, trackWidth/2+0.01, 0.01, 2);
			  //tempBlueCones.push_back(virtualBlue);
			  //tempYellowCones.push_back(virtualYellow);
			  tempBlueCones.insert(tempBlueCones.begin(), virtualBlue);
			  tempYellowCones.insert(tempYellowCones.begin(), virtualYellow);
			}
			*/
      //Guess missing cones, choose the row with more cones as reference
      //First, Sort cones from closest to furthest, based on distance to car
      if(tempBlueCones.size()>1)
        tempBlueCones = SortConesCFSD19(tempBlueCones);
      if(tempYellowCones.size()>1)
        tempYellowCones = SortConesCFSD19(tempYellowCones);  
      
      GuessMissingCones(&tempBlueCones, &tempYellowCones);
      if(tempBlueCones.size()>1)
        tempBlueCones = SortConesCFSD19(tempBlueCones);
      if(tempYellowCones.size()>1)
        tempYellowCones = SortConesCFSD19(tempYellowCones);  
      
      //Create virtual points on each cone row to make middle path
      uint32_t numberOfSidePoints = 6;
      Eigen::ArrayXXf BlueSidePoints;
			Eigen::ArrayXXf YellowSidePoints;
			
      if(tempBlueCones.size() > 1 and  tempYellowCones.size() > 1){
        BlueSidePoints = MakeSidePoints(tempBlueCones, numberOfSidePoints);
        YellowSidePoints = MakeSidePoints(tempYellowCones, numberOfSidePoints);
      }

      //Create middle path
      if(BlueSidePoints.rows() > 1 || YellowSidePoints.rows() > 1)
      {
        for(uint32_t i=0;i<BlueSidePoints.rows();i++){
          Point2D middlePoint;
          middlePoint.x = 0.5*(BlueSidePoints(i,0)+YellowSidePoints(i,0));
          middlePoint.y = 0.5*(BlueSidePoints(i,1)+YellowSidePoints(i,1));
          middlePath.push_back(middlePoint);
          //std::cout << "middle path: " << i << ": " <<middlePoint.x<< " " << middlePoint.y << std::endl; 
        }
        //If the path is too far from the car, then add more path points from the car
        /*
        double totalPathLength = 0;
        double averagePathLength = 0;
        for(uint32_t i=0;i<middlePath.size()-1;i++){
          double firstX = middlePath[i].x;
          double firstY = middlePath[i].y;
          double secondX = middlePath[i+1].x;
          double secondY = middlePath[i+1].y;
          totalPathLength += getNorm(firstX, firstY, secondX, secondY);
        }
        averagePathLength = totalPathLength / (numberOfSidePoints-1);
        double distanceToPath = getNorm(0.0,0.0,middlePath[0].x, middlePath[0].y);
        //Additional path points should have the same average distance betweem them as the current path points
        if(distanceToPath > averagePathLength)
        {
          std::vector<Point2D> additionalPoints;
          int numberOfNewPoints = (int)(distanceToPath/averagePathLength);
          int numberOfNewPointsMax = 8;
           
          if(numberOfNewPoints > numberOfNewPointsMax)
            numberOfNewPoints = numberOfNewPointsMax;
          double vecX = middlePath[0].x/numberOfNewPoints;
          double vecY = middlePath[0].y/numberOfNewPoints;
          
          if(m_verbose){
            std::cout << "distanceToPath: " << distanceToPath << std::endl;
            std::cout << "averagePathLength: " << averagePathLength << std::endl;
            std::cout << "numberOfNewPoints: " << numberOfNewPoints << std::endl;
          }
          
          if(numberOfNewPoints > 0)
          {
            for(uint32_t i=0;i<numberOfNewPoints;i++){
              Point2D newPoint;
              newPoint.x = i*vecX;
              newPoint.y = i*vecY;
              additionalPoints.push_back(newPoint);
            }
            //Insert new path points to the current path points
            middlePath.insert(middlePath.begin(), additionalPoints.begin(), additionalPoints.end());
          }
        }
        
        */
      }
			
			//Done processing, start making new frame
    	m_status = isMakingFrame;
    	
			//display cones from perception

      if(m_debug){
        ShowResult(tempBlueCones, tempYellowCones, tempOrangeCones, PredictedBlues, PredictedYellows);
      }
      
        //Update current and past cone positions
        m_pastBlue.clear();
        m_pastYellow.clear();
        if(tempBlueCones.size()){
          for(uint32_t i=0;i<tempBlueCones.size();i++){
            Point2D cone;
            cone.x = tempBlueCones[i].m_x;
            cone.y = tempBlueCones[i].m_y;
            m_pastBlue.push_back(cone);
          }
        }
		  
        if(PredictedBlues.size()){
          for(uint32_t i=0;i<PredictedBlues.size();i++){
            Point2D cone = PredictedBlues[i];
            m_pastBlue.push_back(cone);
          }
        }
		  
        if(tempYellowCones.size()){
          for(uint32_t i=0;i<tempYellowCones.size();i++){
            Point2D cone;
            cone.x = tempYellowCones[i].m_x;
            cone.y = tempYellowCones[i].m_y;
            m_pastYellow.push_back(cone);
          }
        }
		  
        if(PredictedYellows.size()){
        for(uint32_t i=0;i<PredictedYellows.size();i++){
          Point2D cone = PredictedYellows[i];
          m_pastYellow.push_back(cone);
        }
      }		  
    }
    else // not enough cone for path planning
    {
      m_status = isMakingFrame;
      if(m_debug)
        std::cout << "Not enough cones\n";
    }
}

/* Find cones from previous frame that match observed cones in current frame */
std::vector<Point2D> PathPlanner::MatchConePair(std::vector<Cone> observedCone, std::vector<Point2D> pastCone){
  double displaceDistance = 1;
  std::vector<Point2D> MatchedCones;

  for(uint32_t i=0;i<observedCone.size();i++)
  {
    for(uint32_t j=0;j<pastCone.size();j++){
      if(getNorm(observedCone[i].m_x, observedCone[i].m_y, pastCone[j].x, pastCone[j].y)<displaceDistance)
      {// if a past cone is closer the the currently tested cone by a preset distance then it's considered a match
        Point2D matchedCone;
        matchedCone.x = pastCone[j].x;
        matchedCone.y = pastCone[j].y;
        MatchedCones.push_back(matchedCone);
        break;
      }
    }
  }
  return MatchedCones;
}

/* Predict cone positions in current frame */
std::vector<Point2D> PathPlanner::PredictConePositions(std::vector<Cone> observedCone, std::vector<Point2D> *pastCone, Eigen::ArrayXXf displaceVector){
  std::vector<Point2D> PredictedCones;
  for(uint32_t i=0;i<pastCone->size();i++)
  {
    //Find magnitude and arctan angle of past cone positions
    //Then predict current cone positions by moving past cone positions with the displace angle and distance

    double newX = pastCone->at(i).x + displaceVector(0,0);
    double newY = pastCone->at(i).y + displaceVector(0,1);
    bool rejectPrediction = false;
    double seperateDistance = 0.2;
    //if the predicted position is close to an already observed cone, then reject it;
    for(uint32_t j=0; j<observedCone.size(); j++){
      if(getNorm(newX, newY, observedCone[j].m_x, observedCone[j].m_y) < seperateDistance){
        rejectPrediction = true;
        break;
      }
    }
    
    //if the predicted position is close to a past cone, then replace that past cone with new prediction;
    for(uint32_t j=0; j<pastCone->size(); j++){
      if(getNorm(newX, newY, pastCone->at(i).x, pastCone->at(i).y) < seperateDistance){
        pastCone->at(i).x = newX;
        pastCone->at(i).y = newY;
        rejectPrediction = true;
        break;
      }
    }
    
    //if the predicted position is out of range, then reject it;
    if(newX < -20 || newX > 20 || newY < -10)
      rejectPrediction = true;
    if(!rejectPrediction){
    // Push new predicted cone position to return container
    Point2D predictedCone;
    predictedCone.x = newX;
    predictedCone.y = newY;
    PredictedCones.push_back(predictedCone);
    } 
  }
  return PredictedCones;
}

void PathPlanner::GuessMissingCones(std::vector<Cone>* blues, std::vector<Cone>* yellows){
  //Choose the row with more cones as reference row
  float guessDistance = 2.8f;
  std::vector<Cone>* ref; //reference row with more cones
  std::vector<Cone>* fewer; //modified row with fewer cones
  if(blues->size() >= yellows->size())
  {
    ref = blues;
    fewer = yellows;
  }
  else
  {
    ref = yellows;
    fewer = blues;
  }
  if(ref->size() < 2){
    //std::cout << "Too few cones to guess\n"; 
    return;
  }
    
  float direction;
  if((*ref)[0].m_color == 1) //if reference cones are blue, then guess to the right
  {
    direction = -1.0;
  }
  else // ...otherwise, guess to the left
  {
    direction = 1.0;
  }
  
  //For every cone in the reference row, make a perpedicular projection of the cone to the other row, and use that as a guessed cone
  for(uint32_t i=0;i<(*ref).size()-1;i++)
  {  
    Eigen::ArrayXXf firstCone(1,2);
    Eigen::ArrayXXf secondCone(1,2);
    firstCone << (float)(*ref)[i].m_x, (float)(*ref)[i].m_y;
    secondCone << (float)(*ref)[i+1].m_x, (float)(*ref)[i+1].m_y;
    
    if(m_verbose){
      std::cout << "firstCone: " << firstCone(0,0) << ", " << firstCone(0,1) << std::endl;
      std::cout << "secondCone: " << secondCone(0,0) << ", " << secondCone(0,1) << std::endl;
    }
    
    Eigen::ArrayXXf vector = secondCone-firstCone;

    Eigen::ArrayXXf normal(1,2);
    normal << -vector(1),vector(0);
    normal = normal/((normal.matrix()).norm());
    Eigen::ArrayXXf guessVector = direction*guessDistance*normal;
    
    Eigen::ArrayXXf guessedConePos(1,2);
    guessedConePos << secondCone(0,0)+guessVector(0,0),secondCone(0,1)+guessVector(0,1);
    
    Cone newGuessedCone(111, guessedConePos(0,0), guessedConePos(0,1),0);
    if((*ref)[i].m_color == 1)
       newGuessedCone.m_color = 0;
    else
       newGuessedCone.m_color = 1;
    (*fewer).push_back(newGuessedCone);
    //Make projection for the first cone in the reference row as well
    if(i==0)
    {
      guessedConePos << firstCone(0,0)+guessVector(0,0),firstCone(0,1)+guessVector(0,1);
    
      Cone newCone(999, guessedConePos(0,0), guessedConePos(0,1),0);
      if((*ref)[i].m_color == 1)
        newCone.m_color = 0;
      else
        newCone.m_color = 1;
      (*fewer).push_back(newCone);
    }
    if(m_verbose)
      std::cout << "direction: " << direction << ", guessVector=(" << guessVector(0) << ", " <<guessVector(1) <<std::endl;
  }
}

Eigen::ArrayXXf PathPlanner::MakeSidePoints(std::vector<Cone> cones, uint32_t numberOfPoints){
  // Places linearly equidistant points along a sequence of points
  int nCones = cones.size();
  Eigen::ArrayXXf sidePoints(nCones,2);
  for(uint32_t i=0; i<cones.size();i++){
    sidePoints(i,0) = (float)cones[i].m_x;
    sidePoints(i,1) = (float)cones[i].m_y;
  }
  
  // Full path length, and save lengths of individual segments
  float pathLength = 0;
  Eigen::ArrayXXf segLength(nCones-1,1);
  for(int i = 0; i < nCones-1; i = i+1)
  {
    segLength(i) = ((sidePoints.row(i+1)-sidePoints.row(i)).matrix()).norm();
    //segLength(i) = (float)getNorm(cones[i+1].m_x, cones[i+1].m_y, cones[i].m_x, cones[i].m_y);
    pathLength = pathLength + segLength(i);
  }
  
  // Calculate equal subdistances
  float eqDistance = pathLength/(numberOfPoints-1);
  // The latest point that you placed
  Eigen::ArrayXXf latestPointCoords = sidePoints.row(0);
  // The latest cone that you passed
  int latestConeIndex = 0;
  // How long is left of the current segment
  float remainderOfSeg = segLength(0);
  // The new list of linearly equidistant points
  Eigen::ArrayXXf newSidePoints(numberOfPoints,2);
  // The first point should be at the same place as the first cone
  newSidePoints.row(0) = latestPointCoords;

  // A temporary vector
  Eigen::ArrayXXf vec(1,2);
  // Temporary distances
  float distanceToGoFromLatestPassedPoint, lengthOfNextSeg;
  // Start stepping through the given path
  for(uint32_t i = 1; i < numberOfPoints-1; i = i+1)
  {
    // If the next point should be in the segment you are currently in, simply place it.
    if(remainderOfSeg > eqDistance)
    {
      vec = sidePoints.row(latestConeIndex+1)-latestPointCoords;
      latestPointCoords = latestPointCoords + (eqDistance/remainderOfSeg)*vec;
    }
    else // If you need to go to the next segment, keep in mind which cones you pass and how long distance you have left to go.
    {
      latestConeIndex = latestConeIndex+1;
      distanceToGoFromLatestPassedPoint = eqDistance-remainderOfSeg;
      lengthOfNextSeg = segLength(latestConeIndex); 
      
      while(distanceToGoFromLatestPassedPoint > lengthOfNextSeg)
      {
        latestConeIndex = latestConeIndex+1;
        distanceToGoFromLatestPassedPoint = distanceToGoFromLatestPassedPoint - lengthOfNextSeg;
        lengthOfNextSeg = segLength(latestConeIndex); 
      } // End of while

      latestPointCoords = sidePoints.row(latestConeIndex);
      vec = sidePoints.row(latestConeIndex+1)-latestPointCoords;
      latestPointCoords = latestPointCoords + (distanceToGoFromLatestPassedPoint/segLength(latestConeIndex))*vec; 
    } // End of else

    // In every case, save the point you just placed and check how much of that segment is left.
    newSidePoints.row(i) = latestPointCoords;
    remainderOfSeg = ((sidePoints.row(latestConeIndex+1)-latestPointCoords).matrix()).norm();
  } // End of for
  // The last point should be at the same place as the last cone.
  newSidePoints.row(numberOfPoints-1) = sidePoints.row(nCones-1); 
  return newSidePoints;
}

void PathPlanner::ShowResult(std::vector<Cone> blue, std::vector<Cone> yellow, std::vector<Cone> orange, std::vector<Point2D> PredictedBlues, std::vector<Point2D> PredictedYellows){
  int outWidth = 376;
  int outHeight = 376;
  int heightOffset = 50;
  double resultResize = 15;
  cv::Mat outImg = cv::Mat::zeros(outWidth,outHeight,CV_8UC4);
  cv::Mat outImgOrange = cv::Mat::zeros(outWidth,outHeight,CV_8UC4);    
  // Draw circles on output image
  cv::Scalar coneColor(255,255,255); 

  if(blue.size()){//blue cones
    int colorIncrement = 0; //for distingusing sorted cone
    for(uint32_t i=0; i < blue.size(); i++){
      int xt = int(blue[i].m_x * float(resultResize) + outWidth/2);
      int yt = int(blue[i].m_y * float(resultResize));
      coneColor= cv::Scalar(255,0,colorIncrement);
      cv::circle(outImg, cv::Point(xt,-yt+outHeight-heightOffset), 6, coneColor, -1);
      colorIncrement = colorIncrement + 80;
    }
  }

  if(yellow.size()){//yellow cones
    int colorIncrement = 0; //for distingusing sorted cone
    for(uint32_t i=0; i < yellow.size(); i++){
      int xt = int(yellow[i].m_x * float(resultResize) + outWidth/2);
      int yt = int(yellow[i].m_y * float(resultResize));
      coneColor= cv::Scalar(0,255,255 + colorIncrement);
      cv::circle(outImg, cv::Point(xt,-yt+outHeight-heightOffset), 6, coneColor, -1);
      colorIncrement = colorIncrement + 80;
    }
  }

  if(orange.size()){//orange cones
    for(uint32_t i=0; i < orange.size(); i++){
      int xt = int(orange[i].m_x * float(resultResize) + outWidth/2);
      int yt = int(orange[i].m_y * float(resultResize));
      coneColor= cv::Scalar(0,69,255);
      cv::circle(outImgOrange, cv::Point(xt,-yt+outHeight-heightOffset), 6, coneColor, -1);
    }
  }

  //Show middle path
   if(middlePath.size()){
    int colorIncrement = 0; //for distingusing sorted cone
    for(uint32_t i=0; i < middlePath.size(); i++){
      int xt = int(middlePath[i].x * float(resultResize) + outWidth/2);
      int yt = int(middlePath[i].y * float(resultResize));
      coneColor= cv::Scalar(100,255,colorIncrement);
      cv::circle(outImg, cv::Point(xt,-yt+outHeight-heightOffset), 2, coneColor, -1);
      colorIncrement = colorIncrement + 40;
    }
  }

  //Show prediction
  /*
  if(PredictedBlues.size()){//predicted blue cones
    for(uint32_t i=0; i < PredictedBlues.size(); i++){
      int xt = int(PredictedBlues[i].x * float(resultResize) + outWidth/2);
      int yt = int(PredictedBlues[i].y * float(resultResize));
      coneColor= cv::Scalar(236,236,0);
      cv::circle(outImg, cv::Point(xt,-yt+outHeight-heightOffset), 4, coneColor, -1);
    }
  }
  
  if(PredictedYellows.size()){//predicted yellow cones
    for(uint32_t i=0; i < PredictedYellows.size(); i++){
      int xt = int(PredictedYellows[i].x * float(resultResize) + outWidth/2);
      int yt = int(PredictedYellows[i].y * float(resultResize));
      coneColor= cv::Scalar(0,221,133);
      cv::circle(outImg, cv::Point(xt,-yt+outHeight-heightOffset), 4, coneColor, -1);
    }
  }
  */
  //Show aimpoint from Aimpoint module
  //current aimpoint
  int xt = int(currentAim.x * float(resultResize) + outWidth/2);
  int yt = int(currentAim.y * float(resultResize));
  cv::Scalar aimpointColor(0,0,255);
  cv::circle(outImg, cv::Point(xt,-yt+outHeight-heightOffset), 4, aimpointColor, -1);
  
  // Show result image
  cv::imshow("detectconelane-blue-yellow", outImg);
  cv::imshow("detectconelane-orange", outImgOrange);
  cv::waitKey(1);
}
