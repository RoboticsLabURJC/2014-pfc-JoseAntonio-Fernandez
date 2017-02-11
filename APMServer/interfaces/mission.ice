/*
 *  Copyright (C) 1997-20103 JDE Developers Team
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/.
 *
 *  Authors : Jose Antonio Fernandez Casillas
 */


#ifndef POSE3D_ICE
#define POSE3D_ICE


module jderobot{  
	/**
	* Pose3D data information 
	*/
  class Pose3DData
  {
		float x;
		float y;
		float z;
  	float h;
		float q0;
		float q1;
		float q2;
		float q3;
  };
  sequence<Pose3DData> PoseSequence;
  /**
  * Mission data information 
  */
  class MissionData
  {
    PoseSequence mission;
  };

  /** 
   * Interface to the Mission.
   */
  interface Mission
  {
    idempotent PoseSequence getMissionData();
    int setMissionData(PoseSequence data);
  };
}; //module

#endif //Mission_ice