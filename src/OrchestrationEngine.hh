/*
 * 
 */

// Author: Vinicius Facco Rodrigues

#ifndef Beholder_Qualicare_OrchestrationEngine_h
#define Beholder_Qualicare_OrchestrationEngine_h

#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

namespace Beholder {
    namespace Qualicare {

        class OrchestrationEngine {
        public:
		
            /*! \brief Class constructor. */
            OrchestrationEngine();
			
            /*! \brief Class destructor. */
            ~OrchestrationEngine();

			//PA
			float PA(int sensor_index, std::vector<float> &I);

			float PAc(std::vector<float> &I);

			//setters
			void NumberOfSensors(int value) { aNumberOfSensors = value; };
			void AdaptingRate(float value) { aAdaptingRate = value; };
			void w_0(float weight) { aWeightVector[0] = weight; }
			void w_1(float weight) { aWeightVector[1] = weight; }
			void w_2(float weight) { aWeightVector[2] = weight; }
			void w_3(float weight) { aWeightVector[3] = weight; }
			void w_4(float weight) { aWeightVector[4] = weight; }
			void w_0_core(float weight) { aWeightVectorCore[0] = weight; }
			void w_1_core(float weight) { aWeightVectorCore[1] = weight; }
			void w_2_core(float weight) { aWeightVectorCore[2] = weight; }

			//gettrs
			float w_5() { return aWeightVector[5]; }
			float w_6() { return aWeightVector[6]; }

			void AddApplication(int sensor_index, int qos_delay, int qos_jitter);
			void UpdateApplication(int app_index, int sensor_index, int qos_violation_delay, int qos_violation_jitter);
			void PrintMMC() { PrintMatrix(aMMC); }
			void PrintMQD() { PrintMatrix(aMQD); }
			void PrintMQJ() { PrintMatrix(aMQJ); }
			void PrintMVD() { PrintMatrix(aMVD); }
			void PrintMVJ() { PrintMatrix(aMVJ); }

        private:

			void PrintMatrix(std::vector<int> &matrix);

			//importance factor
			float iu(std::vector<int> &V);
			
			//w5(s) and w6(s)
			float w(std::vector<int> &MQ, std::vector<int> &MV);

			//sigmoid function
			float sigmoid(float input);

			float MultiplyOneDimentionalVector(int size, std::vector<float> &v1, std::vector<float> &v2);

			//matrizes with dimentions aNumberOfApplications x aNumberOfSensors
			//exemple 3 aplications and 3 sensors
			//[0   1   2    3    4   5  6   7   8 ]
			//|s1          |s2         |s3        |
			//|a1, a2, a3  |a1, a2, a3 |a1, a2, a3|
			std::vector<int> aMMC;
			std::vector<int> aMQD;
			std::vector<int> aMQJ;
			std::vector<int> aMVD;
			std::vector<int> aMVJ;

			int aNumberOfApplications;
			int aNumberOfSensors;
			float aAdaptingRate;
			std::vector<float> aWeightVector;
			std::vector<float> aWeightVectorCore;

        };

    } // namespace Calibration
} // namespace Beholder


// Beholder_Qualicare_OrchestrationEngine_h
#endif