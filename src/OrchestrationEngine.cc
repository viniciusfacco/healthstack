/*
 * 
 *
 * Author: Vinicius Facco Rodrigues
 */


#include "OrchestrationEngine.hh"
#include "../../commons/LoggerSingleton.hh"
#include <boost/lexical_cast.hpp>

namespace Beholder {
    namespace Qualicare {

        // - OrchestrationEngine::OrchestrationEngine ----------------------------
        OrchestrationEngine::OrchestrationEngine():
			aWeightVector(std::vector<float>()),
			aWeightVectorCore(std::vector<float>()),
			aMMC(std::vector<int>()),
			aMQD(std::vector<int>()),
			aMQJ(std::vector<int>()),
			aMVD(std::vector<int>()),
			aMVJ(std::vector<int>()),
			aNumberOfApplications(0){
			//collectors
			aWeightVector.push_back(0.0); //w0
			aWeightVector.push_back(0.0); //w1
			aWeightVector.push_back(0.0); //w2
			aWeightVector.push_back(0.0); //w3
			aWeightVector.push_back(0.0); //w4
			aWeightVector.push_back(0.0); //w5
			aWeightVector.push_back(0.0); //w6
			aWeightVector.push_back(1.0); //b
			//core
			aWeightVectorCore.push_back(0.0); //w0
			aWeightVectorCore.push_back(0.0); //w1
			aWeightVectorCore.push_back(0.0); //w2
			aWeightVectorCore.push_back(1.0); //b
		}

        // - OrchestrationEngine::~OrchestrationEngine ---------------------------
        OrchestrationEngine::~OrchestrationEngine() {}

		float OrchestrationEngine::PA(int sensor_index, std::vector<float> &I){
			//W[] = [w0, w1, w2, w3, w4, w5, w6, 1]
			//I[] = [cpu, mem, net, conn, pl(pri), delay, jitter, b]	
			//-----------------------------------------------------------------------------			
			float w5 = 0;
			float w6 = 0;
            //if we have at least one application, e can compute the weights
			if (aNumberOfApplications > 0) {
				int matrix_index = sensor_index * aNumberOfApplications;
				//primeiro para w5 tenho que pegar os vetores de aMQD e aMDV			
				//extraio a submatrix do aMQD do sensor sensor_index
				auto mqdfirst = aMQD.cbegin() + matrix_index;
				auto mqdlast = aMQD.cbegin() + matrix_index + aNumberOfApplications;
				std::vector<int> mqd(mqdfirst, mqdlast);
				//extraio a submatrix do aMVD do sensor sensor_index
				auto mvdfirst = aMVD.cbegin() + matrix_index;
				auto mvdlast = aMVD.cbegin() + matrix_index + aNumberOfApplications;
				std::vector<int> mvd(mvdfirst, mvdlast);
				//calculo w5
				w5 = w(mqd, mvd);
				//agora para w6 tenho que pegar os vetores de aMQJ e aMJV
				//extraio a submatrix do aMQJ do sensor sensor_index
				auto mqjfirst = aMQJ.cbegin() + matrix_index;
				auto mqjlast = aMQJ.cbegin() + matrix_index + aNumberOfApplications;
				std::vector<int> mqj(mqjfirst, mqjlast);
				//extraio a submatrix do aMVJ do sensor sensor_index
				auto mvjfirst = aMVJ.cbegin() + matrix_index;
				auto mvjlast = aMVJ.cbegin() + matrix_index + aNumberOfApplications;
				std::vector<int> mvj(mvjfirst, mvjlast);
				//calculo w6
				w6 = w(mqj, mvj);
			}			
			//-----------------------------------------------------------------------------
			//now that we have w5 and w6, we can adjust the weight in the vector
			aWeightVector[5] = w5;
			aWeightVector[6] = w6;
			//and then multiply the vectores
			float weights_x_inputs = MultiplyOneDimentionalVector(8, aWeightVector, I);
			//finally, we apply the sigmoid activation function
			float result = sigmoid(weights_x_inputs);
			return result;
		}

		float OrchestrationEngine::PAc(std::vector<float> &I) {
			//W[] = [w0, w1, w2, 1]
			//I[] = [cpu, mem, net, b]				
            //and then multiply the vectores
			float weights_x_inputs = MultiplyOneDimentionalVector(4, aWeightVectorCore, I);
			//apply the sigmoid activation function
			float result = sigmoid(weights_x_inputs);
			return result;
		}

		void OrchestrationEngine::AddApplication(int sensor_index, int qos_delay, int qos_jitter){
			if (aNumberOfApplications == 0) {
				for (int i = 0; i < aNumberOfSensors; i++) {
					if (i == sensor_index) {
						aMMC.push_back(1);
						aMQD.push_back(qos_delay);
						aMQJ.push_back(qos_jitter);
					}
					else {
						aMMC.push_back(0);
						aMQD.push_back(0);
						aMQJ.push_back(0);
					}
					//as we are adding an application we do not have violations, so insert 0
					aMVD.push_back(0);
					aMVJ.push_back(0);
				}
			}
			else {
				int counter = aNumberOfSensors;
				for (int i = 0; i < aNumberOfSensors; i++) {
					int insert_position = (i + 1) * aNumberOfApplications + (aNumberOfSensors - counter);
					counter--;
					if ((i + 1) == aNumberOfSensors) {//if this is the last, the position is the final of the vector
						if (i == sensor_index) {
							aMMC.push_back(1);
							aMQD.push_back(qos_delay);
							aMQJ.push_back(qos_jitter);
						}
						else {
							aMMC.push_back(0);
							aMQD.push_back(0);
							aMQJ.push_back(0);
						}
                        //as we are adding an application we do not have violations, so insert 0
						aMVD.push_back(0);
						aMVJ.push_back(0);
					} else {//if it is not the last, we have to insert in the right position
						if (i == sensor_index) {
							aMMC.insert(aMMC.begin() + insert_position, 1);
							aMQD.insert(aMQD.begin() + insert_position, qos_delay);
							aMQJ.insert(aMQJ.begin() + insert_position, qos_jitter);
						}
						else {
							aMMC.insert(aMMC.begin() + insert_position, 0);
							aMQD.insert(aMQD.begin() + insert_position, 0);
							aMQJ.insert(aMQJ.begin() + insert_position, 0);
						}
                        //as we are adding an application we do not have violations, so insert 0
						aMVD.insert(aMVD.begin() + insert_position, 0);
						aMVJ.insert(aMVJ.begin() + insert_position, 0);
					}
				}
			}
			aNumberOfApplications++;
			BEHOLDER_LOG(BH_INFO, "OrchestrationEngine::AddApplication: new application. Now the total is " + boost::lexical_cast<std::string>(aNumberOfApplications));
			PrintMMC();
		}

		void OrchestrationEngine::UpdateApplication(int app_index, int sensor_index, int qos_violation_delay, int qos_violation_jitter){
			int insert_position = (sensor_index * aNumberOfApplications) + app_index;
			aMVD[insert_position] = qos_violation_delay;
			aMVJ[insert_position] = qos_violation_jitter;
		}

		void OrchestrationEngine::PrintMatrix(std::vector<int>& matrix)	{
			std::string output = "";
			for (int i = 0; i < (aNumberOfSensors * aNumberOfApplications); i++) {				
				output += boost::lexical_cast<std::string>(matrix[i]) + "\t";
				if (((i+1) % aNumberOfApplications) == 0) {
					output += "\n";
				}
			}			
			BEHOLDER_LOG(BH_INFO, "OrchestrationEngine::PrintMatrix:\n" + output);
		}

		float OrchestrationEngine::iu(std::vector<int> &V){
			float result = 0;
			int lowest_value = INT_MAX;
			int sum_values = 0;
			for (int i = 0; i < aNumberOfApplications; i++) {
				if (V[i] > 0) {
					if (V[i] < lowest_value) {
						lowest_value = V[i];
					}
				}				
				sum_values += V[i];
			}
			if (sum_values > 0) {
				result = 1.0f / static_cast<float>(lowest_value);
			}
			return result;
		}

		float OrchestrationEngine::w(std::vector<int> &MQ, std::vector<int> &MV){
			float importance_unit = iu(MQ);
			int sum_violations = 0;
			for (int i = 0; i < aNumberOfApplications; i++) {
				sum_violations += MV[i];
			}
			float result = importance_unit * (1 + aAdaptingRate * sum_violations);
			return result;
		}

		float OrchestrationEngine::sigmoid(float input){
			return 1.0f / (1 + std::exp(-1 * input));
		}

		float OrchestrationEngine::MultiplyOneDimentionalVector(int size, std::vector<float> &v1, std::vector<float> &v2){
			float result = 0;
			for (int i = 0; i < size; i++) {
				result += v1[i] * v2[i];
			}
			return result;
		}

    } // namespace Qualicare
} // namespace Beholder
