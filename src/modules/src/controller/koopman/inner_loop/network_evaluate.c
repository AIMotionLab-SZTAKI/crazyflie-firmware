
#include "controller_koopman.h"
#include <math.h>


static float output_0[NUM_NEURONS];
static float output_1[NUM_NEURONS];
static float output_2[NUM_OUTPUTS];
static float residual_output[NUM_OUTPUTS];

float linear(float num) {
	return num;
}


float sigmoid(float num) {
	return 1 / (1 + exp(-num));
}


// float tanh(float num) {
// 	return (exp(2*num) - 1) / (exp(2*num) + 1);
// }

float relu(float num) {
	if (num > 0) {
		return num;
	} else {
		return 0;
	}
}

void network_evaluate(float *phys_state, float *koop_state) {

	for (int i = 0; i < structure[0][1]; i++) {
		output_0[i] = 0;
		for (int j = 0; j < structure[0][0]; j++) {
			output_0[i] += phys_state[j] * layer_0_weight[j][i];
		}
		output_0[i] += layer_0_bias[i];
		output_0[i] = tanh(output_0[i]);
	}

	for (int i = 0; i < structure[1][1]; i++) {
		output_1[i] = 0;
		for (int j = 0; j < structure[1][0]; j++) {
			output_1[i] += output_0[j] * layer_1_weight[j][i];
		}
		output_1[i] += layer_1_bias[i];
		output_1[i] = tanh(output_1[i]);
	}
	
	for (int i = 0; i < structure[2][1]; i++) {
		output_2[i] = 0;
		for (int j = 0; j < structure[2][0]; j++) {
			output_2[i] += output_1[j] * layer_2_weight[j][i];
		}
		output_2[i] += layer_2_bias[i];
	}

	// Residual connection from phys_state to output_2 through a linear layer
	for (int i = 0; i < structure[2][1]; i++) {
		residual_output[i] = 0;
		for (int j = 0; j < structure[0][0]; j++) {
			residual_output[i] += phys_state[j] * residual_weight[j][i];
		}
		residual_output[i] += residual_bias[i];
	}

	// Add the residual connection to the output
	for (int i = 0; i < structure[2][1]; i++) {
		koop_state[i] = output_2[i];
		koop_state[i] += residual_output[i];
	}

}
	