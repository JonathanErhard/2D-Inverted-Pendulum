/*

  Copyright (C) 2021  Biagio Festa

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/

#include <cmath>
#include <iostream>
#include "EvolutionNet/EvolutionNet.hpp"
#include "Pencil_Sim.h"

static constexpr int NumInput = 4;
static constexpr int NumOutput = 1;
static constexpr bool Bias = true;
static constexpr float ThresholdFitness = 0.99f;
static constexpr std::size_t PopulationSize = 100;
using ParamConfig = EvolutionNet::DefaultParamConfig;
using EvolutionNetT = EvolutionNet::EvolutionNet<NumInput, NumOutput, Bias, ParamConfig>;
using Network = EvolutionNetT::NetworkT;
using FitnessScore = EvolutionNet::FitnessScore;

std::string NetworkFilename = "EvoController";

void SaveNetworkOnFile(const Network& network) {
  std::cout << "Trying saving network on file '" << NetworkFilename << "'\n";
  std::ofstream file(NetworkFilename, std::ios_base::out | std::ios_base::binary);
  if (file.fail()) {
    std::cerr << "  Cannot save the network on file '" << NetworkFilename << "'\n";
    return;
  }

  network.serialize(&file);
  file.close();
  std::cout << "  Saved!\n";
}

int main() {
  EvolutionNetT evolutionNet;
  evolutionNet.initialize(PopulationSize);

  int i = 0;
  
  while (true) {
    evolutionNet.evaluateAll([](Network* network) {

      //INIT POPULATION
      Pencil_Sim pensim(1e-4,(Regulator*)0,1);
      
      double state[4];
      uint survivaled = 0;

      float scoreS = 0.0;
      //SIMULATE
      while(pensim.simulation_time < 10.0) {
        //SET INPUTS

        network->setInputValue(0, (state[0]+0.05)/0.1);
        network->setInputValue(1, (state[1]+3.0)/6.0);
        network->setInputValue(2, (state[2]+M_PI/2)+M_PI);
        network->setInputValue(3, (state[3]+3.0)/6.0);

        //GET OUTPUTS
        network->feedForward<ParamConfig>();
        const float output = network->getOutputValue(0);
        assert(output >= 0.f && output <= 1.f);

        //PUSH BACK OUTPUTS
        pensim.next_state((output-0.5)*50.0,state);

        scoreS = (pensim.simulation_time);
        if(std::abs(state[2]) > 1.5)
          break;

      }

      FitnessScore score = scoreS;

      //std::cout << score << std::endl;

      network->setFitness(score);
      network->timeSurvived = pensim.simulation_time;

      //std::cout<<score<<"  "<<survivaled<<std::endl;
    });
    i++;

      std::cout<<"Current Longest Survival: "<<evolutionNet.getBestNetwork().timeSurvived<<"  "<<std::endl;
    
    if (evolutionNet.getBestFitness() >= ::ThresholdFitness) {
      break;
    }

    evolutionNet.evolve();
  }

  auto finalNetwork = evolutionNet.getBestNetworkMutable();

  SaveNetworkOnFile(*finalNetwork);
}
