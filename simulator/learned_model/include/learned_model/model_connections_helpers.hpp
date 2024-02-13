#ifndef LEARNED_MODEL__MODEL_CONNECTIONS_HELPERS_HPP_
#define LEARNED_MODEL__MODEL_CONNECTIONS_HELPERS_HPP_

#include <cstring>
#include <vector>

std::vector<double> fillVectorUsingMap(
  std::vector<double> vector1, std::vector<double> vector2, std::vector<int> map, bool inverse);

std::vector<int> createConnectionsMap(
  std::vector<char *> connection_names_1, std::vector<char *> connection_names_2);

#endif  // LEARNED_MODEL__MODEL_CONNECTIONS_HELPERS_HPP_
