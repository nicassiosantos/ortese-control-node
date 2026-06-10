// ============================================================================
//  mot_reader.cpp  -  implementacao do leitor de .mot/.sto do OpenSim
// ============================================================================
#include "control_node/mot_reader.hpp"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

namespace control_node
{

namespace
{
// divide uma linha em tokens por espaco/tab
std::vector<std::string> tokenize(const std::string & line)
{
  std::vector<std::string> out;
  std::istringstream iss(line);
  std::string tok;
  while (iss >> tok) out.push_back(tok);
  return out;
}
}  // namespace

MotData MotReader::read(const std::string & path, const std::string & column_name)
{
  MotData data;
  std::ifstream f(path);
  if (!f.is_open()) {
    data.error = "nao foi possivel abrir o arquivo: " + path;
    return data;
  }

  std::string line;
  bool in_degrees = false;
  bool header_done = false;

  // --- 1) percorre o cabecalho ate "endheader" ---
  while (std::getline(f, line)) {
    // normaliza para minusculas para deteccao
    std::string low = line;
    std::transform(low.begin(), low.end(), low.begin(), ::tolower);

    if (low.find("indegrees") != std::string::npos) {
      in_degrees = (low.find("yes") != std::string::npos);
    }
    if (low.find("endheader") != std::string::npos) {
      header_done = true;
      break;
    }
  }
  if (!header_done) {
    data.error = "cabecalho sem 'endheader' (arquivo .mot invalido?)";
    return data;
  }

  // --- 2) linha de nomes das colunas ---
  if (!std::getline(f, line)) {
    data.error = "arquivo terminou antes da linha de nomes das colunas";
    return data;
  }
  std::vector<std::string> names = tokenize(line);

  // encontra os indices de 'time' e da coluna pedida
  int time_idx = -1, col_idx = -1;
  for (size_t i = 0; i < names.size(); ++i) {
    std::string n = names[i];
    std::string nlow = n;
    std::transform(nlow.begin(), nlow.end(), nlow.begin(), ::tolower);
    if (nlow == "time") time_idx = static_cast<int>(i);
    if (n == column_name) col_idx = static_cast<int>(i);
  }
  if (time_idx < 0) { data.error = "coluna 'time' nao encontrada"; return data; }
  if (col_idx < 0) {
    data.error = "coluna '" + column_name + "' nao encontrada no .mot";
    return data;
  }

  // --- 3) le os dados ---
  const double deg2rad = M_PI / 180.0;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::vector<std::string> tok = tokenize(line);
    if (static_cast<int>(tok.size()) <= std::max(time_idx, col_idx)) continue;
    try {
      double t = std::stod(tok[time_idx]);
      double v = std::stod(tok[col_idx]);
      if (in_degrees) v *= deg2rad;
      data.time.push_back(t);
      data.values.push_back(v);
    } catch (...) {
      // linha nao numerica; ignora
    }
  }

  if (data.time.size() < 2) {
    data.error = "menos de 2 amostras de dados lidas";
    return data;
  }

  data.ok = true;
  return data;
}

}  // namespace control_node
